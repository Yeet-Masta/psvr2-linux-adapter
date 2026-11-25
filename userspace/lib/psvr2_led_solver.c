// SPDX-License-Identifier: GPL-2.0
/*
 * Sense controller pose from headset LED blobs. Behaviour from
 * docs/sony-re/overview.md §A.9; the numerics (numeric Jacobian, Nielsen damping
 * start, ray solve) are our own choices and are marked as such.
 */
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "psvr2_led_solver.h"
#include "psvr2_led_models.h"

#define DEG (180.0 / M_PI)

/* Sony's orientation-boot LED pairs (table at 0x180319380), logical LEDs. */
static const int boot_pairs[][2] = {
	{ 3, 4 }, { 5, 6 }, { 7, 8 }, { 8, 9 }, { 9, 10 }, { 10, 11 },
	{ 11, 12 }, { 12, 13 }, { 13, 14 }, { 0, 16 }, { 16, 15 },
};

/*
 * The table fits the 17-LED layout, where 0, 15, 16 are the back trio
 * (normals toward -z). Sony indexes it into every model unchanged, but on
 * the 20-LED rev 1 the back trio is 0, 18, 19 and 15/16 sit on the ring.
 * Our change: on 20-LED models read 15 as 18 and 16 as 19. In the
 * synthetic test this lifts rev 1 boot from 121/200 (12 wrong) to 195/200
 * (0 wrong).
 */
static int boot_led(const struct psvr2_led_model *model, int k)
{
	return model->count == 20 && (k == 15 || k == 16) ? k + 3 : k;
}

void psvr2_solver_default_cfg(struct psvr2_solver_cfg *cfg)
{
	cfg->search_radius_px = 3;
	cfg->pothole = 1;
	cfg->min_inliers_boot = 7;
	cfg->min_inliers_orientation = 4;
	cfg->max_iterations = 250;
	cfg->min_depth_mm = 5;
	cfg->max_depth_mm = 1500;
	cfg->own_accept = 1;
	cfg->sony_match = 1;
	cfg->few_led = 1;
	cfg->grazing_check = 1;
	cfg->edge_state = 1;
	cfg->inlier_px = 2;
	cfg->ransac_samples = 200;
	cfg->ransac_samples_in_track = 50;
	cfg->ransac_ring_ratio = 0.5;
}

static float bits_to_float(uint32_t b)
{
	float f;

	memcpy(&f, &b, sizeof(f));
	return f;
}

void psvr2_led_model_load(int rev, struct psvr2_led_model *model)
{
	const int m = rev >= 1 && rev <= 6 ? rev - 1 : 6;

	memset(model, 0, sizeof(*model));
	model->count = psvr2_led_models[m].count;
	model->cone_deg = psvr2_led_models[m].cone_deg;
	for (int i = 0; i < model->count; i++)
		for (int k = 0; k < 3; k++) {
			model->pos[i][k] = bits_to_float(psvr2_led_models[m].pos[i][k]);
			model->nrm[i][k] = bits_to_float(psvr2_led_models[m].nrm[i][k]);
		}
	model->rings = psvr2_led_models[m].rings;
	for (int r = 0; r < model->rings; r++) {
		model->ring_len[r] = psvr2_led_models[m].ring_len[r];
		for (int i = 0; i < model->ring_len[r]; i++)
			model->ring[r][i] = psvr2_led_models[m].ring[r][i];
	}
}

void psvr2_led_model_mirror_x(struct psvr2_led_model *model)
{
	for (int i = 0; i < model->count; i++) {
		model->pos[i][0] = -model->pos[i][0];
		model->nrm[i][0] = -model->nrm[i][0];
	}
	for (int r = 0; r < model->rings; r++)
		for (int i = 0, j = model->ring_len[r] - 1; i < j; i++, j--) {
			const int t = model->ring[r][i];
			model->ring[r][i] = model->ring[r][j];
			model->ring[r][j] = t;
		}
}

/* ---- small linear algebra (row-major 3x3) ------------------------------ */

static void mv(const double M[9], const double v[3], double o[3])
{
	for (int i = 0; i < 3; i++)
		o[i] = M[3 * i] * v[0] + M[3 * i + 1] * v[1] + M[3 * i + 2] * v[2];
}

static void mtv(const double M[9], const double v[3], double o[3])
{
	for (int i = 0; i < 3; i++)
		o[i] = M[i] * v[0] + M[3 + i] * v[1] + M[6 + i] * v[2];
}

static void mm(const double A[9], const double B[9], double C[9])
{
	double T[9];

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			T[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
	memcpy(C, T, sizeof(T));
}

static double dot(const double a[3], const double b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double norm(const double a[3]) { return sqrt(dot(a, a)); }

/* Rotation from an axis-angle vector (exp map). */
static void rodrigues(const double w[3], double R[9])
{
	const double th = norm(w);
	double k[3], c, s, v;

	if (th < 1e-12) {
		const double I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
		memcpy(R, I, sizeof(I));
		return;
	}
	k[0] = w[0] / th; k[1] = w[1] / th; k[2] = w[2] / th;
	c = cos(th); s = sin(th); v = 1 - c;
	R[0] = c + k[0] * k[0] * v;        R[1] = k[0] * k[1] * v - k[2] * s; R[2] = k[0] * k[2] * v + k[1] * s;
	R[3] = k[1] * k[0] * v + k[2] * s; R[4] = c + k[1] * k[1] * v;        R[5] = k[1] * k[2] * v - k[0] * s;
	R[6] = k[2] * k[0] * v - k[1] * s; R[7] = k[2] * k[1] * v + k[0] * s; R[8] = c + k[2] * k[2] * v;
}

/* Solve A x = b for a symmetric positive (semi)definite n x n, n <= 6. */
static int solve_spd(int n, double A[36], double b[6], double x[6])
{
	double L[36] = { 0 };

	for (int i = 0; i < n; i++)
		for (int j = 0; j <= i; j++) {
			double s = A[n * i + j];
			for (int k = 0; k < j; k++)
				s -= L[n * i + k] * L[n * j + k];
			if (i == j) {
				if (s <= 1e-18)
					return -1;
				L[n * i + i] = sqrt(s);
			} else {
				L[n * i + j] = s / L[n * j + j];
			}
		}
	for (int i = 0; i < n; i++) {
		double s = b[i];
		for (int k = 0; k < i; k++)
			s -= L[n * i + k] * x[k];
		x[i] = s / L[n * i + i];
	}
	for (int i = n - 1; i >= 0; i--) {
		double s = x[i];
		for (int k = i + 1; k < n; k++)
			s -= L[n * k + i] * x[k];
		x[i] = s / L[n * i + i];
	}
	return 0;
}

/* ---- projection ---------------------------------------------------------- */

static int project_point(const struct psvr2_rig_camera *cam, const double P[3],
			 double px[2], double *depth)
{
	double d[3] = { P[0] - cam->t[0], P[1] - cam->t[1], P[2] - cam->t[2] }, Xc[3];

	mtv(cam->R, d, Xc);	/* camera <- rig */
	if (depth)
		*depth = Xc[2];
	if (psvr2_cam_project_rig(&cam->model, Xc, px))
		return -1;
	return px[0] >= 0 && px[1] >= 0 && px[0] < cam->model.width && px[1] < cam->model.height ? 0 : -1;
}

int psvr2_led_project(const struct psvr2_rig_camera *cam,
		      const struct psvr2_led_model *model,
		      const struct psvr2_pose3 *pose, int led,
		      double px[2], double *view_deg, double *depth_mm)
{
	double P[3], n[3], v[3], len, a;

	mv(pose->R, model->pos[led], P);
	P[0] += pose->t[0]; P[1] += pose->t[1]; P[2] += pose->t[2];
	if (project_point(cam, P, px, depth_mm))
		return -1;
	mv(pose->R, model->nrm[led], n);
	v[0] = cam->t[0] - P[0]; v[1] = cam->t[1] - P[1]; v[2] = cam->t[2] - P[2];
	len = norm(v);
	if (len <= 0)
		return -1;
	a = acos(fmax(-1.0, fmin(1.0, dot(n, v) / len))) * DEG;	/* 0 = face-on */
	if (view_deg)
		*view_deg = a;
	return a < model->cone_deg ? 0 : -1;
}

/* ---- association and matching (docs/sony-re/overview.md §A.9.3 steps 2-3) --------------- */

struct vis {
	int n;
	int led[PSVR2_LED_MAX];
	double px[PSVR2_LED_MAX][2], angle[PSVR2_LED_MAX], depth[PSVR2_LED_MAX];
};

/* limit: the view-angle cut (Sony's association modes, FUN_1800daff0). */
static void associate(const struct psvr2_rig_camera *cam, const struct psvr2_led_model *model,
		      const struct psvr2_pose3 *pose, double limit, struct vis *v)
{
	struct vis all = { 0 };
	double origin_depth, opx[2];
	int i, j;

	v->n = 0;
	for (i = 0; i < model->count; i++) {
		const int k = all.n;
		all.angle[k] = 1e9;	/* stays when the LED does not project */
		psvr2_led_project(cam, model, pose, i, all.px[k], &all.angle[k], &all.depth[k]);
		if (all.angle[k] < limit)
			all.led[all.n++] = i;
	}
	project_point(cam, pose->t, opx, &origin_depth);
	for (i = 0; i < all.n; i++) {
		int culled = 0;
		/* Self-occlusion: within a 3 px box of a nearer LED, > 10 mm deeper. */
		if (origin_depth >= 120)
			for (j = 0; j < all.n && !culled; j++)
				culled = j != i && fabs(all.px[i][0] - all.px[j][0]) < 3 &&
					 fabs(all.px[i][1] - all.px[j][1]) < 3 &&
					 all.depth[i] > all.depth[j] + 10;
		if (culled)
			continue;
		/* Insert sorted by view angle, face-on first. */
		for (j = v->n; j > 0 && v->angle[j - 1] > all.angle[i]; j--) {
			v->led[j] = v->led[j - 1];
			v->angle[j] = v->angle[j - 1];
			v->depth[j] = v->depth[j - 1];
			memcpy(v->px[j], v->px[j - 1], sizeof(v->px[j]));
		}
		v->led[j] = all.led[i];
		v->angle[j] = all.angle[i];
		v->depth[j] = all.depth[i];
		memcpy(v->px[j], all.px[i], sizeof(v->px[j]));
		v->n++;
	}
}

/* Blobs within `gate` of (x, y), nearest first, at most `max`. */
static int nearest(const struct psvr2_blob_list *b, double x, double y, double gate,
		   int max, int out[])
{
	double d[PSVR2_SOLVER_BLOBS];
	int n = 0;

	for (int k = 0; k < b->count; k++) {
		const double d2 = (b->xy[k][0] - x) * (b->xy[k][0] - x) + (b->xy[k][1] - y) * (b->xy[k][1] - y);
		int j;
		if (d2 > gate * gate)
			continue;
		for (j = n < max ? n++ : max; j > 0 && d[j - 1] > d2; j--)
			if (j < max) {
				d[j] = d[j - 1];
				out[j] = out[j - 1];
			}
		if (j < max) {
			d[j] = d2;
			out[j] = k;
		}
	}
	return n;
}

/*
 * Gated match. LEDs sit ~6 px apart at half a metre, so a prediction a few
 * px off makes the nearest blob often a neighbour's, and one wrong pick
 * cascades. First find the image shift most LEDs agree on: candidates are
 * the up-to-7 nearest blobs (Sony's count) of the most face-on LEDs; the
 * consensus vote is our own addition. Then match face-on first, nearest
 * unused blob to the shifted prediction, and apply Sony's outlier cut
 * against the best match. Returns the match count.
 */
static int match(const struct vis *v, const struct psvr2_blob_list *b, double gate,
		 double origin_depth, int led_blob[PSVR2_LED_MAX], double dist[PSVR2_LED_MAX])
{
	int used[PSVR2_SOLVER_BLOBS] = { 0 }, n = 0, i, k, ref = -1, cand[7];
	const double f = origin_depth >= 150 ? 1.5 : fmin(25.0, 900.0 / fmax(origin_depth, 1));
	const double r_in = fmax(1.5, 0.25 * gate);
	double sx = 0, sy = 0, best_votes = -1;

	for (i = 0; i < PSVR2_LED_MAX; i++)
		led_blob[i] = -1;
	for (i = 0; i < v->n && i < 4; i++) {
		const int nc = nearest(b, v->px[i][0], v->px[i][1], gate, 7, cand);
		for (int c = 0; c < nc; c++) {
			const double tx = b->xy[cand[c]][0] - v->px[i][0], ty = b->xy[cand[c]][1] - v->px[i][1];
			double votes = 0;
			for (int j = 0; j < v->n; j++) {
				int nb;
				if (nearest(b, v->px[j][0] + tx, v->px[j][1] + ty, r_in, 1, &nb))
					votes += 1;
			}
			votes -= 1e-3 * hypot(tx, ty);	/* tie: the smaller shift */
			if (votes > best_votes) {
				best_votes = votes;
				sx = tx;
				sy = ty;
			}
		}
	}
	for (i = 0; i < v->n; i++) {
		int best;
		double dx, dy;
		const double x = v->px[i][0] + sx, y = v->px[i][1] + sy;
		double bd = 4 * r_in * r_in;
		best = -1;
		for (k = 0; k < b->count; k++) {
			const double d2 = (b->xy[k][0] - x) * (b->xy[k][0] - x) + (b->xy[k][1] - y) * (b->xy[k][1] - y);
			if (!used[k] && d2 <= bd) {
				bd = d2;
				best = k;
			}
		}
		if (best < 0)
			continue;
		dx = b->xy[best][0] - v->px[i][0];
		dy = b->xy[best][1] - v->px[i][1];
		if (dx * dx + dy * dy > gate * gate)
			continue;
		used[best] = 1;
		led_blob[v->led[i]] = best;
		dist[v->led[i]] = hypot(dx, dy);
		if (ref < 0 || dist[v->led[i]] < dist[ref])
			ref = v->led[i];
		n++;
	}
	if (ref >= 0)
		for (i = 0; i < PSVR2_LED_MAX; i++)
			if (i != ref && led_blob[i] >= 0 && dist[i] > f * dist[ref] && dist[i] > 3) {
				led_blob[i] = -1;
				n--;
			}
	return n;
}

/* ---- Sony's gated match (FUN_1800df5b0, docs/sony-re/controller-tracking.md §B.2.11) ---------------- */

/* Squared distance from q to the nearest blob other than `self`, or s2
 * when none is closer than s2 (FUN_1800f1ce0). */
static double nearest_other(const struct psvr2_blob_list *b, int self, double qx, double qy, double s2)
{
	double m = s2;

	for (int k = 0; k < b->count; k++) {
		const double dx = b->xy[k][0] - qx, dy = b->xy[k][1] - qy, d2 = dx * dx + dy * dy;
		if (k != self && d2 < m)
			m = d2;
	}
	return m;
}

/* Nearest blob to (x, y) not in `skip`, its squared distance in *d2; -1
 * when none (FUN_1800f0ee0). */
static int nearest_free(const struct psvr2_blob_list *b, double x, double y, const unsigned char *skip,
			double *d2)
{
	int best = -1;

	*d2 = 9999.9;
	for (int k = 0; k < b->count; k++) {
		const double dx = b->xy[k][0] - x, dy = b->xy[k][1] - y, e = dx * dx + dy * dy;
		if (!skip[k] && (best < 0 || e < *d2)) {
			best = k;
			*d2 = e;
		}
	}
	return best;
}

struct smatch {
	int blob[PSVR2_LED_MAX];	/* -1 = unmatched */
	double dist[PSVR2_LED_MAX];	/* px, prediction to blob */
	double score[PSVR2_LED_MAX];	/* support score, lower is better */
	int rank[PSVR2_LED_MAX];	/* support count */
};

/*
 * Support score of blob `cand` for visible LED `vi` (FUN_1800f0a50 and,
 * with anchors, FUN_1800f0cc0): the up-to-k neighbour LEDs (anchors first
 * when given, then the nearest visible LEDs in the image) must have a blob
 * at cand + (their prediction - this LED's). Score = mean squared distance
 * of the found ones, missing ones cost s2 / 2 each. *found = count.
 */
static double support(const struct vis *v, const struct psvr2_blob_list *b, int vi, int cand, double s2,
		      const int *anchors, int nanch, int *found)
{
	const int k = v->n - 2 > 2 ? (v->n - 2 < 5 ? v->n - 2 : 5) : 2;
	unsigned char taken[PSVR2_LED_MAX] = { 0 };
	double sum = 0;
	int ai = 0;

	*found = 0;
	taken[vi] = 1;
	for (int it = 0; it < 7; it++) {
		int j = -1;
		if (ai < nanch && ai < 5) {
			j = anchors[ai++];
		} else {
			double bd = DBL_MAX;
			for (int t = 0; t < v->n; t++) {
				const double dx = v->px[t][0] - v->px[vi][0], dy = v->px[t][1] - v->px[vi][1];
				if (!taken[t] && dx * dx + dy * dy < bd) {
					bd = dx * dx + dy * dy;
					j = t;
				}
			}
		}
		if (j < 0)
			break;
		taken[j] = 1;
		const double m = nearest_other(b, cand, b->xy[cand][0] + v->px[j][0] - v->px[vi][0],
					       b->xy[cand][1] + v->px[j][1] - v->px[vi][1], s2);
		if (m != s2) {
			sum += m;
			if (++*found >= k)
				break;
		}
	}
	if (*found < k)
		sum += (k - *found) * s2 * 0.5;
	return *found ? sum / *found : sum;
}

/* One-blob-two-LEDs and outlier cut (FUN_1800f1340). Returns the drops. */
static int cleanup(const struct vis *v, struct smatch *s, double depth)
{
	int drops = 0, ref = -1, best_rank = 0;
	double best = DBL_MAX, f;

	for (int i = 0; i < v->n; i++) {
		if (s->blob[i] < 0)
			continue;
		for (int j = 0; j < v->n && s->blob[i] >= 0; j++) {
			if (j == i || s->blob[j] < 0 || s->blob[j] != s->blob[i])
				continue;
			const int ri = s->rank[i], rj = s->rank[j];
			if (rj <= ri && s->score[j] >= s->score[i] && s->dist[j] >= s->dist[i]) {
				s->blob[j] = -1;	/* j dominated */
				drops++;
			} else if (rj < ri || s->score[i] < s->score[j] || s->dist[i] < s->dist[j]) {
				s->blob[i] = s->blob[j] = -1;	/* neither wins */
				drops += 2;
			} else {
				s->blob[i] = -1;	/* i dominated */
				drops++;
			}
		}
	}
	for (int i = 0; i < v->n; i++)
		if (s->blob[i] >= 0 && s->rank[i] >= best_rank && s->dist[i] * s->score[i] < best) {
			best = s->dist[i] * s->score[i];
			best_rank = s->rank[i];
			ref = i;
		}
	f = depth < 150 ? fmin(25.0, 1.5 * 4 * 150 / fmax(depth, 1e-3)) : 1.5;
	if (ref >= 0)
		for (int i = 0; i < v->n; i++)
			if (s->blob[i] >= 0 && s->dist[i] > f * s->dist[ref] && s->dist[i] > 3) {
				s->blob[i] = -1;
				drops++;
			}
	return drops;
}

/*
 * Three passes: (1) each visible LED, face-on first, takes the best-
 * supported of its up to 7 nearest blobs within `gate`; (2) LEDs still
 * free are re-scored against the 5 best-matched LEDs as anchors, on blobs
 * not yet taken; (3) LEDs still free take the nearest free blob when it is
 * within 1.5x the mean match distance. Cleanup after each pass. `s_rad`:
 * the support radius. Needs 2 supporting neighbours (Sony: 3 when the
 * pose is flagged close; that flag is not modelled).
 */
static int match_sony(const struct vis *v, const struct psvr2_blob_list *b, double gate, double s_rad,
		      double depth, int led_blob[PSVR2_LED_MAX], double dist[PSVR2_LED_MAX])
{
	struct smatch s;
	const double g2 = gate * gate, s2 = s_rad * s_rad;
	const int need = 2;
	int anchors[5], nanch = 0, n = 0;
	unsigned char used[PSVR2_SOLVER_BLOBS] = { 0 };

	for (int i = 0; i < v->n; i++)
		s.blob[i] = -1;
	for (int pass = 0; pass < 2; pass++) {
		for (int i = 0; i < v->n; i++) {
			unsigned char tried[PSVR2_SOLVER_BLOBS];
			double best = 99999.0, d2;
			int pick = -1, rank = 0;
			if (s.blob[i] >= 0)
				continue;
			memcpy(tried, used, sizeof(tried));
			for (int t = 0; t < 7; t++) {
				int found;
				const int c = nearest_free(b, v->px[i][0], v->px[i][1], tried, &d2);
				if (c < 0 || d2 > g2)
					break;
				tried[c] = 1;
				const double sc = support(v, b, i, c, s2, anchors, pass ? nanch : 0, &found);
				if (sc < best && found >= need) {
					best = sc;
					pick = c;
					rank = found;
				}
			}
			if (pick >= 0) {
				s.blob[i] = pick;
				s.dist[i] = hypot(b->xy[pick][0] - v->px[i][0], b->xy[pick][1] - v->px[i][1]);
				s.score[i] = best;
				s.rank[i] = rank;
			}
		}
		cleanup(v, &s, depth);
		/* Anchors: the matched LEDs with the smallest distances, ascending. */
		memset(used, 0, sizeof(used));
		nanch = 0;
		for (double last = 0; nanch < 5;) {
			int pick = -1;
			for (int i = 0; i < v->n; i++)
				if (s.blob[i] >= 0 && s.dist[i] > last && (pick < 0 || s.dist[i] < s.dist[pick]))
					pick = i;
			if (pick < 0)
				break;
			anchors[nanch++] = pick;
			last = s.dist[pick];
		}
		for (int i = 0; i < v->n; i++)
			if (s.blob[i] >= 0)
				used[s.blob[i]] = 1;
	}
	{
		double mean = 0, d2;
		int m = 0;
		for (int i = 0; i < v->n; i++)
			if (s.blob[i] >= 0) {
				mean += s.dist[i];
				m++;
			}
		mean = m ? mean / m : 0;
		for (int i = 0; i < v->n; i++) {
			if (s.blob[i] >= 0)
				continue;
			const int c = nearest_free(b, v->px[i][0], v->px[i][1], used, &d2);
			if (c >= 0 && d2 < (1.5 * mean) * (1.5 * mean)) {
				s.blob[i] = c;
				s.dist[i] = sqrt(d2);
				s.score[i] = d2;
				s.rank[i] = 0;	/* `used` is not updated here (Sony) */
			}
		}
		cleanup(v, &s, depth);
	}
	for (int i = 0; i < PSVR2_LED_MAX; i++)
		led_blob[i] = -1;
	for (int i = 0; i < v->n; i++)
		if (s.blob[i] >= 0) {
			led_blob[v->led[i]] = s.blob[i];
			dist[v->led[i]] = s.dist[i];
			n++;
		}
	return n;
}

/* ---- refine: weighted LM with Huber IRLS (docs/sony-re/overview.md §A.9.3 step 7) -------- */

struct problem {
	const struct psvr2_rig_camera *cams;
	const struct psvr2_blob_list *blobs;
	const struct psvr2_led_model *model;
	int led_blob[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];
	int n;			/* matched pairs */
	int expected;		/* associated LEDs clearly facing a camera (< 70 deg) */
	uint64_t prev[PSVR2_SOLVER_CAMS];	/* LEDs matched last frame (FUN_1800f1950) */
};

/* Residuals (projected - blob), 2 per pair; 0 when a point fails to project. */
static void residuals(const struct problem *p, const struct psvr2_pose3 *pose, double *r)
{
	int m = 0;

	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++)
		for (int l = 0; l < p->model->count; l++) {
			const int b = p->led_blob[c][l];
			const struct psvr2_rig_camera *cam = &p->cams[c];
			double P[3], d[3], Xc[3], px[2];
			if (b < 0)
				continue;
			mv(pose->R, p->model->pos[l], P);
			for (int i = 0; i < 3; i++)
				d[i] = P[i] + pose->t[i] - cam->t[i];
			mtv(cam->R, d, Xc);
			/* No image bounds here: a pair near the edge keeps its residual. */
			if (psvr2_cam_project_rig(&cam->model, Xc, px) == 0) {
				r[m] = px[0] - p->blobs[c].xy[b][0];
				r[m + 1] = px[1] - p->blobs[c].xy[b][1];
			} else {
				r[m] = r[m + 1] = 0;
			}
			m += 2;
		}
}

static void apply(const struct psvr2_pose3 *in, const double d[6], struct psvr2_pose3 *out)
{
	double dR[9];

	rodrigues(d, dR);
	mm(dR, in->R, out->R);
	for (int i = 0; i < 3; i++)
		out->t[i] = in->t[i] + d[3 + i];
}

static int cmp_d(const void *a, const void *b)
{
	const double x = *(const double *)a, y = *(const double *)b;
	return (x > y) - (x < y);
}

/* Huber weights per pair, scale 1.483 * median |r| (floor 0.25 px);
 * exactly 3 pairs: plain least squares. */
static void weights(const double *r, int n, double *w)
{
	double mag[2 * PSVR2_SOLVER_CAMS * PSVR2_LED_MAX], s, k;

	for (int i = 0; i < n; i++)
		mag[i] = hypot(r[2 * i], r[2 * i + 1]);
	if (n == 3) {
		for (int i = 0; i < n; i++)
			w[i] = 1;
		return;
	}
	double sorted[2 * PSVR2_SOLVER_CAMS * PSVR2_LED_MAX];
	memcpy(sorted, mag, sizeof(double) * n);
	qsort(sorted, n, sizeof(double), cmp_d);
	s = fmax(1.483 * sorted[n / 2], 0.25);
	k = 1.0 * s;
	for (int i = 0; i < n; i++)
		w[i] = mag[i] <= k ? 1 : k / mag[i];
}

static double cost(const double *r, const double *w, int n)
{
	double c = 0;

	for (int i = 0; i < n; i++)
		c += w[i] * (r[2 * i] * r[2 * i] + r[2 * i + 1] * r[2 * i + 1]);
	return c;
}

static int refine(const struct problem *p, int max_iter, struct psvr2_pose3 *pose)
{
	enum { MAXR = 2 * PSVR2_SOLVER_CAMS * PSVR2_LED_MAX };
	double r[MAXR], r2[MAXR], w[MAXR / 2], J[MAXR][6];
	double mu = -1, nu = 2, c;
	const int n = p->n, m = 2 * n;

	if (n < 3)
		return -1;
	residuals(p, pose, r);
	weights(r, n, w);
	c = cost(r, w, n);
	for (int it = 0; it < max_iter; it++) {
		double A[36] = { 0 }, g[6] = { 0 }, d[6], dmax = 0;
		/* Numeric Jacobian (our choice): 1e-5 rad, 1e-3 mm. */
		for (int j = 0; j < 6; j++) {
			double e[6] = { 0 };
			struct psvr2_pose3 q;
			e[j] = j < 3 ? 1e-5 : 1e-3;
			apply(pose, e, &q);
			residuals(p, &q, r2);
			for (int i = 0; i < m; i++)
				J[i][j] = (r2[i] - r[i]) / e[j];
		}
		for (int i = 0; i < m; i++) {
			const double wi = w[i / 2];
			for (int a = 0; a < 6; a++) {
				g[a] += wi * J[i][a] * r[i];
				for (int b = 0; b < 6; b++)
					A[6 * a + b] += wi * J[i][a] * J[i][b];
			}
		}
		for (int a = 0; a < 6; a++)
			dmax = fmax(dmax, A[7 * a]);
		if (mu < 0)
			mu = 1e-3 * dmax;	/* Nielsen's start (our choice) */
		for (int tries = 0; tries < 100; tries++) {
			double B[36], ng[6], c2, pred = 0, dn = 0, xn;
			struct psvr2_pose3 q;
			memcpy(B, A, sizeof(B));
			for (int a = 0; a < 6; a++) {
				B[7 * a] += mu * fmax(A[7 * a], 1e-9);
				ng[a] = -g[a];
			}
			if (solve_spd(6, B, ng, d))
				return -1;
			for (int a = 0; a < 6; a++) {
				pred += d[a] * (mu * fmax(A[7 * a], 1e-9) * d[a] - g[a]);
				dn += d[a] * d[a];
			}
			xn = dot(pose->t, pose->t) + 1;
			if (dn <= 1e-16 * xn)
				return 0;	/* converged */
			apply(pose, d, &q);
			residuals(p, &q, r2);
			c2 = cost(r2, w, n);
			if (!isfinite(c2))
				return -1;
			if (c2 < c) {
				const double rho = (c - c2) / fmax(pred, 1e-30);
				const double t = 2 * rho - 1;
				*pose = q;
				memcpy(r, r2, sizeof(double) * m);
				weights(r, n, w);	/* IRLS */
				c = cost(r, w, n);
				mu *= fmax(1.0 / 3, 1 - t * t * t);
				nu = 2;
				break;
			}
			mu *= nu;
			nu *= 2;
			if (tries == 99)
				return 0;
		}
		if (c < 1e-12)
			return 0;
	}
	return 0;
}

/* ---- tracking from a prediction ------------------------------------------ */

static double scale_for(const struct psvr2_pose3 *pose)
{
	const double d_m = norm(pose->t) * 0.001;

	return fmin(1.0 / fmax(d_m, 1e-3), 10.0);
}

static int match_nearest(const struct vis *v, const struct psvr2_blob_list *b, double gate, uint64_t prev,
			 int led_blob[PSVR2_LED_MAX], double dist[PSVR2_LED_MAX]);

/* s_rad: Sony's support radius; mode 1: match_sony, 0: match, 2:
 * match_nearest; limit: the view-angle cut. */
static int match_all_ex(const struct psvr2_rig_camera cams[], const struct psvr2_blob_list blobs[],
			const struct psvr2_led_model *model, const struct psvr2_pose3 *pose, double gate,
			double s_rad, int mode, double limit, struct problem *p,
			double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX])
{
	p->n = p->expected = 0;
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		struct vis v;
		double opx[2], od = 0;
		for (int l = 0; l < PSVR2_LED_MAX; l++)
			p->led_blob[c][l] = -1;
		if (!cams[c].active || !blobs[c].count)
			continue;
		associate(&cams[c], model, pose, limit, &v);
		for (int i = 0; i < v.n; i++)
			p->expected += v.angle[i] < 70;
		project_point(&cams[c], pose->t, opx, &od);
		p->n += mode == 1 ? match_sony(&v, &blobs[c], gate, s_rad, od, p->led_blob[c], dist[c])
			: mode == 2 ? match_nearest(&v, &blobs[c], gate, p->prev[c], p->led_blob[c], dist[c])
				    : match(&v, &blobs[c], gate, od, p->led_blob[c], dist[c]);
	}
	return p->n;
}

static int match_all(const struct psvr2_rig_camera cams[], const struct psvr2_blob_list blobs[],
		     const struct psvr2_led_model *model, const struct psvr2_pose3 *pose, double gate,
		     double s_rad, int sony, struct problem *p, double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX])
{
	return match_all_ex(cams, blobs, model, pose, gate, s_rad, sony ? 1 : 0, model->cone_deg, p, dist);
}

static void finish(const struct problem *p, const struct psvr2_pose3 *pose, double T,
		   struct psvr2_solve_result *res)
{
	double r[2 * PSVR2_SOLVER_CAMS * PSVR2_LED_MAX], s = 0, e = 0;

	residuals(p, pose, r);
	for (int i = 0; i < p->n; i++) {
		const double d2 = r[2 * i] * r[2 * i] + r[2 * i + 1] * r[2 * i + 1];
		e += d2;
		if (d2 <= T)
			s += d2 - T;	/* MSAC */
	}
	res->matched = p->n;
	res->score = p->n ? s / T : 0;
	res->rms_px = p->n ? sqrt(e / p->n) : 0;
	memcpy(res->led_blob, p->led_blob, sizeof(res->led_blob));
}

/* Angle between two rotations, degrees. */
static double rot_angle_deg(const double A[9], const double B[9])
{
	double tr = 0;

	for (int i = 0; i < 9; i++)
		tr += A[i] * B[i];
	return acos(fmax(-1.0, fmin(1.0, (tr - 1) / 2))) * DEG;
}

int psvr2_led_pose_check(const struct psvr2_solver_cfg *cfg,
			 const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			 const struct psvr2_led_model *model,
			 const struct psvr2_imu_prior *prior,
			 const struct psvr2_pose3 *pose,
			 const struct psvr2_solve_result *res, int close)
{
	if (prior && prior->gravity_valid && prior->gravity_age_s < 10) {
		double g[3];
		mv(pose->R, prior->g_model, g);
		if (dot(g, prior->g_rig) <= cos(0.3490658503988659))	/* 20 deg */
			return 2;
	}
	/* FUN_1800e1d50: 30 deg (0x18031998c), prior < 5 s old, not while the
	 * controller turns at >= 180 deg/s. */
	if (prior && prior->orient_valid && prior->orient_age_s < 5 && prior->spin_rad_s < M_PI &&
	    rot_angle_deg(pose->R, prior->R) > 30)
		return 1;
	if (model->count > 0)
		for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
			double P[3], d[3], z;
			int any = 0;
			if (!cams[c].active)
				continue;
			for (int l = 0; l < model->count && !any; l++)
				any = res && res->led_blob[c][l] >= 0;
			if (!any)
				continue;
			mv(pose->R, model->pos[0], P);
			for (int i = 0; i < 3; i++)
				d[i] = P[i] + pose->t[i] - cams[c].t[i];
			z = cams[c].R[2] * d[0] + cams[c].R[5] * d[1] + cams[c].R[8] * d[2];
			if (z > cfg->max_depth_mm || (!close && z < cfg->min_depth_mm))
				return 3;
		}
	return 0;
}

double psvr2_led_support(const struct psvr2_solver_cfg *cfg,
			 const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			 const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			 const struct psvr2_led_model *model,
			 const struct psvr2_pose3 *pose,
			 const struct psvr2_solve_result *res)
{
	const double radius = cfg->search_radius_px * scale_for(pose);
	double sum = 0;
	int ncam = 0;

	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		int vis = 0, m = 0;
		double nd = 0;
		if (!cams[c].active)
			continue;
		for (int l = 0; l < model->count; l++) {
			double px[2], va;
			if (psvr2_led_project(&cams[c], model, pose, l, px, &va, NULL) || va >= 90 ||
			    px[0] < 0 || px[1] < 0 || px[0] >= cams[c].model.width || px[1] >= cams[c].model.height)
				continue;
			vis++;
			const int k = res->led_blob[c][l];
			if (k >= 0 && k < blobs[c].count) {
				m++;
				nd += fmin(1.0, hypot(blobs[c].xy[k][0] - px[0], blobs[c].xy[k][1] - px[1]) / radius);
			}
		}
		if (vis < 2)
			continue;
		ncam++;
		sum += m ? (1.0 - nd / m) * m / vis : 0.0;
	}
	return ncam ? sum / ncam : 0.0;
}

static int unproject(const struct psvr2_rig_camera *cam, const double px[2], double ray[3]);

/*
 * Sony's pair depth (FUN_1800e0d20 inline, and FUN_1800d1240 with the
 * roles swapped). Camera frame, rays (x, y, 1). LED a on ray ra, LED b on
 * ray rb, D = R p_b - R p_a: z_b (ra - rb) = D.z ra - D.xy, solved for
 * 1 / z_b over the image-plane residual; z_a = z_b - D.z. Returns
 * t = z_a (ra, 1) - R p_a, i.e. LED a exactly on its ray. -1 if degenerate.
 */
static int pair_depth(const double ra[2], const double rb[2], const double Rpa[3], const double Rpb[3],
		      double t[3])
{
	const double D[3] = { Rpb[0] - Rpa[0], Rpb[1] - Rpa[1], Rpb[2] - Rpa[2] };
	const double u[2] = { D[0] - D[2] * ra[0], D[1] - D[2] * ra[1] };
	const double w[2] = { ra[0] - rb[0], ra[1] - rb[1] };
	const double uu = u[0] * u[0] + u[1] * u[1];
	double inv, za;

	if (!(uu > 1e-12))
		return -1;
	inv = -(w[0] * u[0] + w[1] * u[1]) / uu;
	if (!(fabs(inv) > 1e-12))
		return -1;
	za = 1.0 / inv - D[2];
	t[0] = za * ra[0] - Rpa[0];
	t[1] = za * ra[1] - Rpa[1];
	t[2] = za - Rpa[2];
	return 0;
}

static int unproject_xy(const struct psvr2_cam_model *m, const double px[2], double xy[2]);

/*
 * Sony's few-LED solve (FUN_1800dcf00 -> FUN_1800e0d20): rotation = the
 * IMU orientation prior; translation from the matched LEDs' blob rays.
 * One LED: the point on its ray nearest the predicted LED position (P1P,
 * no distance gate, as Sony). 2+ LEDs in a camera: every pair gives two
 * translations (each LED put exactly on its ray, FUN_1800d1240 for the
 * second); each within 100 mm of the prediction counts for both LEDs. An
 * LED's cost = sum over its pairs of |t1 - t2|^2 / |p_a - p_b| (9999 when
 * not both counted), floor 0.01; the result = the LEDs' mean translations
 * weighted by 1 / cost. Sony runs this per camera; here every camera's
 * pairs pool, and rays only in different cameras (no in-camera pair) use
 * the least-squares ray intersection, gated at 100 mm. Not modelled:
 * Sony drops blobs touching the image border (no blob size here).
 * Returns the LED count used, -1 = no pose.
 */
static int few_led_solve(const struct psvr2_rig_camera cams[], const struct psvr2_blob_list blobs[],
			 const struct psvr2_led_model *model, const struct problem *p,
			 const struct psvr2_pose3 *predicted, const double R[9], struct psvr2_pose3 *out)
{
	enum { MAXE = PSVR2_SOLVER_CAMS * PSVR2_LED_MAX };
	double xy[MAXE][2], Rp[MAXE][3], sum[MAXE][3] = { { 0 } }, cost[MAXE] = { 0 }, tp[PSVR2_SOLVER_CAMS][3];
	int cam[MAXE], led[MAXE], cnt[MAXE] = { 0 }, n = 0, accepted = 0, pairs = 0;

	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		double v[3];
		for (int i = 0; i < 3; i++)
			v[i] = predicted->t[i] - cams[c].t[i];
		mtv(cams[c].R, v, tp[c]);
		for (int l = 0; l < model->count && n < MAXE; l++) {
			const int k = p->led_blob[c][l];
			double w[3];
			if (k < 0 || k >= blobs[c].count || unproject_xy(&cams[c].model, blobs[c].xy[k], xy[n]))
				continue;
			if (blobs[c].major[k] > 0) {
				/* Sony: the blob box, centre +- size x pi/2, must lie in
				 * the image (size = blob list +0x28, the LD major axis
				 * [likely]). */
				const double r = blobs[c].major[k] * M_PI / 2, *b = blobs[c].xy[k];
				if (b[0] - r < 0 || b[1] - r < 0 || b[0] + r >= cams[c].model.width ||
				    b[1] + r >= cams[c].model.height)
					continue;
			}
			mv(R, model->pos[l], w);
			mtv(cams[c].R, w, Rp[n]);
			cam[n] = c;
			led[n] = l;
			n++;
		}
	}
	if (n == 0)
		return -1;
	memcpy(out->R, R, sizeof(out->R));
	if (n == 1) {
		const double r[3] = { xy[0][0], xy[0][1], 1 };
		double v[3], tc[3], s;
		for (int i = 0; i < 3; i++)
			v[i] = Rp[0][i] + tp[cam[0]][i];
		s = dot(r, v) / dot(r, r);
		for (int i = 0; i < 3; i++)
			tc[i] = s * r[i] - Rp[0][i];
		mv(cams[cam[0]].R, tc, out->t);
		for (int i = 0; i < 3; i++)
			out->t[i] += cams[cam[0]].t[i];
		return 1;
	}
	for (int a = 0; a < n; a++)
		for (int b = a + 1; b < n; b++) {
			const double dx = xy[b][0] - xy[a][0], dy = xy[b][1] - xy[a][1];
			double t1[3], t2[3], e[3], c = 9999.0;
			int ok1 = 0, ok2 = 0;
			if (cam[a] != cam[b] || dx * dx + dy * dy <= 0.001)
				continue;
			pairs++;
			if (!pair_depth(xy[a], xy[b], Rp[a], Rp[b], t1)) {
				for (int i = 0; i < 3; i++)
					e[i] = t1[i] - tp[cam[a]][i];
				ok1 = dot(e, e) < 10000.0;	/* 100 mm */
			}
			if (!pair_depth(xy[b], xy[a], Rp[b], Rp[a], t2)) {
				for (int i = 0; i < 3; i++)
					e[i] = t2[i] - tp[cam[a]][i];
				ok2 = dot(e, e) < 10000.0;
			}
			for (int i = 0; i < 3; i++) {
				const double add = (ok1 ? t1[i] : 0) + (ok2 ? t2[i] : 0);
				sum[a][i] += add;
				sum[b][i] += add;
			}
			cnt[a] += ok1 + ok2;
			cnt[b] += ok1 + ok2;
			accepted += ok1 + ok2;
			if (ok1 && ok2) {
				double dp[3], dt[3];
				for (int i = 0; i < 3; i++) {
					dp[i] = model->pos[led[a]][i] - model->pos[led[b]][i];
					dt[i] = t1[i] - t2[i];
				}
				if (norm(dp) > 0)
					c = dot(dt, dt) / norm(dp);
			}
			cost[a] += c;
			cost[b] += c;
		}
	if (pairs == 0) {
		/* Rays only in different cameras: least-squares intersection. */
		double A[36] = { 0 }, bv[6] = { 0 }, x[6], e[3];
		for (int q = 0; q < n; q++) {
			const double r[3] = { xy[q][0], xy[q][1], 1 };
			const struct psvr2_rig_camera *C = &cams[cam[q]];
			double d[3], w[3], o[3], dn;
			mv(C->R, r, d);
			dn = norm(d);
			for (int i = 0; i < 3; i++)
				d[i] /= dn;
			mv(C->R, Rp[q], w);	/* R p, rig frame */
			for (int i = 0; i < 3; i++)
				o[i] = C->t[i] - w[i];
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++) {
					A[3 * i + j] += (i == j) - d[i] * d[j];
					bv[i] += ((i == j) - d[i] * d[j]) * o[j];
				}
		}
		if (solve_spd(3, A, bv, x))
			return -1;
		memcpy(out->t, x, sizeof(double) * 3);
		for (int i = 0; i < 3; i++)
			e[i] = out->t[i] - predicted->t[i];
		return dot(e, e) < 10000.0 ? n : -1;
	}
	if (accepted == 0)
		return -1;
	{
		/* Camera-frame means to the rig frame, weighted by 1 / cost. */
		double S = 0, W = 0, t[3] = { 0 };
		for (int q = 0; q < n; q++) {
			cost[q] = fmax(cost[q], 0.01);
			S += cost[q];
		}
		for (int q = 0; q < n; q++)
			if (cnt[q])
				W += S / cost[q];
		if (!(W > 0))
			return -1;
		for (int q = 0; q < n; q++) {
			double tw[3];
			if (!cnt[q])
				continue;
			const double f = S / cost[q] / cnt[q] / W;
			mv(cams[cam[q]].R, sum[q], tw);
			for (int i = 0; i < 3; i++)
				t[i] += f * (tw[i] + cnt[q] * cams[cam[q]].t[i]);
		}
		memcpy(out->t, t, sizeof(t));
	}
	return n;
}

/*
 * FUN_1800dcf00's choice: the IMU-rotation solve when <= 3 LEDs matched,
 * the pose < 700 mm away and the IMU orientation is set. Sony keeps that
 * mode (+0x4c98) until 16 frames with >= 5 matches (+0x4c9c), or until
 * the IMU orientation goes; mem NULL = no memory (each frame decides).
 */
static int few_decide(const struct psvr2_solver_cfg *cfg, const struct psvr2_imu_prior *prior,
		      const struct psvr2_pose3 *pose, int n, struct psvr2_track_mem *mem)
{
	struct psvr2_track_mem tmp = { 0 };

	if (!mem)
		mem = &tmp;
	if (!cfg->few_led || !prior || !prior->orient_valid) {
		mem->few_mode = 0;
		return 0;
	}
	if (mem->few_mode) {
		if (n >= 5 && ++mem->few_good > 15) {
			mem->few_mode = 0;
			return 0;
		}
		return 1;
	}
	if (n > 3 || norm(pose->t) >= 700)
		return 0;
	mem->few_mode = 1;
	mem->few_good = 0;
	return 1;
}

/*
 * FUN_1800dcf00: the few-LED solve (FUN_1800e0d20; P1P at most 6 frames
 * in a row) or the refine (FUN_1800dd2f0, >= 3 matches), on p's matches.
 * res->method 2/3 or 0.
 */
static int few_or_refine(const struct psvr2_solver_cfg *cfg, const struct psvr2_rig_camera cams[],
			 const struct psvr2_blob_list blobs[], const struct psvr2_led_model *model,
			 struct problem *p, const struct psvr2_pose3 *predicted,
			 const struct psvr2_imu_prior *prior, struct psvr2_track_mem *mem,
			 struct psvr2_pose3 *pose, struct psvr2_solve_result *res)
{
	const int n = p->n;

	res->method = 0;
	if (few_decide(cfg, prior, pose, n, mem)) {
		const int p1p = mem ? mem->p1p_frames : prior->p1p_frames;
		int m;
		if (n < 1 || (n == 1 && p1p > 5))
			return -1;
		m = few_led_solve(cams, blobs, model, p, predicted, prior->R, pose);
		if (m < 1 || (m == 1 && p1p > 5))
			return -1;
		res->method = m == 1 ? 3 : 2;
		if (mem)
			mem->p1p_frames = m == 1 ? p1p + 1 : 0;
	} else {
		if (n < 3 || refine(p, cfg->max_iterations, pose))
			return -1;
		if (mem)
			mem->p1p_frames = 0;
	}
	finish(p, pose, cfg->pothole * scale_for(pose), res);
	res->expected = p->expected;
	return 0;
}

/* Our acceptance check (cfg->own_accept): a full solve that explains under
 * 3/4 of the LEDs facing the cameras, or any solve leaving more than
 * 1.5 px rms, is a wrong local minimum. 1 = reject. */
static int own_reject(const struct psvr2_solver_cfg *cfg, const struct problem *p,
		      const struct psvr2_solve_result *res)
{
	if (!cfg->own_accept)
		return 0;
	if (res->rms_px > 1.5)
		return 1;
	return !res->method && (p->n < cfg->min_inliers_orientation || 4 * p->n < 3 * p->expected);
}

/*
 * State 0 (FUN_1800db980): round 0 gated match + refine (FUN_1800d9d50)
 * at SEARCHRADIUS x scale x 3 x (dt / 16.7 ms); its failure keeps the
 * prediction (Sony goes on). Round 1 at 1.5 x POTHOLE x scale: < 3
 * matches fail (code 8). Then FUN_1800dcf00 and the pose check.
 */
static int track_normal(const struct psvr2_solver_cfg *cfg, const struct psvr2_rig_camera cams[],
			const struct psvr2_blob_list blobs[], const struct psvr2_led_model *model,
			const struct psvr2_pose3 *predicted, const struct psvr2_imu_prior *prior, double dt_s,
			struct psvr2_track_mem *mem, struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	struct problem p = { .cams = cams, .blobs = blobs, .model = model };
	double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];
	struct psvr2_pose3 pose = *predicted;
	double sc = scale_for(&pose);
	const double frames = fmin(fmax(dt_s * 60.0, 0.0), 10.0);

	if (mem)
		memcpy(p.prev, mem->prev, sizeof(p.prev));
	res->method = 0;
	{
		const double gate = cfg->search_radius_px * sc * 3.0 * frames;
		struct psvr2_pose3 r0 = pose;
		if (match_all(cams, blobs, model, &pose, fmax(gate, 1.0), cfg->search_radius_px * sc, cfg->sony_match,
			      &p, dist) >= 3 && refine(&p, cfg->max_iterations, &r0) == 0)
			pose = r0;
		sc = scale_for(&pose);
	}
	{
		const double gate = 1.5 * cfg->pothole * sc;
		if (match_all(cams, blobs, model, &pose, fmax(gate, 1.0), gate, cfg->sony_match, &p, dist) < 3)
			return -1;
	}
	if (few_or_refine(cfg, cams, blobs, model, &p, predicted, prior, mem, &pose, res))
		return -1;
	if (psvr2_led_pose_check(cfg, cams, model, prior, &pose, res, norm(pose.t) < 150) || own_reject(cfg, &p, res))
		return -1;
	*out = pose;
	return 0;
}

int psvr2_led_track_ex(const struct psvr2_solver_cfg *cfg,
		       const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
		       const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
		       const struct psvr2_led_model *model,
		       const struct psvr2_pose3 *predicted,
		       const struct psvr2_imu_prior *prior, double dt_s,
		       struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	return track_normal(cfg, cams, blobs, model, predicted, prior, dt_s, NULL, out, res);
}

int psvr2_led_track(const struct psvr2_solver_cfg *cfg,
		    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
		    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
		    const struct psvr2_led_model *model,
		    const struct psvr2_pose3 *predicted,
		    struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	return psvr2_led_track_ex(cfg, cams, blobs, model, predicted, NULL, 1.0 / 60, out, res);
}

/* ---- close-range and edge states (FUN_1800db690) ------------------------ */

/* Camera distance of the controller origin: the nearest camera, among
 * those that matched LEDs when res is given (Sony: per-camera trackers,
 * |translation| in that camera's frame). */
static double cam_dist(const struct psvr2_rig_camera cams[], const struct psvr2_pose3 *pose,
		       const struct psvr2_solve_result *res, int nleds)
{
	double best = DBL_MAX;

	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		double v[3];
		int used = !res;
		if (!cams[c].active)
			continue;
		for (int l = 0; l < nleds && !used; l++)
			used = res->led_blob[c][l] >= 0;
		if (!used)
			continue;
		for (int i = 0; i < 3; i++)
			v[i] = pose->t[i] - cams[c].t[i];
		best = fmin(best, norm(v));
	}
	return best;
}

/*
 * FUN_1800db420: LEDs visible at cone x 0.8 (association mode 3) whose
 * prediction lies 20 px inside the image and within 1.2391838 fx of the
 * principal point. Summed over the cameras (Sony: per camera).
 */
static int inner_count(const struct psvr2_rig_camera cams[], const struct psvr2_led_model *model,
		       const struct psvr2_pose3 *pose)
{
	int n = 0;

	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		const struct psvr2_cam_model *m = &cams[c].model;
		const double r = 1.2391838 * m->fx;
		struct vis v;
		if (!cams[c].active)
			continue;
		associate(&cams[c], model, pose, model->cone_deg * 0.8, &v);
		for (int i = 0; i < v.n; i++) {
			const double x = v.px[i][0], y = v.px[i][1], dx = x - m->cx, dy = y - m->cy;
			n += x >= 20 && x < m->width - 20 && y >= 20 && y < m->height - 20 && dx * dx + dy * dy <= r * r;
		}
	}
	return n;
}

/*
 * FUN_1800ddb90 + FUN_1800f1950: every visible LED takes its nearest blob
 * within `gate` and remembers its second; then for two LEDs i < j (in the
 * view-angle order) on one blob:
 *  - squared distances >= 400 px^2 apart: the farther LED drops;
 *  - neither matched last frame (`prev`): j drops;
 *  - else, both with a second blob: the one whose second is nearer moves
 *    to it (Sony re-checks nothing after a move);
 *  - else the LED not matched last frame drops; both were: the farther.
 */
static int match_nearest(const struct vis *v, const struct psvr2_blob_list *b, double gate, uint64_t prev,
			 int led_blob[PSVR2_LED_MAX], double dist[PSVR2_LED_MAX])
{
	int first[PSVR2_LED_MAX], second[PSVR2_LED_MAX], n = 0;
	double d1[PSVR2_LED_MAX], d2[PSVR2_LED_MAX];

	for (int i = 0; i < PSVR2_LED_MAX; i++)
		led_blob[i] = -1;
	for (int i = 0; i < v->n; i++) {
		int cand[2];
		const int nc = nearest(b, v->px[i][0], v->px[i][1], gate, 2, cand);
		first[i] = nc > 0 ? cand[0] : -1;
		second[i] = nc > 1 ? cand[1] : -1;
		d1[i] = d2[i] = 1e18;
		for (int k = 0; k < nc; k++) {
			const double dx = b->xy[cand[k]][0] - v->px[i][0], dy = b->xy[cand[k]][1] - v->px[i][1];
			(k ? d2 : d1)[i] = dx * dx + dy * dy;
		}
	}
	for (int i = 0; i < v->n; i++)
		for (int j = i + 1; j < v->n && first[i] >= 0; j++) {
			const int pi = (int)(prev >> v->led[i] & 1), pj = (int)(prev >> v->led[j] & 1);
			int loser, move;
			if (first[j] != first[i])
				continue;
			if (fabs(d1[i] - d1[j]) >= 400.0) {
				loser = d1[i] >= d1[j] ? i : j;
				move = 0;
			} else if (!pi && !pj) {
				loser = j;
				move = 0;
			} else if (second[i] >= 0 && second[j] >= 0) {
				loser = d2[i] > d2[j] ? j : i;
				move = 1;
			} else {
				loser = pi && pj ? (d1[i] >= d1[j] ? i : j) : (pi ? j : i);
				move = 0;
			}
			if (move) {
				first[loser] = second[loser];
				d1[loser] = d2[loser];
			} else {
				first[loser] = -1;
			}
		}
	for (int i = 0; i < v->n; i++)
		if (first[i] >= 0) {
			led_blob[v->led[i]] = first[i];
			dist[v->led[i]] = sqrt(d1[i]);
			n++;
		}
	return n;
}

/*
 * FUN_1800e4ec0 (flag 0x1803517c3, set in this build): a matched LED seen
 * >= 140 deg off its axis and <= 300 mm from the camera images as an
 * ellipse whose major axis is across the projected LED normal; it is
 * dropped when that axis is more than 25 deg from perpendicular. Needs the
 * blob angles (blobs[c].has_angle). Returns the matches left.
 */
static int grazing(const struct psvr2_rig_camera cams[], const struct psvr2_blob_list blobs[],
		   const struct psvr2_led_model *model, const struct psvr2_pose3 *pose, struct problem *p)
{
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		if (!blobs[c].has_angle)
			continue;
		for (int l = 0; l < model->count; l++) {
			const int k = p->led_blob[c][l];
			double px[2], q[2], va = 0, P[3], n[3], v[3], d, e[2], a, dep;
			if (k < 0 || k >= blobs[c].count || blobs[c].angle[k] < 0)
				continue;
			if (psvr2_led_project(&cams[c], model, pose, l, px, &va, NULL) || va < 140)
				continue;
			mv(pose->R, model->pos[l], P);
			for (int i = 0; i < 3; i++) {
				P[i] += pose->t[i];
				v[i] = P[i] - cams[c].t[i];
			}
			if (norm(v) > 300)
				continue;
			/* Projected normal: image step of a 1 mm move along it. */
			mv(pose->R, model->nrm[l], n);
			for (int i = 0; i < 3; i++)
				P[i] += n[i];
			if (project_point(&cams[c], P, q, &dep))
				continue;
			e[0] = q[0] - px[0];
			e[1] = q[1] - px[1];
			d = hypot(e[0], e[1]);
			if (!(d > 0))
				continue;
			a = acos(fmax(-1.0, fmin(1.0, (cos(blobs[c].angle[k]) * e[0] + sin(blobs[c].angle[k]) * e[1]) / d))) * DEG;
			if (fabs(a - 90.0) > 25.0) {
				p->led_blob[c][l] = -1;
				p->n--;
			}
		}
	}
	return p->n;
}

/*
 * States 1-3 (FUN_1800dbda0, FUN_1800dc210, FUN_1800dc680): one gated
 * match, gate = support radius = SEARCHRADIUS x scale (scale capped at
 * 10), LEDs visible to 174 deg (states 1, 3) or the model cone (state 2);
 * under 3 matches the nearest-blob fallback; the grazing check; then the
 * few-LED solve or the refine (FUN_1800dcf00 / FUN_1800dd2f0). No pose
 * check (Sony runs none in these states); our acceptance check applies.
 */
static int track_close(const struct psvr2_solver_cfg *cfg, const struct psvr2_rig_camera cams[],
		       const struct psvr2_blob_list blobs[], const struct psvr2_led_model *model,
		       const struct psvr2_pose3 *predicted, const struct psvr2_imu_prior *prior, int st,
		       struct psvr2_track_mem *mem, struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	struct problem p = { .cams = cams, .blobs = blobs, .model = model };
	double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];
	struct psvr2_pose3 pose = *predicted;
	const double gate = cfg->search_radius_px * scale_for(&pose);
	const double limit = st == PSVR2_TRACK_CLOSE ? model->cone_deg : 174.0;
	int n;

	if (mem)
		memcpy(p.prev, mem->prev, sizeof(p.prev));
	res->method = 0;
	n = match_all_ex(cams, blobs, model, &pose, gate, gate, cfg->sony_match ? 1 : 0, limit, &p, dist);
	if (n < 3)
		n = match_all_ex(cams, blobs, model, &pose, gate, gate, 2, model->cone_deg, &p, dist);
	if (cfg->grazing_check)
		n = grazing(cams, blobs, model, &pose, &p);
	if (n <= 0 || few_or_refine(cfg, cams, blobs, model, &p, predicted, prior, mem, &pose, res) ||
	    own_reject(cfg, &p, res))
		return -1;
	*out = pose;
	return 0;
}

/* FUN_1800db690's transitions; d in mm, imu = orientation prior set. */
static int next_state(int st, int valid, int imu, double d, int matched)
{
	if (!valid)
		return PSVR2_TRACK_LOST;
	switch (st) {
	case PSVR2_TRACK_NORMAL:
		if (!imu || d >= 130)
			return st;
		return d < 80 ? PSVR2_TRACK_CLOSE : PSVR2_TRACK_NEAR;
	case PSVR2_TRACK_NEAR:
		if (d > 170)
			return PSVR2_TRACK_NORMAL;
		return imu && d < 80 ? PSVR2_TRACK_CLOSE : st;
	case PSVR2_TRACK_CLOSE:
		return d <= 120 ? st : PSVR2_TRACK_NEAR;
	case PSVR2_TRACK_EDGE:
		if (imu && d < 130)
			return PSVR2_TRACK_NEAR;
		return matched > 5 ? PSVR2_TRACK_NORMAL : st;
	}
	return st;
}

static void set_prev(struct psvr2_track_mem *mem, const struct psvr2_solve_result *res, int nleds)
{
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		mem->prev[c] = 0;
		for (int l = 0; res && l < nleds && l < 64; l++)
			if (res->led_blob[c][l] >= 0)
				mem->prev[c] |= (uint64_t)1 << l;
	}
}

int psvr2_led_track_state(const struct psvr2_solver_cfg *cfg,
			  const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			  const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			  const struct psvr2_led_model *model,
			  const struct psvr2_pose3 *predicted,
			  const struct psvr2_imu_prior *prior, double dt_s,
			  struct psvr2_track_mem *mem,
			  struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	int st = mem->state, r;

	if (st < PSVR2_TRACK_NORMAL || st >= PSVR2_TRACK_LOST)
		st = PSVR2_TRACK_NORMAL;
	if (st == PSVR2_TRACK_EDGE && !cfg->edge_state)
		st = PSVR2_TRACK_NORMAL;
	if (st == PSVR2_TRACK_NORMAL && cfg->edge_state && inner_count(cams, model, predicted) <= 3)
		st = PSVR2_TRACK_EDGE;
	r = st == PSVR2_TRACK_NORMAL ? track_normal(cfg, cams, blobs, model, predicted, prior, dt_s, mem, out, res)
				     : track_close(cfg, cams, blobs, model, predicted, prior, st, mem, out, res);
	mem->state = next_state(st, r == 0, prior && prior->orient_valid,
				r == 0 ? cam_dist(cams, out, res, model->count) : cam_dist(cams, predicted, NULL, 0),
				r == 0 ? res->matched : 0);
	set_prev(mem, r == 0 ? res : NULL, model->count);
	return r;
}

int psvr2_led_boot_state(const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS], const struct psvr2_pose3 *pose)
{
	const double d = cam_dist(cams, pose, NULL, 0);

	return d < 100 ? PSVR2_TRACK_CLOSE : d < 150 ? PSVR2_TRACK_NEAR : PSVR2_TRACK_NORMAL;
}

void psvr2_led_track_mem_boot(struct psvr2_track_mem *mem, const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			      const struct psvr2_pose3 *pose, const struct psvr2_solve_result *res)
{
	memset(mem, 0, sizeof(*mem));
	mem->state = psvr2_led_boot_state(cams, pose);
	set_prev(mem, res, PSVR2_LED_MAX);
}

/* ---- orientation-prior boot (docs/sony-re/overview.md §A.9.3 step 6) -------------------- */

/* Pixel -> unit ray in the rig frame (inverse projection by Gauss-Newton
 * on the forward model; our choice, Sony's inverse was not found). */
/* Blob pixel -> normalised camera coordinates (x, y), z = 1. */
static int unproject_xy(const struct psvr2_cam_model *m, const double px[2], double xy[2])
{
	double x = -(px[0] - m->cx) / m->fx, y = -(px[1] - m->cy) / m->fy;

	for (int it = 0; it < 20; it++) {
		double P[3] = { x, y, 1 }, a[2], bx[2], by[2], J[4], det, ex, ey;
		const double h = 1e-6;
		if (psvr2_cam_project_rig(m, P, a))
			return -1;
		ex = a[0] - px[0];
		ey = a[1] - px[1];
		if (ex * ex + ey * ey < 1e-12)
			break;
		P[0] = x + h;
		psvr2_cam_project_rig(m, P, bx);
		P[0] = x;
		P[1] = y + h;
		psvr2_cam_project_rig(m, P, by);
		J[0] = (bx[0] - a[0]) / h; J[1] = (by[0] - a[0]) / h;
		J[2] = (bx[1] - a[1]) / h; J[3] = (by[1] - a[1]) / h;
		det = J[0] * J[3] - J[1] * J[2];
		if (fabs(det) < 1e-12)
			return -1;
		x -= (J[3] * ex - J[1] * ey) / det;
		y -= (-J[2] * ex + J[0] * ey) / det;
	}
	xy[0] = x;
	xy[1] = y;
	return 0;
}

static int unproject(const struct psvr2_rig_camera *cam, const double px[2], double ray[3])
{
	double d[3] = { 0, 0, 1 };

	if (unproject_xy(&cam->model, px, d))
		return -1;
	mv(cam->R, d, ray);	/* rig <- camera */
	const double n = norm(ray);
	for (int i = 0; i < 3; i++)
		ray[i] /= n;
	return 0;
}

/* t minimising the distance of R p_i + t to two rays from the camera. */
static int solve_translation(const double R[9], const double *p1, const double *p2,
			     const double c[3], const double d1[3], const double d2[3], double t[3])
{
	double A[36] = { 0 }, b[6] = { 0 }, x[6];
	const double *p[2] = { p1, p2 }, *d[2] = { d1, d2 };

	for (int k = 0; k < 2; k++) {
		double Rp[3], q[3], M[9];
		mv(R, p[k], Rp);
		for (int i = 0; i < 3; i++)
			q[i] = c[i] - Rp[i];
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				M[3 * i + j] = (i == j) - d[k][i] * d[k][j];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++)
				A[3 * i + j] += M[3 * i + j];
			b[i] += M[3 * i] * q[0] + M[3 * i + 1] * q[1] + M[3 * i + 2] * q[2];
		}
	}
	if (solve_spd(3, A, b, x))
		return -1;
	memcpy(t, x, sizeof(double) * 3);
	return 0;
}

int psvr2_led_boot_oriented(const struct psvr2_solver_cfg *cfg,
			    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			    const struct psvr2_led_model *model,
			    const double R_prior[9],
			    struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	struct psvr2_pose3 best = { .t = { 0 } };
	int best_m = 0;
	double best_e = DBL_MAX;
	const int npairs = sizeof(boot_pairs) / sizeof(boot_pairs[0]);

	memcpy(best.R, R_prior, sizeof(best.R));
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		const int nb = blobs[c].count < 10 ? blobs[c].count : 10;
		double rays[10][3], fwd[3] = { cams[c].R[2], cams[c].R[5], cams[c].R[8] };
		if (!cams[c].active || nb < 2)
			continue;
		for (int i = 0; i < nb; i++)
			if (unproject(&cams[c], blobs[c].xy[i], rays[i]))
				rays[i][0] = NAN;
		for (int pi = 0; pi < npairs * 2; pi++) {
			const int a = boot_led(model, boot_pairs[pi / 2][pi & 1]);
			const int b = boot_led(model, boot_pairs[pi / 2][!(pi & 1)]);
			double na[3], nbv[3];
			if (a >= model->count || b >= model->count)
				continue;
			/* Both LEDs must face the camera under the prior. */
			mv(R_prior, model->nrm[a], na);
			mv(R_prior, model->nrm[b], nbv);
			if (dot(na, fwd) >= 0 || dot(nbv, fwd) >= 0)
				continue;
			for (int i = 0; i < nb; i++)
				for (int j = 0; j < nb; j++) {
					struct psvr2_pose3 cand;
					struct problem p = { .cams = cams, .blobs = blobs, .model = model };
					double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX], e = 0;
					int m;
					if (i == j || isnan(rays[i][0]) || isnan(rays[j][0]))
						continue;
					memcpy(cand.R, R_prior, sizeof(cand.R));
					if (solve_translation(R_prior, model->pos[a], model->pos[b], cams[c].t,
							      rays[i], rays[j], cand.t))
						continue;
					if (norm(cand.t) > 2000)
						continue;
					m = match_all(cams, blobs, model, &cand, cfg->search_radius_px * scale_for(&cand),
						      cfg->search_radius_px * scale_for(&cand), cfg->sony_match,
						      &p, dist);
					for (int k = 0; k < PSVR2_SOLVER_CAMS; k++)
						for (int l = 0; l < model->count; l++)
							if (p.led_blob[k][l] >= 0)
								e += dist[k][l];
					if (m > best_m || (m == best_m && e < best_e)) {
						best = cand;
						best_m = m;
						best_e = e;
					}
				}
		}
	}
	if (best_m < cfg->min_inliers_orientation)
		return -1;
	if (psvr2_led_track(cfg, cams, blobs, model, &best, out, res))
		return -1;
	if (res->matched >= cfg->min_inliers_boot)
		return 0;
	/*
	 * Sony's rule below MININLIERSFORBOOT (docs/sony-re/overview.md §A.9.3): keep the pose
	 * only when it stays within 15 deg of the IMU prior (30 deg closer
	 * than 150 mm) and is nearer than 500 mm. The gravity-only variant
	 * needs measured gravity, which this entry point does not take.
	 */
	{
		const double d = norm(out->t);
		double tr = 0;
		for (int i = 0; i < 9; i++)
			tr += out->R[i] * R_prior[i];
		if (res->matched < cfg->min_inliers_orientation || d >= 500 ||
		    acos(fmax(-1.0, fmin(1.0, (tr - 1) / 2))) * DEG > (d < 150 ? 30 : 15))
			return -1;
	}
	return 0;
}

/* ---- Sony's RANSAC boot (FUN_1800d8b60 -> FUN_1800c2960) ----------------- */

/* MT19937 (RnD::MersenneTwister, FUN_18009f3e0 seed / FUN_18009f530 draw). */
struct mt19937 {
	uint32_t s[624];
	int i;
};

static void mt_seed(struct mt19937 *m, uint32_t seed)
{
	m->s[0] = seed;
	for (int i = 1; i < 624; i++)
		m->s[i] = 1812433253u * (m->s[i - 1] ^ (m->s[i - 1] >> 30)) + (uint32_t)i;
	m->i = 624;
}

static uint32_t mt_next(struct mt19937 *m)
{
	uint32_t y;

	if (m->i >= 624) {
		for (int k = 0; k < 624; k++) {
			y = (m->s[k] & 0x80000000u) | (m->s[(k + 1) % 624] & 0x7fffffffu);
			m->s[k] = m->s[(k + 397) % 624] ^ (y >> 1) ^ (y & 1 ? 0x9908b0dfu : 0);
		}
		m->i = 0;
	}
	y = m->s[m->i++];
	y ^= y >> 11;
	y ^= (y << 7) & 0x9d2c5680u;
	y ^= (y << 15) & 0xefc60000u;
	return y ^ (y >> 18);
}

/* Sony's index draw: (int)(r * 2^-32 * n) (0x18031a2e0). */
static int mt_index(struct mt19937 *m, int n)
{
	return (int)(mt_next(m) * (1.0 / 4294967296.0) * n);
}

/* Real roots of a4 x^4 + ... + a0 (Durand-Kerner, then Newton polish). */
struct cx { double re, im; };
static struct cx cx_mul(struct cx a, struct cx b)
{
	return (struct cx){ a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re };
}

static int quartic_roots(const double a[5], double x[4])
{
	struct cx z[4] = { { 1, 0 }, { 0.4, 0.9 } };
	double p[5];
	int n = 0;

	if (fabs(a[4]) < 1e-12)
		return 0;
	for (int i = 0; i < 5; i++)
		p[i] = a[i] / a[4];
	z[2] = cx_mul(z[1], z[1]);
	z[3] = cx_mul(z[2], z[1]);
	for (int it = 0; it < 100; it++) {
		double move = 0;
		for (int i = 0; i < 4; i++) {
			struct cx num = { 1, 0 }, den = { 1, 0 }, q;
			double dd;
			for (int k = 3; k >= 0; k--) {	/* Horner, monic */
				num = cx_mul(num, z[i]);
				num.re += p[k];
			}
			for (int j = 0; j < 4; j++)
				if (j != i)
					den = cx_mul(den, (struct cx){ z[i].re - z[j].re, z[i].im - z[j].im });
			dd = den.re * den.re + den.im * den.im;
			if (dd < 1e-300)
				continue;
			q.re = (num.re * den.re + num.im * den.im) / dd;
			q.im = (num.im * den.re - num.re * den.im) / dd;
			z[i].re -= q.re;
			z[i].im -= q.im;
			move = fmax(move, hypot(q.re, q.im) / fmax(1.0, hypot(z[i].re, z[i].im)));
		}
		if (move < 1e-10)
			break;
	}
	for (int i = 0; i < 4; i++) {
		double r = z[i].re;
		if (fabs(z[i].im) > 1e-6 * fmax(1.0, hypot(z[i].re, z[i].im)))
			continue;
		for (int it = 0; it < 5; it++) {
			const double f = (((r + p[3]) * r + p[2]) * r + p[1]) * r + p[0];
			const double df = ((4 * r + 3 * p[3]) * r + 2 * p[2]) * r + p[1];
			if (df == 0)
				break;
			r -= f / df;
		}
		x[n++] = r;
	}
	return n;
}

/* Rigid map of three points P onto three points X (exact for a minimal set). */
static int pose_from_3(const double P[3][3], const double X[3][3], struct psvr2_pose3 *out)
{
	double Fp[9], Fx[9];
	const double (*S[2])[3] = { P, X };
	double *F[2] = { Fp, Fx };

	for (int k = 0; k < 2; k++) {
		double e1[3], e2[3], e3[3], w[3], n;
		for (int i = 0; i < 3; i++) {
			e1[i] = S[k][1][i] - S[k][0][i];
			w[i] = S[k][2][i] - S[k][0][i];
		}
		n = norm(e1);
		if (n < 1e-9)
			return -1;
		for (int i = 0; i < 3; i++)
			e1[i] /= n;
		e3[0] = e1[1] * w[2] - e1[2] * w[1];
		e3[1] = e1[2] * w[0] - e1[0] * w[2];
		e3[2] = e1[0] * w[1] - e1[1] * w[0];
		n = norm(e3);
		if (n < 1e-9)
			return -1;
		for (int i = 0; i < 3; i++)
			e3[i] /= n;
		e2[0] = e3[1] * e1[2] - e3[2] * e1[1];
		e2[1] = e3[2] * e1[0] - e3[0] * e1[2];
		e2[2] = e3[0] * e1[1] - e3[1] * e1[0];
		for (int i = 0; i < 3; i++) {	/* columns e1 e2 e3 */
			F[k][3 * i] = e1[i];
			F[k][3 * i + 1] = e2[i];
			F[k][3 * i + 2] = e3[i];
		}
	}
	for (int i = 0; i < 3; i++)	/* R = Fx Fp^T */
		for (int j = 0; j < 3; j++)
			out->R[3 * i + j] = Fx[3 * i] * Fp[3 * j] + Fx[3 * i + 1] * Fp[3 * j + 1] +
					    Fx[3 * i + 2] * Fp[3 * j + 2];
	{
		double Rp[3];
		mv(out->R, P[0], Rp);
		for (int i = 0; i < 3; i++)
			out->t[i] = X[0][i] - Rp[i];
	}
	return 0;
}

/*
 * P3P (Sony FUN_1800f43e0, up to 4 poses). Ours: Grunert's quartic in the
 * form of Haralick et al. 1994; Sony's closed form was not ported, the
 * poses are the same up to rounding. c: camera centre, f: unit rays (rig),
 * P: model points. Returns the number of poses.
 */
static int p3p(const double c[3], const double f[3][3], const double P[3][3], struct psvr2_pose3 out[4])
{
	double d[3], ca, cb, cg, a2, b2, c2, A[5], v[4], K, K2;
	int nr, n = 0;

	for (int i = 0; i < 3; i++) {
		const int j = (i + 1) % 3, k = (i + 2) % 3;
		double e[3] = { P[j][0] - P[k][0], P[j][1] - P[k][1], P[j][2] - P[k][2] };
		d[i] = dot(e, e);	/* d[0] = a^2 = |P2-P3|^2, d[1] = b^2, d[2] = c^2 */
	}
	a2 = d[0];
	b2 = d[1];
	c2 = d[2];
	ca = dot(f[1], f[2]);
	cb = dot(f[0], f[2]);
	cg = dot(f[0], f[1]);
	if (b2 < 1e-9)
		return 0;
	K = (a2 - c2) / b2;
	K2 = K * K;
	A[4] = (K - 1) * (K - 1) - 4 * c2 / b2 * ca * ca;
	A[3] = 4 * (K * (1 - K) * cb - (1 - (a2 + c2) / b2) * ca * cg + 2 * c2 / b2 * ca * ca * cb);
	A[2] = 2 * (K2 - 1 + 2 * K2 * cb * cb + 2 * (b2 - c2) / b2 * ca * ca -
		    4 * (a2 + c2) / b2 * ca * cb * cg + 2 * (b2 - a2) / b2 * cg * cg);
	A[1] = 4 * (-K * (1 + K) * cb + 2 * a2 / b2 * cg * cg * cb - (1 - (a2 + c2) / b2) * ca * cg);
	A[0] = (1 + K) * (1 + K) - 4 * a2 / b2 * cg * cg;
	nr = quartic_roots(A, v);
	for (int r = 0; r < nr && n < 4; r++) {
		const double den = 2 * (cg - v[r] * ca);
		double u, s1, s[3], X[3][3], q;
		if (v[r] <= 0 || fabs(den) < 1e-12)
			continue;
		u = ((K - 1) * v[r] * v[r] - 2 * K * cb * v[r] + 1 + K) / den;
		q = 1 + v[r] * v[r] - 2 * v[r] * cb;
		if (u <= 0 || q <= 0)
			continue;
		s1 = sqrt(b2 / q);
		s[0] = s1;
		s[1] = u * s1;
		s[2] = v[r] * s1;
		/* Reject spurious roots: the side c must hold too. */
		if (fabs(s[0] * s[0] + s[1] * s[1] - 2 * s[0] * s[1] * cg - c2) > 1e-3 * fmax(c2, 1.0) ||
		    fabs(s[1] * s[1] + s[2] * s[2] - 2 * s[1] * s[2] * ca - a2) > 1e-3 * fmax(a2, 1.0))
			continue;
		for (int i = 0; i < 3; i++)
			for (int k = 0; k < 3; k++)
				X[i][k] = c[k] + s[i] * f[i][k];
		if (pose_from_3(P, X, &out[n]) == 0)
			n++;
	}
	return n;
}

/*
 * MSAC score in one camera (FUN_1800c2080): every LED projected (in front,
 * inside the image; no facing test here), nearest blob within thr; a blob
 * takes at most 2 LEDs, in LED order. cost = sum(min(d^2, thr^2)) / LEDs.
 */
static int ransac_score(const struct psvr2_rig_camera *cam, const struct psvr2_blob_list *b,
			const struct psvr2_led_model *model, const struct psvr2_pose3 *pose, double thr,
			double *cost)
{
	unsigned char used[PSVR2_SOLVER_BLOBS] = { 0 };
	const double t2 = thr * thr;
	double sum = 0;
	int inl = 0;

	for (int l = 0; l < model->count; l++) {
		double P[3], px[2], best = t2, dep;
		int bi = -1;
		mv(pose->R, model->pos[l], P);
		for (int i = 0; i < 3; i++)
			P[i] += pose->t[i];
		if (project_point(cam, P, px, &dep) == 0 && dep > 0)
			for (int k = 0; k < b->count; k++) {
				const double dx = b->xy[k][0] - px[0], dy = b->xy[k][1] - px[1];
				const double e = dx * dx + dy * dy;
				if (e < best) {
					best = e;
					bi = k;
				}
			}
		if (bi >= 0 && used[bi] < 2) {
			used[bi]++;
			inl++;
			sum += best;
		} else {
			sum += t2;
		}
	}
	*cost = sum / model->count;
	return inl;
}

/* k nearest of n points (dist2 callback), self excluded, ascending. */
static int nearest_k(int self, int n, const unsigned char *ok, double (*d2)(const void *, int, int),
		     const void *ctx, int k, int out[4])
{
	double dk[4];
	int m = 0;

	for (int j = 0; j < n; j++) {
		double e;
		int p;
		if (j == self || (ok && !ok[j]))
			continue;
		e = d2(ctx, self, j);
		if (m < k)
			m++;
		else if (e >= dk[m - 1])
			continue;
		for (p = m - 1; p > 0 && dk[p - 1] > e; p--) {
			dk[p] = dk[p - 1];
			out[p] = out[p - 1];
		}
		dk[p] = e;
		out[p] = j;
	}
	return m;
}

static double d2_led(const void *ctx, int i, int j)
{
	const struct psvr2_led_model *m = ctx;
	double e[3] = { m->pos[i][0] - m->pos[j][0], m->pos[i][1] - m->pos[j][1], m->pos[i][2] - m->pos[j][2] };

	return dot(e, e);
}

static double d2_blob(const void *ctx, int i, int j)
{
	const struct psvr2_blob_list *b = ctx;

	return (b->xy[i][0] - b->xy[j][0]) * (b->xy[i][0] - b->xy[j][0]) +
	       (b->xy[i][1] - b->xy[j][1]) * (b->xy[i][1] - b->xy[j][1]);
}

/*
 * Sample winding (FUN_1800c58f0): the blob triangle's turn and the LED
 * triangle's (cross product against the summed normals) must agree, else
 * LEDs 1 and 2 swap. The sign relation is for our camera frame (z forward,
 * pixels a 180 deg turn of x/z, y/z), derived, not copied.
 */
static void fix_winding(const struct psvr2_led_model *m, const struct psvr2_blob_list *b, int s[6])
{
	const double *p0 = m->pos[s[0]], *p1 = m->pos[s[1]], *p2 = m->pos[s[2]];
	double e1[3], e2[3], cr[3], ns[3];
	const double d1x = b->xy[s[4]][0] - b->xy[s[3]][0], d1y = b->xy[s[4]][1] - b->xy[s[3]][1];
	const double d2x = b->xy[s[5]][0] - b->xy[s[4]][0], d2y = b->xy[s[5]][1] - b->xy[s[4]][1];
	int img, mod;

	for (int i = 0; i < 3; i++) {
		e1[i] = p1[i] - p0[i];
		e2[i] = p2[i] - p1[i];
		ns[i] = m->nrm[s[0]][i] + m->nrm[s[1]][i] + m->nrm[s[2]][i];
	}
	cr[0] = e1[1] * e2[2] - e1[2] * e2[1];
	cr[1] = e1[2] * e2[0] - e1[0] * e2[2];
	cr[2] = e1[0] * e2[1] - e1[1] * e2[0];
	img = d1y * d2x < d1x * d2y;
	mod = dot(cr, ns) < 0;
	if (img != mod) {
		const int t = s[1];
		s[1] = s[2];
		s[2] = t;
	}
}

/*
 * Mode 7 direction test (FUN_1800c5ac0): the image vector from blob 0 to
 * the middle of blobs 1, 2 (mid = 1, second sampler stage) or to blob 2
 * (mid = 0, ring stage) must be within 90 deg of the same LED vector under
 * the IMU rotation, projected at the blobs' mean ray. Sony builds it from
 * gravity in the image (+0x1000); ours uses the full prior rotation, which
 * it has in this mode too.
 */
static int prior_direction_ok(const struct psvr2_rig_camera *cam, const struct psvr2_led_model *m,
			      const struct psvr2_blob_list *b, const double Rp[9], const int s[6], int mid)
{
	double w[3], wr[3], wc[3], xy[2], u = 0, v = 0, dx, dy, ex, ey;

	for (int k = 3; k < 6; k++) {
		if (unproject_xy(&cam->model, b->xy[s[k]], xy))
			return 1;
		u += xy[0] / 3;
		v += xy[1] / 3;
	}
	for (int i = 0; i < 3; i++)
		w[i] = (mid ? (m->pos[s[1]][i] + m->pos[s[2]][i]) * 0.5 : m->pos[s[2]][i]) - m->pos[s[0]][i];
	mv(Rp, w, wr);
	mtv(cam->R, wr, wc);	/* camera <- rig */
	/* d(-x/z), d(-y/z) at (u, v), up to the depth: pixel direction. */
	dx = -(wc[0] - u * wc[2]);
	dy = -(wc[1] - v * wc[2]);
	ex = (mid ? (b->xy[s[4]][0] + b->xy[s[5]][0]) * 0.5 : b->xy[s[5]][0]) - b->xy[s[3]][0];
	ey = (mid ? (b->xy[s[4]][1] + b->xy[s[5]][1]) * 0.5 : b->xy[s[5]][1]) - b->xy[s[3]][1];
	return dx * ex + dy * ey >= 0;
}

/*
 * Blob chain for the ring stage (FUN_1800c52f0): order three blobs as
 * (end, middle, end) through the vertex where the path turns by < 90 deg,
 * ends chosen so the turn is d_in.y d_out.x < d_in.x d_out.y; a fourth
 * neighbour of the last blob must continue the chain (same turn, forward).
 */
static int blob_chain(const struct psvr2_blob_list *b, const int bn[][4], const int nbn[], const int t[3],
		      int out[3])
{
	double e[4][2], in[2], dout[2], c[2];
	int k, prev, next, first, last, mid;

	for (int i = 0; i < 3; i++) {	/* e[i + 1] = p[i + 1] - p[i] */
		const int j = (i + 1) % 3;
		e[i + 1][0] = b->xy[t[j]][0] - b->xy[t[i]][0];
		e[i + 1][1] = b->xy[t[j]][1] - b->xy[t[i]][1];
	}
	for (k = 0; k < 3; k++) {	/* vertex k: in = e[k] (e[3] for 0), out = e[k + 1] */
		const double *pi = e[k ? k : 3], *po = e[k + 1];
		if (po[0] * pi[0] + po[1] * pi[1] >= 0)
			break;
	}
	if (k == 3)
		return 0;
	in[0] = e[k ? k : 3][0];
	in[1] = e[k ? k : 3][1];
	dout[0] = e[k + 1][0];
	dout[1] = e[k + 1][1];
	prev = k ? k - 1 : 2;
	next = (k + 1) % 3;
	mid = t[k];
	if (in[1] * dout[0] < in[0] * dout[1]) {
		first = t[prev];
		last = t[next];
	} else {
		first = t[next];
		last = t[prev];
		dout[0] = -in[0];
		dout[1] = -in[1];
	}
	for (int j = 0;; j++) {
		int q;
		if (j >= nbn[last])
			return 0;
		q = bn[last][j];
		if (q == mid || q == first)
			continue;
		c[0] = b->xy[q][0] - b->xy[last][0];
		c[1] = b->xy[q][1] - b->xy[last][1];
		if (dout[1] * c[0] < dout[0] * c[1])
			break;
	}
	if (c[0] * dout[0] + c[1] * dout[1] < 0)
		return 0;
	out[0] = first;
	out[1] = mid;
	out[2] = last;
	return 1;
}

/* FUN_1800c4940 state: the ring stage's current blob chain and window. */
struct ring_stage {
	int chain[3], win, accepted, cap, total, ntried;
	int tried[256];
};

/*
 * Ring stage (FUN_1800c4940 first part): a new blob chain from a blob and
 * two of its neighbours (up to min(4 n, 20) draws, chains already tried
 * skipped), then the next 3-LED window of the rings (window i of a ring of
 * n: i <= n - 4; after the last, the next ring) that passes the mode 7
 * test. Each accepted window is one sample, at most cap of them. 0 = none
 * left now (the random stage runs).
 */
static int sample_ring(struct ring_stage *rs, struct mt19937 *mt, const struct psvr2_rig_camera *cam,
		       const struct psvr2_led_model *m, const struct psvr2_blob_list *b, int nb,
		       const unsigned char *ok, const int bn[][4], const int nbn[], int kb, const double Rp[9],
		       int s[6])
{
	if (rs->accepted >= rs->cap || !rs->total || kb < 2)
		return 0;
	if (rs->win < 0) {
		const int tries = 4 * nb < 20 ? 4 * nb : 20;
		for (int n = 0; n < tries && rs->win < 0; n++) {
			const int b0 = mt_index(mt, nb), r1 = mt_index(mt, kb), r2 = mt_index(mt, kb);
			int t[3], ch[3], key, dup = 0;
			if (r1 == r2 || !ok[b0] || r1 >= nbn[b0] || r2 >= nbn[b0])
				continue;
			t[0] = b0;
			t[1] = bn[b0][r1];
			t[2] = bn[b0][r2];
			if (!blob_chain(b, bn, nbn, t, ch))
				continue;
			key = (ch[0] << 16) | (ch[1] << 8) | ch[2];
			for (int i = 0; i < rs->ntried && !dup; i++)
				dup = rs->tried[i] == key;
			if (dup)
				continue;
			if (rs->ntried < 256)
				rs->tried[rs->ntried++] = key;
			memcpy(rs->chain, ch, sizeof(ch));
			rs->win = 0;
		}
	}
	while (rs->win >= 0 && rs->win < rs->total) {
		int r = 0, i = rs->win, pass;
		while (r < m->rings && m->ring_len[r] <= i + 3) {
			i -= m->ring_len[r];
			r++;
		}
		if (r >= m->rings)
			break;
		s[0] = m->ring[r][i];
		s[1] = m->ring[r][i + 1];
		s[2] = m->ring[r][i + 2];
		memcpy(s + 3, rs->chain, sizeof(rs->chain));
		rs->win += m->ring_len[r] <= i + 4 ? 4 : 1;
		pass = !Rp || prior_direction_ok(cam, m, b, Rp, s, 0);
		if (pass) {
			rs->accepted++;
			return 1;
		}
	}
	rs->win = -1;
	return 0;
}

/* Best RANSAC pose in camera c; returns inliers (0 = none). */
static int ransac_cam(const struct psvr2_solver_cfg *cfg, const struct psvr2_rig_camera cams[],
		      int ci, const struct psvr2_blob_list *b, const struct psvr2_led_model *m,
		      const struct psvr2_imu_prior *prior, int samples, struct psvr2_pose3 *best,
		      double *best_cost)
{
	const struct psvr2_rig_camera *cam = &cams[ci];
	const double *Rp = prior && prior->orient_valid ? prior->R : NULL;
	struct mt19937 mt;
	double ray[PSVR2_SOLVER_BLOBS][3];
	unsigned char ok[PSVR2_SOLVER_BLOBS] = { 0 };
	int bn[PSVR2_SOLVER_BLOBS][4], nbn[PSVR2_SOLVER_BLOBS], ln[PSVR2_LED_MAX][4], nln[PSVR2_LED_MAX];
	int cand[PSVR2_LED_MAX], ncand = 0, nok = 0, best_inl = 0, nb = b->count, kb;
	struct ring_stage rs;
	const double fwd[3] = { cam->R[2], cam->R[5], cam->R[8] };
	const double cos95 = cos(1.6580629);

	if (nb > PSVR2_SOLVER_BLOBS)
		nb = PSVR2_SOLVER_BLOBS;
	*best_cost = DBL_MAX;
	/* Blobs that undistort (FUN_1800c3c90 valid bits). */
	for (int i = 0; i < nb; i++) {
		ok[i] = unproject(cam, b->xy[i], ray[i]) == 0;
		nok += ok[i];
	}
	if (nok < 3 || m->count < 3)
		return 0;
	/* 4 nearest valid blobs in the image (FUN_1800c62c0, modes 6/7). */
	for (int i = 0; i < nb; i++)
		nbn[i] = ok[i] ? nearest_k(i, nb, ok, d2_blob, b, 4, bn[i]) : 0;
	/* 4 nearest LEDs in the model, all LEDs (FUN_1800c6df0). */
	for (int l = 0; l < m->count; l++)
		nln[l] = nearest_k(l, m->count, NULL, d2_led, m, 4, ln[l]);
	/* Sampled LEDs (FUN_1800c3320): all of them in modes 6 and 7 (its
	 * 95 deg facing cut applies to mode 4 only). */
	for (int l = 0; l < m->count; l++)
		cand[ncand++] = l;
	rs.win = -1;
	rs.accepted = rs.ntried = rs.total = 0;
	/* +0xf7c = (int)(RANSACMAXSAMPLES * RANSACRINGBOOTRATIO), set before
	 * the boot-in-track cap replaces +0x00 (FUN_1800c18a0). */
	rs.cap = (int)(cfg->ransac_samples * cfg->ransac_ring_ratio);
	for (int r = 0; r < m->rings; r++)
		rs.total += m->ring_len[r];
	kb = nok - 1 < 4 ? nok - 1 : 4;	/* +0xed8 */
	mt_seed(&mt, 0);	/* FUN_1800d8b60 reseeds with +0x60c = 0 */
	for (int it = 0; it < samples; it++) {
		int s[6], tries = 0, found;
		struct psvr2_pose3 sol[4];
		double f[3][3], P[3][3], mean = 0;
		int nsol;
		found = sample_ring(&rs, &mt, cam, m, b, nb, ok, bn, nbn, kb, Rp, s);
		/* Random stage (FUN_1800c4940 second part): LED + 2 of its
		 * neighbours, blob + 2 of its neighbours, all distinct
		 * (FUN_1800c61d0), then winding and the prior test. */
		while (!found && tries++ < 64) {
			const int l0 = cand[mt_index(&mt, ncand)], b0 = mt_index(&mt, nb);
			int i1, i2, j1, j2;
			if (!ok[b0] || nln[l0] < 2 || nbn[b0] < 2)
				continue;
			i1 = mt_index(&mt, nln[l0]);
			i2 = mt_index(&mt, nln[l0]);
			j1 = mt_index(&mt, nbn[b0]);
			j2 = mt_index(&mt, nbn[b0]);
			if (i1 == i2 || j1 == j2)
				continue;
			s[0] = l0; s[1] = ln[l0][i1]; s[2] = ln[l0][i2];
			s[3] = b0; s[4] = bn[b0][j1]; s[5] = bn[b0][j2];
			fix_winding(m, b, s);
			if (Rp && !prior_direction_ok(cam, m, b, Rp, s, 1))
				continue;
			found = 1;
		}
		if (!found)
			continue;
		for (int k = 0; k < 3; k++) {
			memcpy(f[k], ray[s[3 + k]], sizeof(f[k]));
			memcpy(P[k], m->pos[s[k]], sizeof(P[k]));
			mean += (b->has_angle && b->major[s[3 + k]] > 0 ? b->major[s[3 + k]] : 0) / 3;
		}
		nsol = p3p(cam->t, f, P, sol);
		for (int q = 0; q < nsol; q++) {
			double cost, thr = cfg->inlier_px, oc[3];
			int good = 1, inl;
			/* The 3 sampled LEDs face the camera (n . view < 0.08) and
			 * lie within 95 deg of the optical axis. */
			for (int k = 0; k < 3 && good; k++) {
				double X[3], nr[3], d[3], len;
				mv(sol[q].R, P[k], X);
				mv(sol[q].R, m->nrm[s[k]], nr);
				for (int i = 0; i < 3; i++)
					d[i] = X[i] + sol[q].t[i] - cam->t[i];
				len = norm(d);
				for (int i = 0; i < 3; i++)
					d[i] /= len;
				good = dot(nr, d) < 0.08 && dot(d, fwd) >= cos95;
			}
			/* Hypothesis constraints (RnD::FullBootConstraints at
			 * boot obj +0x4ca0, slot 1: FUN_1800d89f0 with close = 1). */
			if (!good || psvr2_led_pose_check(cfg, cams, m, prior, &sol[q], NULL, 1))
				continue;
			/* 6 px when nearer than 200 mm with blobs > 3 px
			 * (+0x28, +0x2c, +0x24). */
			for (int i = 0; i < 3; i++)
				oc[i] = sol[q].t[i] - cam->t[i];
			if (norm(oc) < 200 && mean > 3)
				thr = 6;
			inl = ransac_score(cam, b, m, &sol[q], thr, &cost);
			if (inl > best_inl || (inl == best_inl && inl > 0 && cost < *best_cost)) {
				best_inl = inl;
				*best_cost = cost;
				*best = sol[q];
			}
		}
	}
	return best_inl;
}

/*
 * Boot acceptance (FUN_1800d8b60, section 2.9). At or above
 * MININLIERSFORBOOT the pose stands; below it, it needs gravity within
 * 3 deg (6 deg nearer than 150 mm, FUN_1800e1f50) with >= 5 inliers, or
 * the orientation prior within 15 deg (30 deg) nearer than 500 mm
 * (FUN_1800e20e0) with >= 4.
 */
static int boot_accept(const struct psvr2_solver_cfg *cfg, const struct psvr2_imu_prior *prior,
		       const struct psvr2_pose3 *pose, int matched)
{
	const double d = norm(pose->t);
	int ok_g = 0, ok_o = 0;

	if (matched >= cfg->min_inliers_boot)
		return 1;
	if (prior && prior->gravity_valid && prior->gravity_age_s < 10) {
		double g[3];
		mv(pose->R, prior->g_model, g);
		ok_g = acos(fmax(-1.0, fmin(1.0, dot(g, prior->g_rig) / (norm(g) * norm(prior->g_rig))))) * DEG <=
		       (d < 150 ? 6 : 3);
	}
	if (prior && prior->orient_valid && prior->orient_age_s < 10 && prior->gravity_age_s < 10 && d < 500)
		ok_o = rot_angle_deg(pose->R, prior->R) <= (d < 150 ? 30 : 15);
	return (ok_g && matched >= 5) || (ok_o && matched >= cfg->min_inliers_orientation);
}

int psvr2_led_boot_ransac(const struct psvr2_solver_cfg *cfg,
			  const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
			  const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
			  const struct psvr2_led_model *model,
			  const struct psvr2_imu_prior *prior, int in_track,
			  struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	const int samples = in_track ? cfg->ransac_samples_in_track : cfg->ransac_samples;
	int found = 0;

	/*
	 * Sony boots each camera on its own (FUN_1800d5070 -> FUN_1800d8b60:
	 * RANSAC, refine, accept) and keeps the candidate with the most
	 * inliers. Ours refines in all cameras and breaks ties by rms.
	 */
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		struct psvr2_pose3 p, o;
		struct psvr2_solve_result r;
		double cost;
		/* FUN_1800d8b60: fewer blobs than MININLIERS-FOR-ORIENTATIONBOOT
		 * (+0x620) skips the camera. */
		if (!cams[c].active || blobs[c].count < cfg->min_inliers_orientation)
			continue;
		if (ransac_cam(cfg, cams, c, &blobs[c], model, prior, samples, &p, &cost) < 4)
			continue;	/* +0xa94 >= 4 */
		if (psvr2_led_track(cfg, cams, blobs, model, &p, &o, &r) || !boot_accept(cfg, prior, &o, r.matched))
			continue;
		/* Ours (own_accept): a RANSAC boot needs cross-camera support
		 * >= 0.75. In the synthetic test wrong boots (a ring turned by
		 * one LED, mostly) score 0.23-0.78, right ones 0.69-0.95. */
		if (cfg->own_accept && psvr2_led_support(cfg, cams, blobs, model, &o, &r) < 0.75)
			continue;
		if (!found || r.matched > res->matched || (r.matched == res->matched && r.rms_px < res->rms_px)) {
			*out = o;
			*res = r;
			found = 1;
		}
	}
	return found ? 0 : -1;
}
