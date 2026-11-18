// SPDX-License-Identifier: GPL-2.0
/*
 * Sense controller pose from headset LED blobs. Behaviour from
 * docs/RESEARCH.md §9; the numerics (numeric Jacobian, Nielsen damping
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

void psvr2_solver_default_cfg(struct psvr2_solver_cfg *cfg)
{
	cfg->search_radius_px = 3;
	cfg->pothole = 1;
	cfg->min_inliers_boot = 7;
	cfg->min_inliers_orientation = 4;
	cfg->max_iterations = 250;
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

/* ---- association and matching (RESEARCH.md §9.3 steps 2-3) --------------- */

struct vis {
	int n;
	int led[PSVR2_LED_MAX];
	double px[PSVR2_LED_MAX][2], angle[PSVR2_LED_MAX], depth[PSVR2_LED_MAX];
};

static void associate(const struct psvr2_rig_camera *cam, const struct psvr2_led_model *model,
		      const struct psvr2_pose3 *pose, struct vis *v)
{
	struct vis all = { 0 };
	double origin_depth, opx[2];
	int i, j;

	v->n = 0;
	for (i = 0; i < model->count; i++) {
		const int k = all.n;
		if (psvr2_led_project(cam, model, pose, i, all.px[k], &all.angle[k], &all.depth[k]) == 0)
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

/* ---- refine: weighted LM with Huber IRLS (RESEARCH.md §9.3 step 7) -------- */

struct problem {
	const struct psvr2_rig_camera *cams;
	const struct psvr2_blob_list *blobs;
	const struct psvr2_led_model *model;
	int led_blob[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];
	int n;			/* matched pairs */
	int expected;		/* associated LEDs clearly facing a camera (< 70 deg) */
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

static int match_all(const struct psvr2_rig_camera cams[], const struct psvr2_blob_list blobs[],
		     const struct psvr2_led_model *model, const struct psvr2_pose3 *pose, double gate,
		     struct problem *p, double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX])
{
	p->n = p->expected = 0;
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		struct vis v;
		double opx[2], od = 0;
		for (int l = 0; l < PSVR2_LED_MAX; l++)
			p->led_blob[c][l] = -1;
		if (!cams[c].active || !blobs[c].count)
			continue;
		associate(&cams[c], model, pose, &v);
		for (int i = 0; i < v.n; i++)
			p->expected += v.angle[i] < 70;
		project_point(&cams[c], pose->t, opx, &od);
		p->n += match(&v, &blobs[c], gate, od, p->led_blob[c], dist[c]);
	}
	return p->n;
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

int psvr2_led_track(const struct psvr2_solver_cfg *cfg,
		    const struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS],
		    const struct psvr2_blob_list blobs[PSVR2_SOLVER_CAMS],
		    const struct psvr2_led_model *model,
		    const struct psvr2_pose3 *predicted,
		    struct psvr2_pose3 *out, struct psvr2_solve_result *res)
{
	struct problem p = { .cams = cams, .blobs = blobs, .model = model };
	double dist[PSVR2_SOLVER_CAMS][PSVR2_LED_MAX];
	struct psvr2_pose3 pose = *predicted;
	double sc = scale_for(&pose);

	/* Two rounds: match with the search gate, refine, re-match tighter. */
	for (int round = 0; round < 2; round++) {
		const double gate = round ? fmax(cfg->search_radius_px, 1.5 * cfg->pothole * sc)
					  : cfg->search_radius_px * sc;
		if (match_all(cams, blobs, model, &pose, gate, &p, dist) < 3)
			return -1;
		if (refine(&p, cfg->max_iterations, &pose))
			return -1;
		sc = scale_for(&pose);
	}
	finish(&p, &pose, cfg->pothole * sc, res);
	res->expected = p.expected;
	/*
	 * Our acceptance check (Sony's per-frame rule was not traced): a pose
	 * that explains under 3/4 of the LEDs facing the cameras, or leaves more
	 * than 1.5 px rms, is a wrong local minimum. Fail so the caller boots.
	 */
	if (p.n < cfg->min_inliers_orientation || 4 * p.n < 3 * p.expected || res->rms_px > 1.5)
		return -1;
	*out = pose;
	return 0;
}

/* ---- orientation-prior boot (RESEARCH.md §9.3 step 6) -------------------- */

/* Pixel -> unit ray in the rig frame (inverse projection by Gauss-Newton
 * on the forward model; our choice, Sony's inverse was not found). */
static int unproject(const struct psvr2_rig_camera *cam, const double px[2], double ray[3])
{
	const struct psvr2_cam_model *m = &cam->model;
	double x = -(px[0] - m->cx) / m->fx, y = -(px[1] - m->cy) / m->fy, d[3];

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
	d[0] = x; d[1] = y; d[2] = 1;
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
			const int a = boot_pairs[pi / 2][pi & 1], b = boot_pairs[pi / 2][!(pi & 1)];
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
	 * Sony's rule below MININLIERSFORBOOT (RESEARCH.md §9.3): keep the pose
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
