// SPDX-License-Identifier: GPL-2.0
/*
 * Synthetic check of the LED pose solver with Sony's LED models: four
 * cameras near the built-in rig positions, random poses 0.4-1 m out, 0.3 px
 * blob noise and two clutter blobs per camera. Blobs exist only for LEDs
 * facing a camera (< 100 deg; LED_VA), as the controller body hides the
 * rest. Env: LED_REV (one model), LED_SONY (0 = our gated match),
 * LED_OWN (0 = Sony's acceptance only), LED_PRIOR (IMU priors),
 * LED_RANSAC (0 = the gravity-only RANSAC boot gets no IMU at all),
 * LED_MIRROR (1 = the left-hand model, mirrored in x).
 *
 * Pass, per model: track and boot succeed often enough, and at most 2.5% of
 * trials accept a wrong pose. A near-planar LED ring seen by one camera has
 * a two-fold pose ambiguity that one frame cannot always resolve; the IMU
 * fusion downstream has to catch those. A pose counts as right within 5 mm and 4 deg (0.3 px noise on a
 * ~9 cm LED spread gives ~1 deg orientation noise, up to ~3 deg with few
 * LEDs in view).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "psvr2_led_solver.h"

#define TRIALS 200

static void rot(const double w[3], double R[9])
{
	const double th = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	double k[3], c, s, v;

	if (th < 1e-12) {
		const double I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
		memcpy(R, I, sizeof(I));
		return;
	}
	for (int i = 0; i < 3; i++)
		k[i] = w[i] / th;
	c = cos(th); s = sin(th); v = 1 - c;
	R[0] = c + k[0] * k[0] * v;        R[1] = k[0] * k[1] * v - k[2] * s; R[2] = k[0] * k[2] * v + k[1] * s;
	R[3] = k[1] * k[0] * v + k[2] * s; R[4] = c + k[1] * k[1] * v;        R[5] = k[1] * k[2] * v - k[0] * s;
	R[6] = k[2] * k[0] * v - k[1] * s; R[7] = k[2] * k[1] * v + k[0] * s; R[8] = c + k[2] * k[2] * v;
}

static void mm(const double A[9], const double B[9], double C[9])
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
}

static double gauss(void)
{
	const double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);

	return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static void perturb(const struct psvr2_pose3 *in, double rad, double mm_sd, struct psvr2_pose3 *out)
{
	double w[3] = { rad * gauss(), rad * gauss(), rad * gauss() }, dR[9];

	rot(w, dR);
	mm(dR, in->R, out->R);
	for (int i = 0; i < 3; i++)
		out->t[i] = in->t[i] + mm_sd * gauss();
}

/* Off by more than 20 mm or 10 deg. */
static int gross(const struct psvr2_pose3 *a, const struct psvr2_pose3 *b)
{
	double d = 0, tr = 0;

	for (int i = 0; i < 3; i++)
		d += (a->t[i] - b->t[i]) * (a->t[i] - b->t[i]);
	for (int i = 0; i < 9; i++)
		tr += a->R[i] * b->R[i];
	return sqrt(d) > 20 || acos(fmax(-1, fmin(1, (tr - 1) / 2))) * 180 / M_PI > 10;
}

/* 1 = right, 0 = wrong. */
static int right(const struct psvr2_pose3 *a, const struct psvr2_pose3 *b)
{
	double d = 0, tr = 0;

	for (int i = 0; i < 3; i++)
		d += (a->t[i] - b->t[i]) * (a->t[i] - b->t[i]);
	for (int i = 0; i < 9; i++)
		tr += a->R[i] * b->R[i];
	return sqrt(d) < 5 && acos(fmax(-1, fmin(1, (tr - 1) / 2))) * 180 / M_PI < 4;
}

/*
 * Sony's close-range and edge states (psvr2_led_track_state). Blobs carry
 * ellipse angles across the projected LED normal (5 deg noise), as the
 * grazing check expects.
 *  close: the controller 90-170 mm in front of camera 0, tracked from a
 *    prediction 2 deg / 3 mm off with an IMU orientation 1 deg off;
 *  edge: one camera, the controller 500 mm away towards an image corner
 *    (outside 1.24 fx of the centre), so <= 3 LEDs are predicted inside.
 * Each compares the state machine with the normal track step on the same
 * input. Right: within 5 mm / 4 deg; gross: off by > 20 mm or > 10 deg.
 * Pass: <= 5 % gross (edge: 10 %; the normal step has 2-5 % there);
 * close: right at least as often as the normal step
 * less 5 %. The edge state is less accurate than the normal step here
 * (fewer LEDs per solve, one match round); blobs cut by the image border,
 * which it is for on hardware, are not simulated.
 */
static void blobs_for(const struct psvr2_rig_camera cams[], const struct psvr2_led_model *mod,
		      const struct psvr2_pose3 *truth, struct psvr2_blob_list bl[])
{
	for (int c = 0; c < PSVR2_SOLVER_CAMS; c++) {
		struct psvr2_blob_list *b = &bl[c];
		memset(b, 0, sizeof(*b));
		b->has_angle = 1;
		if (!cams[c].active)
			continue;
		for (int l = 0; l < mod->count && b->count < PSVR2_SOLVER_BLOBS; l++) {
			double px[2], q[2], va;
			struct psvr2_pose3 moved = *truth;
			if (psvr2_led_project(&cams[c], mod, truth, l, px, &va, NULL) || va >= 100)
				continue;
			/* The image direction of the LED normal: project the LED
			 * moved 1 mm along it (pose shifted by R n). */
			for (int i = 0; i < 3; i++)
				moved.t[i] += truth->R[3 * i] * mod->nrm[l][0] + truth->R[3 * i + 1] * mod->nrm[l][1] +
					      truth->R[3 * i + 2] * mod->nrm[l][2];
			b->angle[b->count] = -1;
			if (psvr2_led_project(&cams[c], mod, &moved, l, q, NULL, NULL) == 0 &&
			    hypot(q[0] - px[0], q[1] - px[1]) > 1e-6)
				b->angle[b->count] = atan2(q[1] - px[1], q[0] - px[0]) + M_PI / 2 + 2 * M_PI +
						     0.087 * gauss();
			b->xy[b->count][0] = px[0] + 0.3 * gauss();
			b->xy[b->count][1] = px[1] + 0.3 * gauss();
			b->count++;
		}
	}
}

static int run_close(int rev, const struct psvr2_rig_camera cams_in[])
{
	struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS];
	struct psvr2_led_model mod;
	struct psvr2_solver_cfg cfg;
	int ok = 1;

	psvr2_led_model_load(rev, &mod);
	psvr2_solver_default_cfg(&cfg);
	if (getenv("LED_SONY"))
		cfg.sony_match = atoi(getenv("LED_SONY"));
	srand(2);
	for (int edge = 0; edge < 2; edge++) {
		int st_ok = 0, st_gross = 0, ex_ok = 0, ex_gross = 0, entered = 0, n = 0;
		memcpy(cams, cams_in, sizeof(cams));
		if (edge)
			for (int c = 1; c < PSVR2_SOLVER_CAMS; c++)
				cams[c].active = 0;
		for (int trial = 0; trial < TRIALS; trial++) {
			struct psvr2_blob_list bl[PSVR2_SOLVER_CAMS];
			struct psvr2_pose3 truth, start, out;
			struct psvr2_solve_result res;
			struct psvr2_imu_prior pr = { 0 };
			const struct psvr2_pose3 id = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 0 } };
			double dir[3], dc[3];
			struct psvr2_track_mem mem = { 0 };

			perturb(&id, 0.4, 0, &truth);
			if (edge) {
				/* Camera-frame direction ~65-80 deg off axis, towards a corner. */
				const double th = (65 + 15 * rand() / (double)RAND_MAX) * M_PI / 180;
				const double ph = (rand() % 4) * M_PI / 2 + M_PI / 4;
				dc[0] = sin(th) * cos(ph);
				dc[1] = sin(th) * sin(ph);
				dc[2] = cos(th);
			} else {
				dc[0] = 0.1 * gauss();
				dc[1] = 0.1 * gauss();
				dc[2] = 1;
			}
			for (int i = 0; i < 3; i++)
				dir[i] = cams[0].R[3 * i] * dc[0] + cams[0].R[3 * i + 1] * dc[1] + cams[0].R[3 * i + 2] * dc[2];
			{
				const double dn = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
				const double d = edge ? 500 : 90 + 80 * rand() / (double)RAND_MAX;
				for (int i = 0; i < 3; i++)
					truth.t[i] = cams[0].t[i] + d * dir[i] / dn;
			}
			blobs_for(cams, &mod, &truth, bl);
			if (bl[0].count < 3)
				continue;
			perturb(&truth, 0.035, 3, &start);
			{
				struct psvr2_pose3 g;
				perturb(&truth, 0.017, 0, &g);
				memcpy(pr.R, g.R, sizeof(pr.R));
				pr.orient_valid = 1;
			}
			n++;
			mem.state = edge ? PSVR2_TRACK_NORMAL : psvr2_led_boot_state(cams, &truth);
			{
				const int st0 = mem.state;
				if (psvr2_led_track_state(&cfg, cams, bl, &mod, &start, &pr, 1.0 / 60, &mem, &out, &res) == 0) {
					st_ok += right(&out, &truth);
					st_gross += gross(&out, &truth);
				}
				entered += edge ? mem.state == PSVR2_TRACK_EDGE : st0 != PSVR2_TRACK_NORMAL;
			}
			if (psvr2_led_track_ex(&cfg, cams, bl, &mod, &start, &pr, 1.0 / 60, &out, &res) == 0) {
				ex_ok += right(&out, &truth);
				ex_gross += gross(&out, &truth);
			}
		}
		printf("rev %d: %s %d trials (%s %d): states %d right, %d gross; normal step %d right, %d gross\n",
		       rev, edge ? "edge" : "close", n, edge ? "edge state after" : "started near/close", entered,
		       st_ok, st_gross, ex_ok, ex_gross);
		ok &= (edge ? 10 : 20) * st_gross <= n && (edge || 20 * st_ok >= 19 * ex_ok - n);
	}
	return ok;
}

static int run_model(int rev)
{
	static const double ct[4][3] = {
		{ 40, -41.68, 93.58 }, { -40, -41.68, 93.58 }, { 61.5, 22.97, 97.03 }, { -61.5, 22.97, 97.03 },
	};
	static const double toe[4] = { 0.35, -0.35, 0.7, -0.7 };
	struct psvr2_rig_camera cams[PSVR2_SOLVER_CAMS] = { 0 };
	struct psvr2_led_model mod;
	struct psvr2_solver_cfg cfg;
	int track_ok = 0, track_wrong = 0, boot_ok = 0, boot_wrong = 0;
	int rp_ok = 0, rp_wrong = 0, rn_ok = 0, rn_wrong = 0, rp_gross = 0, rn_gross = 0;
	/* Blobs for LEDs < 100 deg off-axis: that gives 28 matches per pose,
	 * as on a recorded hardware frame (28-30, docs/development.md part A). LED_VA sets it. */
	const double max_va = getenv("LED_VA") ? atof(getenv("LED_VA")) : 100;
	double matched_sum = 0, support_sum = 0, ransac_s = 0;
	int support_low = 0, few_n = 0, few_ok = 0, few_wrong = 0, sticky_n = 0, sticky_ok = 0;

	for (int c = 0; c < 4; c++) {
		struct psvr2_cam_model *m = &cams[c].model;
		const double I[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, w[3] = { 0, toe[c], 0 };
		cams[c].active = 1;
		m->fx = m->fy = 180;
		m->cx = m->cy = 253.5;
		m->width = m->height = PSVR2_CAM_MODEL_SIZE;
		m->num[0] = -0.08; m->num[1] = 0.01; m->den[0] = 0.02;
		m->p1 = 1e-4; m->p2 = -2e-4;
		memcpy(m->tilt, I, sizeof(I));
		rot(w, cams[c].R);
		memcpy(cams[c].t, ct[c], sizeof(ct[c]));
	}
	psvr2_led_model_load(rev, &mod);
	if (getenv("LED_MIRROR") && atoi(getenv("LED_MIRROR")))
		psvr2_led_model_mirror_x(&mod);	/* the left-hand model */
	psvr2_solver_default_cfg(&cfg);
	if (getenv("LED_SONY"))
		cfg.sony_match = atoi(getenv("LED_SONY"));
	if (getenv("LED_FEW"))
		cfg.few_led = atoi(getenv("LED_FEW"));
	if (getenv("LED_OWN"))
		cfg.own_accept = atoi(getenv("LED_OWN"));
	srand(1);
	for (int trial = 0; trial < TRIALS; trial++) {
		struct psvr2_blob_list bl[PSVR2_SOLVER_CAMS] = { 0 };
		struct psvr2_pose3 truth, start, out;
		struct psvr2_solve_result res;
		const struct psvr2_pose3 id = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 0 } };

		perturb(&id, 0.4, 0, &truth);
		truth.t[0] = 150 * gauss();
		truth.t[1] = 100 * gauss();
		truth.t[2] = 400 + 150 * fabs(gauss());
		for (int c = 0; c < 4; c++) {
			struct psvr2_blob_list *b = &bl[c];
			for (int l = 0; l < mod.count; l++) {
				double px[2], va;
				if (psvr2_led_project(&cams[c], &mod, &truth, l, px, &va, NULL) == 0 && va < max_va) {
					b->xy[b->count][0] = px[0] + 0.3 * gauss();
					b->xy[b->count][1] = px[1] + 0.3 * gauss();
					b->count++;
				}
			}
			for (int k = 0; k < 2; k++, b->count++) {
				b->xy[b->count][0] = 508.0 * rand() / RAND_MAX;
				b->xy[b->count][1] = 508.0 * rand() / RAND_MAX;
			}
			for (int i = b->count - 1; i > 0; i--) {
				const int j = rand() % (i + 1);
				double t[2];
				memcpy(t, b->xy[i], sizeof(t));
				memcpy(b->xy[i], b->xy[j], sizeof(t));
				memcpy(b->xy[j], t, sizeof(t));
			}
		}
		/* Track from a prediction ~2 deg / 9 mm off. LED_PRIOR=1 adds IMU
		 * priors: gravity ~1 deg off, orientation ~3 deg off. */
		perturb(&truth, 0.02, 5, &start);
		struct psvr2_imu_prior pr = { 0 };
		{
			struct psvr2_pose3 g;
			const double up[3] = { 0, -1, 0 };
			perturb(&truth, 0.017, 0, &g);
			pr.gravity_valid = pr.orient_valid = 1;
			memcpy(pr.g_rig, up, sizeof(up));
			for (int i = 0; i < 3; i++)	/* g_model = R^T up */
				pr.g_model[i] = g.R[i] * up[0] + g.R[3 + i] * up[1] + g.R[6 + i] * up[2];
			perturb(&truth, 0.05, 0, &g);
			memcpy(pr.R, g.R, sizeof(pr.R));
		}
		if (psvr2_led_track_ex(&cfg, cams, bl, &mod, &start, getenv("LED_PRIOR") ? &pr : NULL, 1.0 / 60,
				       &out, &res) == 0) {
			matched_sum += res.matched;
			support_sum += psvr2_led_support(&cfg, cams, bl, &mod, &out, &res);
			if (psvr2_led_support(&cfg, cams, bl, &mod, &out, &res) < 0.5)
				support_low++;
			if (right(&out, &truth))
				track_ok++;
			else
				track_wrong++;
		}
		/* Boot with only an IMU orientation ~5 deg off. */
		perturb(&truth, 0.03, 0, &start);
		if (psvr2_led_boot_oriented(&cfg, cams, bl, &mod, start.R, &out, &res) == 0) {
			if (right(&out, &truth))
				boot_ok++;
			else
				boot_wrong++;
		}
		/* Sony's RANSAC boot: with that orientation and gravity ~1 deg
		 * off (mode 7), and with gravity only (mode 6; the driver's case
		 * when the heading is unknown). LED_RANSAC=0: no IMU at all. */
		struct psvr2_imu_prior pb = pr, pg = pr;
		memcpy(pb.R, start.R, sizeof(pb.R));
		pg.orient_valid = 0;
		const clock_t t0 = clock();
		const int rp_ret = psvr2_led_boot_ransac(&cfg, cams, bl, &mod, &pb, 0, &out, &res);
		ransac_s += (double)(clock() - t0) / CLOCKS_PER_SEC;
		if (rp_ret == 0) {
			rp_ok += right(&out, &truth);
			rp_wrong += !right(&out, &truth);
			rp_gross += gross(&out, &truth);
		}
		if (psvr2_led_boot_ransac(&cfg, cams, bl, &mod,
					  getenv("LED_RANSAC") && !atoi(getenv("LED_RANSAC")) ? NULL : &pg, 0, &out,
					  &res) == 0) {
			rn_ok += right(&out, &truth);
			rn_wrong += !right(&out, &truth);
			rn_gross += gross(&out, &truth);
		}
		/*
		 * Occlusion: camera 0 sees only a patch of 1-3 neighbouring LEDs
		 * (the most face-on one and its nearest in the image), no clutter.
		 * With an IMU orientation ~1 deg off, Sony's few-LED solve keeps the
		 * pose; right = within 40 mm. Two LEDs ~6 px apart at 600 mm with
		 * 0.3 px noise give 20-35 mm depth error from the pair geometry
		 * alone (Sony's solve has the same), so a tighter cut counts noise,
		 * not wrong associations. Sony's gated match needs 2 supporting
		 * neighbours per LED, so it matches nothing with 1-2 isolated blobs:
		 * those trials then only count wrong poses (cfg.sony_match), not
		 * misses, and about half the 3-LED patches still miss.
		 */
		{
			struct psvr2_blob_list ob[PSVR2_SOLVER_CAMS] = { 0 };
			struct psvr2_imu_prior fp = pr;
			struct psvr2_pose3 g;
			const int want = 1 + trial % 3;
			double vpx[PSVR2_LED_MAX][2], best = 1e9;
			int nv = 0, seed = -1, nb = 0;
			unsigned char taken[PSVR2_LED_MAX] = { 0 };
			for (int l = 0; l < mod.count; l++) {
				double va;
				if (psvr2_led_project(&cams[0], &mod, &truth, l, vpx[nv], &va, NULL) || va >= 60)
					continue;
				if (va < best) {
					best = va;
					seed = nv;
				}
				nv++;
			}
			for (int q = 0; q < want && seed >= 0; q++) {
				int pick = -1;
				double bd = 1e18;
				for (int t = 0; t < nv; t++) {
					const double dx = vpx[t][0] - vpx[seed][0], dy = vpx[t][1] - vpx[seed][1];
					if (!taken[t] && dx * dx + dy * dy < bd) {
						bd = dx * dx + dy * dy;
						pick = t;
					}
				}
				if (pick < 0)
					break;
				taken[pick] = 1;
				ob[0].xy[nb][0] = vpx[pick][0] + 0.3 * gauss();
				ob[0].xy[nb][1] = vpx[pick][1] + 0.3 * gauss();
				nb++;
			}
			ob[0].count = nb;
			perturb(&truth, 0.017, 0, &g);
			memcpy(fp.R, g.R, sizeof(fp.R));
			fp.orient_valid = fp.gravity_valid = 1;
			fp.p1p_frames = 0;
			if (nb == want) {
				struct psvr2_track_mem fm = { 0 };
				const int counted = want == 3;
				few_n += counted;
				if (psvr2_led_track_state(&cfg, cams, ob, &mod, &start, &fp, 1.0 / 60, &fm, &out, &res) == 0) {
					double d = 0;
					for (int i = 0; i < 3; i++)
						d += (out.t[i] - truth.t[i]) * (out.t[i] - truth.t[i]);
					if (sqrt(d) >= 40)
						few_wrong++;
					else if (counted)
						few_ok++;
					/*
					 * Sony keeps the IMU-rotation solve after a few-LED
					 * frame (FUN_1800dcf00 +0x4c98) until 16 frames with
					 * >= 5 matches: with all LEDs back, frames 1-15 are
					 * P2P (method 2), frame 16 a full refine.
					 */
					if (fm.few_mode && sqrt(d) < 40) {
						struct psvr2_pose3 cur = out, o2;
						struct psvr2_solve_result r2;
						int seq_ok = 1;
						for (int f = 1; f <= 17; f++) {
							const int rr = psvr2_led_track_state(&cfg, cams, bl, &mod, &cur, &fp, 1.0 / 60,
											     &fm, &o2, &r2);
							seq_ok &= rr == 0 && r2.method == (f < 16 ? 2 : 0) && !gross(&o2, &truth);
							if (rr == 0)
								cur = o2;
						}
						sticky_n++;
						sticky_ok += seq_ok;
					}
				}
			}
		}
	}
	const int close_ok = run_close(rev, cams);
	printf("rev %d: RANSAC boot, orientation + gravity %d/%d right, %d wrong (%d gross), %.1f ms; "
	       "gravity only %d/%d right, %d wrong (%d gross)\n",
	       rev, rp_ok, TRIALS, rp_wrong, rp_gross, 1000 * ransac_s / TRIALS, rn_ok, TRIALS, rn_wrong, rn_gross);
	printf("rev %d: few-LED (1-3 seen) %d/%d right, %d wrong; few mode kept 15 frames %d/%d\n", rev, few_ok,
	       few_n, few_wrong, sticky_ok, sticky_n);
	printf("rev %d: track %3d/%d right, %d wrong; boot %3d/%d right, %d wrong; %.1f LEDs matched, "
	       "support %.2f (%d < 0.5)\n",
	       rev, track_ok, TRIALS, track_wrong, boot_ok, TRIALS, boot_wrong,
	       track_ok + track_wrong ? matched_sum / (track_ok + track_wrong) : 0.0,
	       track_ok + track_wrong ? support_sum / (track_ok + track_wrong) : 0.0, support_low);
	return track_ok >= TRIALS * 85 / 100 && boot_ok >= TRIALS * 90 / 100 &&
	       40 * track_wrong <= TRIALS && 40 * boot_wrong <= TRIALS &&
	       rp_ok >= TRIALS * 85 / 100 && rn_ok >= TRIALS * 70 / 100 && 40 * rp_wrong <= TRIALS &&
	       40 * rn_wrong <= TRIALS &&
	       (!cfg.few_led || (few_ok >= few_n * 30 / 100 && 20 * few_wrong <= few_n &&
				 10 * sticky_ok >= 9 * sticky_n)) && close_ok;
}

int main(void)
{
	int ok = 1, lo = 1, hi = 7;
	const char *one = getenv("LED_REV");

	/* LED_REV=n runs model n only. */
	if (one)
		lo = hi = atoi(one);
	for (int rev = lo; rev <= hi; rev++)
		ok &= run_model(rev);
	printf("%s\n", ok ? "PASS" : "FAIL");
	return !ok;
}
