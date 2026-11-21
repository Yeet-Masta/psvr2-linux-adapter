// SPDX-License-Identifier: GPL-2.0
/*
 * Synthetic check of the LED pose solver with Sony's LED models: four
 * cameras near the built-in rig positions, random poses 0.4-1 m out, 0.3 px
 * blob noise and two clutter blobs per camera. Blobs exist only for LEDs
 * facing a camera (< 100 deg; LED_VA), as the controller body hides the
 * rest. Env: LED_REV (one model), LED_SONY (0 = our gated match),
 * LED_OWN (0 = Sony's acceptance only), LED_PRIOR (IMU priors).
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
	/* Blobs for LEDs < 100 deg off-axis: that gives 28 matches per pose,
	 * as on a recorded hardware frame (28-30, DEV-NOTES.md). LED_VA sets it. */
	const double max_va = getenv("LED_VA") ? atof(getenv("LED_VA")) : 100;
	double matched_sum = 0;

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
	psvr2_solver_default_cfg(&cfg);
	if (getenv("LED_SONY"))
		cfg.sony_match = atoi(getenv("LED_SONY"));
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
	}
	printf("rev %d: track %3d/%d right, %d wrong; boot %3d/%d right, %d wrong; %.1f LEDs matched\n",
	       rev, track_ok, TRIALS, track_wrong, boot_ok, TRIALS, boot_wrong,
	       track_ok + track_wrong ? matched_sum / (track_ok + track_wrong) : 0.0);
	return track_ok >= TRIALS * 85 / 100 && boot_ok >= TRIALS * 90 / 100 &&
	       40 * track_wrong <= TRIALS && 40 * boot_wrong <= TRIALS;
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
