// SPDX-License-Identifier: GPL-2.0
/*
 * Tracking-camera calibration blob ("SIE", VD types 10/11) and the camera
 * projection model. Layout: docs/sony-re/overview.md part B, "Camera calibration
 * blob"; record offsets checked against FUN_180161b70. Model and mapping:
 * docs/sony-re/overview.md §A.8 (FUN_180182e80, FUN_180045420, FUN_180045650).
 */
#include <math.h>
#include <string.h>

#include "libpsvr2.h"

static unsigned int rd16(const unsigned char *p) { return p[0] | p[1] << 8; }
static float rdf(const unsigned char *p) { float f; memcpy(&f, p, 4); return f; }
static double rdd(const unsigned char *p) { double d; memcpy(&d, p, 8); return d; }

int psvr2_cam_calib_parse(const void *blob, unsigned int len,
			  struct psvr2_cam_calib *out)
{
	const unsigned char *b = blob;
	unsigned int i, j, rec, need;
	int colon = 0;

	memset(out, 0, sizeof(*out));
	if (len < 0x18 || memcmp(b, "SIE", 3) || b[3] > 1 || rd16(b + 4) != len)
		return -1;
	out->version = b[3];
	memcpy(out->model, b + 7, 8);
	/* Model string with ':' and a later 'd': double-precision records. */
	for (i = 0; i < 8; i++) {
		if (b[7 + i] == ':')
			colon = 1;
		else if (b[7 + i] == 'd' && colon)
			out->model_d = 1;
	}
	out->ncam = b[0x0f];
	out->next = b[0x10];
	rec = out->model_d ? 0xe4 : 0x94;
	need = 0x18 + out->ncam * rec + out->next * 0x34;
	if (need > len || out->ncam > PSVR2_CAM_MAX || out->next > PSVR2_CAM_EXT_MAX)
		return -1;

	for (i = 0; i < out->ncam; i++) {
		const unsigned char *r = b + 0x18 + i * rec;
		struct psvr2_cam_intrinsics *c = &out->cam[i];

		c->id = r[0];
		c->width = rd16(r + 4);
		c->height = rd16(r + 6);
		for (j = 0; j < 9; j++)
			c->K[j] = rdf(r + 8 + 4 * j);
		for (j = 0; j < PSVR2_CAM_RAW; j++)
			c->raw[j] = out->model_d ? rdd(r + 0x2c + 8 * j) : rdf(r + 0x2c + 4 * j);
	}
	for (i = 0; i < out->next; i++) {
		const unsigned char *r = b + 0x18 + out->ncam * rec + i * 0x34;
		struct psvr2_cam_extrinsic *e = &out->ext[i];

		e->from = r[0];
		e->to = r[1];
		for (j = 0; j < 9; j++)
			e->R[j] = rdf(r + 4 + 4 * j);
		for (j = 0; j < 3; j++)
			e->t[j] = rdf(r + 0x28 + 4 * j);
	}
	return 0;
}

static double poly(const double *c, double r2)
{
	double v = 0;
	int i;

	for (i = 5; i >= 0; i--)
		v = (v + c[i]) * r2;
	return 1 + v;	/* 1 + c0 r2 + c1 r2^2 + ... + c5 r2^6 */
}

/* OpenCV computeTiltProjectionMatrix(tau_x, tau_y), row-major. */
static void tilt_matrix(double tx, double ty, double T[9])
{
	const double cx = cos(tx), sx = sin(tx), cy = cos(ty), sy = sin(ty);
	/* R = Ry(ty) * Rx(tx) */
	const double R[9] = {
		cy, sy * sx, -sy * cx,
		0, cx, sx,
		sy, -cy * sx, cy * cx,
	};
	/* P = [[R22, 0, -R02], [0, R22, -R12], [0, 0, 1]]; T = P * R */
	const double P[9] = { R[8], 0, -R[2], 0, R[8], -R[5], 0, 0, 1 };
	int i, j, k;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			T[3 * i + j] = 0;
			for (k = 0; k < 3; k++)
				T[3 * i + j] += P[3 * i + k] * R[3 * k + j];
		}
}

int psvr2_cam_model_from_calib(const struct psvr2_cam_calib *cal, unsigned int index,
			       struct psvr2_cam_model *m)
{
	const struct psvr2_cam_intrinsics *c;
	const double *D;
	double sx, sy;

	if (index >= cal->ncam || !cal->cam[index].width || !cal->cam[index].height)
		return -1;
	c = &cal->cam[index];
	D = c->raw;
	memset(m, 0, sizeof(*m));
	sx = (double)PSVR2_CAM_MODEL_SIZE / c->width;
	sy = (double)PSVR2_CAM_MODEL_SIZE / c->height;
	m->width = m->height = PSVR2_CAM_MODEL_SIZE;
	m->fx = sx * c->K[0];
	m->fy = sy * c->K[4];
	m->cx = (c->K[2] + 0.5) * sx - 0.5;
	m->cy = (c->K[5] + 0.5) * sy - 0.5;
	/* D: k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4 tau_x tau_y n8 n10 n12 d8 d10 d12 */
	m->num[0] = D[0]; m->num[1] = D[1]; m->num[2] = D[4];
	m->num[3] = D[14]; m->num[4] = D[15]; m->num[5] = D[16];
	m->den[0] = D[5]; m->den[1] = D[6]; m->den[2] = D[7];
	m->den[3] = D[17]; m->den[4] = D[18]; m->den[5] = D[19];
	m->p1 = D[2];
	m->p2 = D[3];
	m->tau_x = D[12];
	m->tau_y = D[13];
	tilt_matrix(m->tau_x, m->tau_y, m->tilt);
	return 0;
}

int psvr2_cam_project(const struct psvr2_cam_model *m, const double X[3],
		      double px[2])
{
	double x, y, r2, rad, xd, yd, v[3];
	const double *T = m->tilt;

	if (X[2] <= 1e-4)
		return -1;
	x = X[0] / X[2];
	y = X[1] / X[2];
	r2 = x * x + y * y;
	if (r2 > 200.0)
		return -1;
	rad = poly(m->num, r2) / poly(m->den, r2);
	xd = x * rad + 2 * m->p1 * x * y + m->p2 * (r2 + 2 * x * x);
	yd = y * rad + m->p1 * (r2 + 2 * y * y) + 2 * m->p2 * x * y;
	v[0] = T[0] * xd + T[1] * yd + T[2];
	v[1] = T[3] * xd + T[4] * yd + T[5];
	v[2] = T[6] * xd + T[7] * yd + T[8];
	if (v[2] == 0)
		return -1;
	px[0] = m->cx + m->fx * v[0] / v[2];
	px[1] = m->cy + m->fy * v[1] / v[2];
	return 0;
}

int psvr2_cam_project_rig(const struct psvr2_cam_model *m, const double X[3],
			  double px[2])
{
	const double P[3] = { -X[0], -X[1], X[2] };

	return psvr2_cam_project(m, P, px);
}

void psvr2_cam_extrinsic_to_rig(const struct psvr2_cam_extrinsic *e,
				double R[9], double t_mm[3])
{
	static const double g[3] = { -1, -1, 1 };
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++)
			R[3 * i + j] = g[i] * e->R[3 * i + j] * g[j];
		t_mm[i] = 1000.0 * g[i] * e->t[i];
	}
}
