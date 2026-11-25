// SPDX-License-Identifier: GPL-2.0
/* Gyro still detection, IMU conversion and orientation (docs/sony-re/controllers.md part A, "Motion"). */
#include <math.h>
#include <string.h>
#include "psvr2_libpad.h"
#if defined(__SSE__) || defined(__x86_64__)
#include <xmmintrin.h>
static float fsqrt(float x) { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(x))); }
#else
static float fsqrt(float x) { return sqrtf(x); }
#endif

/* Sense IMU mounting: rotation about X (row-major 3x3 with 4-float rows). */
float psvr2_imu_axes[12] = {
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 0.639166533946991f, 0.7690683603286743f, 0.0f,
	0.0f, -0.7690683603286743f, 0.639166533946991f, 0.0f,
};

void psvr2_gyro_still_reset(psvr2_gyro_still *s)
{
	memset(s, 0, sizeof *s);
}

void psvr2_gyro_still_thresholds(psvr2_gyro_still *s)
{
	/* Twice the squared allowed spread, per axis. */
	for (int i = 0; i < 3; i++) {
		const float f = s->range[i] * s->k_fast;
		s->fast_thr[i] = f * f + f * f;
	}
	for (int i = 0; i < 3; i++) {
		const float f = s->k_slow * s->range[i];
		s->slow_thr[i] = f * f + f * f;
	}
}

static void still_init(psvr2_gyro_still *s, const int16_t bias[3], const int16_t ref[3],
		       int16_t speed, float k_fast, uint16_t win, float slow_frac)
{
	s->flags = 3;
	for (int i = 0; i < 3; i++)
		s->bias[i] = (float)bias[i];
	for (int i = 0; i < 3; i++)
		s->range[i] = fabsf((float)((int)ref[i] - (int)bias[i])) / (float)speed;
	s->k_fast = k_fast;
	s->k_slow = 3.0f;
	s->win_len = win;
	for (int i = 0; i < 3; i++) {
		s->fast_max[i] = s->range[i] * 6.0f;
		s->slow_max[i] = slow_frac == 1.0f ? s->range[i] : s->range[i] * slow_frac;
	}
	psvr2_gyro_still_thresholds(s);
	s->enabled = 1;
}

void psvr2_gyro_still_init_ps5(psvr2_gyro_still *s, const int16_t c[42])
{
	if (!c[39] || !c[40])
		return;
	const int16_t bias[3] = { c[18], c[19], c[20] }, ref[3] = { c[21], c[28], c[35] };
	still_init(s, bias, ref, c[39], 2.0f, 100, 1.0f);
}

void psvr2_gyro_still_init_ds4(psvr2_gyro_still *s, const int16_t c[8])
{
	/* c: bias xyz, plus xyz, speed, speed2 */
	if (!c[6] || !c[7])
		return;
	still_init(s, c, c + 3, c[6], 1.0f, 250, 0.5f);
}

static float clampf(float d, float m)
{
	if (!(d <= m))
		return m;
	return -m <= d ? d : -m;
}

int psvr2_gyro_still_update(psvr2_gyro_still *s, int16_t x, int16_t y, int16_t z,
			    float bias[3], float corr[3])
{
	if (!s->enabled)
		return 0;
	const int32_t v[3] = { x, y, z };
	s->count++;
	for (int i = 0; i < 3; i++)
		s->sum[i] = (int32_t)((uint32_t)s->sum[i] + (uint32_t)v[i]);
	for (int i = 0; i < 3; i++) {
		if (s->count == 1) {
			s->max[i] = s->min[i] = v[i];
		} else {
			if (v[i] > s->max[i])
				s->max[i] = v[i];
			if (v[i] < s->min[i])
				s->min[i] = v[i];
		}
	}
	if (s->count != s->win_len)
		return 0;

	const uint32_t win = ++s->windows;
	const float len = (float)s->win_len;
	float mean[3], var[3];
	mean[1] = (float)s->sum[1] / len;
	mean[2] = (float)s->sum[2] / len;
	mean[0] = (float)s->sum[0] / len;
	for (int i = 0; i < 3; i++) {
		const float lo = mean[i] - (float)s->min[i], hi = (float)s->max[i] - mean[i];
		var[i] = lo * lo + hi * hi;
	}
	int ret = 0;
	const int fast = var[0] < s->fast_thr[0] && var[1] < s->fast_thr[1] &&
			 var[2] < s->fast_thr[2] && (s->flags & 1);
	if (fast) {
		for (int i = 0; i < 3; i++)
			bias[i] = s->bias[i] - clampf(s->bias[i] - mean[i], s->fast_max[i]);
		ret = 1;
	} else {
		if (var[0] < s->slow_thr[0] && var[1] < s->slow_thr[1] &&
		    var[2] < s->slow_thr[2] && (s->flags & 2)) {
			for (int i = 0; i < 3; i++)
				s->acc_sum[i] = (int32_t)((uint32_t)s->acc_sum[i] + (uint32_t)s->sum[i]);
			s->acc_n++;
		}
		if (win != 6)
			goto window;
		if (s->acc_n > 3) {
			const float n = (float)(uint32_t)(s->acc_n * s->win_len);
			for (int i = 0; i < 3; i++)
				bias[i] = s->bias[i] - clampf(s->bias[i] - (float)s->acc_sum[i] / n, s->slow_max[i]);
			ret = 1;
		}
	}
	if (ret) {
		/* Applied correction, new minus old. */
		for (int i = 0; i < 3; i++)
			corr[i] = s->bias[i] <= bias[i] ? bias[i] - s->bias[i] : -(s->bias[i] - bias[i]);
		memcpy(s->bias, bias, sizeof s->bias);
	}
	memset(s->acc_sum, 0, sizeof s->acc_sum);
	s->acc_n = 0;
	s->windows = 0;
window:
	s->count = 0;
	memset(s->max, 0, sizeof s->max);
	memset(s->min, 0, sizeof s->min);
	memset(s->sum, 0, sizeof s->sum);
	return ret;
}

void psvr2_mat3_inverse(float o[12], const float m[12])
{
	/* Adjugate over determinant, rows of 4 floats. */
	const float c0 = m[1] * m[6] - m[2] * m[5];
	const float c1 = m[2] * m[4] - m[0] * m[6];
	const float c2 = m[0] * m[5] - m[1] * m[4];
	const float inv = 1.0f / (c0 * m[8] + c1 * m[9] + c2 * m[10]);
	o[0] = (m[5] * m[10] - m[6] * m[9]) * inv;
	o[1] = (m[2] * m[9] - m[1] * m[10]) * inv;
	o[2] = c0 * inv;
	o[6] = c1 * inv;
	o[10] = c2 * inv;
	o[4] = (m[6] * m[8] - m[4] * m[10]) * inv;
	o[5] = (m[0] * m[10] - m[2] * m[8]) * inv;
	o[8] = (m[4] * m[9] - m[5] * m[8]) * inv;
	o[9] = (m[1] * m[8] - m[0] * m[9]) * inv;
}

void psvr2_tilt_correct(float q[4], const float a[3], float dt)
{
	const float ax = a[0], ay = a[1], az = a[2];
	const float mag = fsqrt(ax * ax + ay * ay + az * az);
	if (!(fabsf(mag - 1.0f) < 0.08f))
		return;
	/* Gravity in the world frame: q a q*. */
	const float x = q[0], y = q[1], z = q[2], w = q[3];
	const float t0 = (y * az + w * ax) - z * ay;
	const float t1 = (z * ax + w * ay) - x * az;
	const float t2 = (w * az + x * ay) - y * ax;
	const float t3 = x * ax + y * ay + z * az;
	const float vx = ((x * t3 + w * t0) - z * t1) + y * t2;
	const float vy = ((w * t1 + y * t3) - x * t2) + z * t0;
	const float vz = ((z * t3 + w * t2) - y * t0) + x * t1;
	const float k = 1.0f / fsqrt(vx * vx + vy * vy + vz * vz);
	/* Axis = v x (0, 1, 0), written with the zero terms kept. */
	const float zx = k * vx * 0.0f, zy = k * vy * 0.0f, zz = k * vz * 0.0f;
	const float e2 = k * vx - zy;
	const float e0 = zy - k * vz;
	const float e1 = zz - zx;
	const float s = fsqrt(e0 * e0 + e1 * e1 + e2 * e2);
	if (!(0.01f < s))
		return;
	const float ang = atan2f(s, zx + k * vy + zz);
	const float g = dt / 0.008f;
	const float gain = 2.0f < g ? 2.0f : g;	/* minss: NaN passes through */
	const float h = ang * (gain * 0.04f) * 0.5f;
	const float sn = sinf(h) / s;
	const float dq[4] = { e0 * sn, e1 * sn, e2 * sn, cosf(h) };
	float r[4];
	psvr2_pad_quat_mul(dq, r, q);
	memcpy(q, r, sizeof r);
}

void psvr2_motion_update(psvr2_motion *m, psvr2_motion_out *o, const psvr2_motion_out *prev,
			 int is_dualsense, const psvr2_sense_input *in)
{
	const uint32_t n = m->count;
	if ((m->flags & 1) && n > 9) {
		float b[3], corr[3];
		if (psvr2_gyro_still_update(&m->still, in->gyro[0], in->gyro[1], in->gyro[2], b, corr))
			memcpy(m->gyro_bias, b, sizeof b);
	}
	o->ts = in->imu_ticks;
	o->temperature = in->temperature;
	if (n < 10) {
		/* Warm-up: report rest, identity orientation. */
		memset(o->gyro, 0, sizeof o->gyro);
		o->accel[0] = 0.0f;
		o->accel[1] = 1.0f;
		o->accel[2] = 0.0f;
		memset(m->q, 0, 3 * sizeof(float));
		m->q[3] = 1.0f;
		memset(o->q, 0, 3 * sizeof(float));
		o->q[3] = 1.0f;
		m->count++;
		return;
	}
	float ax = ((float)in->accel[0] - m->accel_off[0]) * m->accel_scale[0];
	float ay = ((float)in->accel[1] - m->accel_off[1]) * m->accel_scale[1];
	float az = ((float)in->accel[2] - m->accel_off[2]) * m->accel_scale[2];
	float gx = ((float)in->gyro[0] - m->gyro_bias[0]) * m->gyro_scale[0];
	float gy = ((float)in->gyro[1] - m->gyro_bias[1]) * m->gyro_scale[1];
	float gz = ((float)in->gyro[2] - m->gyro_bias[2]) * m->gyro_scale[2];
	if (m->axis & 1) {
		float r[12];
		psvr2_mat3_inverse(r, psvr2_imu_axes);
		const float nx = ay * r[4] + ax * r[0] + az * r[8];
		const float ny = ay * r[5] + ax * r[1] + az * r[9];
		const float nz = az * r[10] + ay * r[6] + ax * r[2];
		ax = nx, ay = ny, az = nz;
	}
	if (m->axis & 2) {
		float r[12];
		psvr2_mat3_inverse(r, psvr2_imu_axes);
		const float nx = gy * r[4] + gx * r[0] + gz * r[8];
		const float ny = gy * r[5] + gx * r[1] + gz * r[9];
		const float nz = gz * r[10] + gx * r[2] + gy * r[6];
		gx = nx, gy = ny, gz = nz;
	}
	o->accel[0] = ax;
	o->accel[1] = ay;
	o->accel[2] = az;
	if (m->flags & 2) {
		/* Dead band of +-1 deg/s. */
		const float d = 0.01745329238474369f;
		if (-d <= gx && gx <= d)
			gx = 0.0f;
		if (-d <= gy && gy <= d)
			gy = 0.0f;
		if (-d <= gz && gz <= d)
			gz = 0.0f;
	}
	o->gyro[0] = gx;
	o->gyro[1] = gy;
	o->gyro[2] = gz;
	if (!prev)
		return;

	uint32_t dts = o->ts - prev->ts;
	if (o->ts < prev->ts && !is_dualsense)
		dts += 0x10000;
	const float dt = (float)dts * m->tick_scale * 1e-6f;
	const float hw = m->q[3] * 0.5f, hy = m->q[1] * 0.5f, hz = m->q[2] * 0.5f, hx = m->q[0] * 0.5f;
	const float x = ((hw * gx + hx * 0.0f + hy * gz) - hz * gy) * dt + m->q[0];
	const float y = ((hw * gy + hy * 0.0f + hz * gx) - hx * gz) * dt + m->q[1];
	const float z = ((hw * gz + hz * 0.0f + hx * gy) - hy * gx) * dt + m->q[2];
	const float w = (((hw * 0.0f - hx * gx) - hy * gy) - hz * gz) * dt + m->q[3];
	const float k = 1.0f / fsqrt(x * x + y * y + z * z + w * w);
	float q[4] = { k * x, k * y, k * z, k * w };
	if (m->flags & 4) {
		const float a[3] = { ax, ay, az };
		psvr2_tilt_correct(q, a, dt);
	}
	memcpy(m->q, q, sizeof q);
	memcpy(o->q, q, sizeof q);
}

/* DS4 / DualSense calibration (psvr2_pad_imu_cal_parse layout) to the
 * motion conversion. Fails, changing nothing, when an axis has no range. */
int32_t psvr2_motion_cal_ds(psvr2_motion *m, uint16_t vid, uint16_t pid, int32_t hid_ver,
			    int16_t cal[18])
{
	const float bx = (float)cal[0], by = (float)cal[1], bz = (float)cal[2];
	float sp, sm;
	if (psvr2_pad_is_ds4_new(vid, pid, hid_ver) && hid_ver == 0x11) {
		/* This revision reports a bad speed: use 540 deg/s. */
		cal[9] = cal[10] = 540;
		sp = sm = 540.0f;
	} else {
		sp = (float)cal[9];
		sm = (float)cal[10];
	}
	float rx = fabsf((float)cal[3] - bx) + fabsf((float)cal[4] - bx);
	float ry = fabsf((float)cal[5] - by) + fabsf((float)cal[6] - by);
	float rz = fabsf((float)cal[7] - bz) + fabsf((float)cal[8] - bz);
	if (rx == 0.0f || ry == 0.0f || rz == 0.0f)
		return (int32_t)0x80010002;
	rx *= 0.5f;
	ry *= 0.5f;
	rz *= 0.5f;
	const float rad = (sm + sp) * 0.5f * 0.01745329238474369f;
	m->gyro_bias[0] = bx;
	m->gyro_bias[1] = by;
	m->gyro_bias[2] = bz;
	m->gyro_scale[0] = rad / rx;
	m->gyro_scale[1] = rad / ry;
	m->gyro_scale[2] = rad / rz;
	for (int i = 0; i < 3; i++) {
		const float hi = (float)cal[11 + 2 * i], lo = (float)cal[12 + 2 * i];
		const float span = fabsf(hi) + fabsf(lo);
		m->accel_off[i] = hi - span * 0.5f;
		m->accel_scale[i] = 2.0f / span;
	}
	if (psvr2_pad_is_dualsense(vid, pid))
		m->tick_scale = 1.0f / 3.0f;
	else
		m->tick_scale = hid_ver < 0x13 ? 7.1f : 16.0f / 3.0f;
	m->axis = 0;
	m->calibrated = 1;
	return 0;
}
