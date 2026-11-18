#include "sony.h"
#include "ct_difftest.h"
#include "psvr2_ct_math.h"
#include "psvr2_ct_noise.h"

typedef float *(*f_oab)(float *, const float *, const float *);
typedef float *(*f_aob)(const float *, float *, const float *);
typedef float *(*f_oi)(float *, const float *);
typedef float *(*f_of)(float *, float);
typedef float (*f_az)(float *, float *, const float *, const float *);
typedef float (*f_rng)(uint32_t *, float, float);
typedef float *(*f_noise)(const void *, float *, const void *);

static float rangle(void)
{
	return (rnd() & 7) == 0 ? rf() : ((int32_t)rnd()) * (1.0f / 2147483648.0f) * 10.0f;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	const int n = 200000;
	int fail = 0, bad;

	bad = 0;
	for (int i = 0; i < n; i++) {
		float a[4], b[4], c[4], x[4], y[4];
		for (int k = 0; k < 4; k++) { a[k] = rf(); b[k] = rf(); c[k] = rf(); }
		memcpy(x, a, 16); memcpy(y, a, 16);
		((f_oi)SONY(0x180061be0))(x, b);
		psvr2_ct_vec3_add_inplace(y, b);
		bad += !feqn(x, y, 3);
		((f_oi)SONY(0x180061c60))(x, a);
		bad += !feq(x[0], psvr2_ct_vec3_length(a));
		((f_oab)SONY(0x180061cc0))(x, a, b);
		psvr2_ct_vec3_cross(y, a, b);
		bad += !feqn(x, y, 3);
		((f_aob)SONY(0x180061fa0))(a, x, c);
		psvr2_ct_quat_mul(y, a, c);
		bad += !feqn(x, y, 4);
	}
	fail |= report("vec3 add/len/cross, quat_mul", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float t = rangle(), x[4], y[4], X[3][4], Y[3][4];
		((f_of)SONY(0x1800620b0))(x, t);
		psvr2_ct_quat_yaw(y, t);
		bad += !feqn(x, y, 4);
		((f_of)SONY(0x1800623d0))(&X[0][0], t);
		psvr2_ct_mat3_rot_y(Y, t);
		for (int c = 0; c < 3; c++)
			bad += !feqn(X[c], Y[c], 3);
	}
	fail |= report("quat_yaw 0x1800620b0, rot_y 0x1800623d0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float A[3][4], B[3][4], X[3][4], Y[3][4];
		for (int k = 0; k < 12; k++) { A[k / 4][k % 4] = rf(); B[k / 4][k % 4] = rf(); }
		((f_aob)SONY(0x1800621a0))(&A[0][0], &X[0][0], &B[0][0]);
		psvr2_ct_mat3_mul(Y, A, B);
		for (int c = 0; c < 3; c++)
			bad += !feqn(X[c], Y[c], 3);
	}
	fail |= report("mat3_mul 0x1800621a0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float a[3], b[3], s1, s2, o1, o2;
		for (int k = 0; k < 3; k++) { a[k] = rf(); b[k] = rf(); }
		const float rs = ((f_az)SONY(0x1800627f0))(&s1, &s2, a, b);
		const float ro = psvr2_ct_azimuth_diff(&o1, &o2, a, b);
		bad += !feq(rs, ro) || !feq(s1, o1) || !feq(s2, o2);
	}
	fail |= report("azimuth_diff 0x1800627f0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		uint32_t s[2] = { rnd(), rnd() }, o[2] = { s[0], s[1] };
		const float lo = rf(), hi = rf();
		for (int k = 0; k < 4; k++) {
			const float rs = ((f_rng)SONY(0x180062bc0))(s, lo, hi);
			const float ro = psvr2_ct_mwc_uniform(o, lo, hi);
			bad += !feq(rs, ro) || s[0] != o[0] || s[1] != o[1];
		}
	}
	fail |= report("mwc_uniform 0x180062bc0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		uint8_t st[0x40] __attribute__((aligned(16))), cfg[0x50];
		float x[3], y[3], v[3];
		for (int k = 0; k < 0x50; k += 4) { float f = rf(); memcpy(cfg + k, &f, 4); }
		for (int k = 0; k < 0x40; k += 4) { float f = rf(); memcpy(st + k, &f, 4); }
		const int second = i & 1;
		uint8_t *fl = st + (second ? 0x33 : 0x30);
		for (int k = 0; k < 3; k++) fl[k] = (rnd() & 3) == 0;
		memcpy(v, st + (second ? 0x10 : 0), 12);
		float g[12];
		memcpy(g, cfg, sizeof g);
		((f_noise)SONY(second ? 0x180062950 : 0x180062880))(st, x, cfg);
		float own, cross, extra, fixv, base[3];
		if (second) {
			memcpy(&own, cfg + 0x40, 4); memcpy(&cross, cfg + 0x44, 4);
			memcpy(&extra, cfg + 0x3c, 4); memcpy(&fixv, cfg + 0x48, 4);
			memcpy(base, cfg + 0x20, 12);
		} else {
			memcpy(&own, cfg + 0x30, 4); memcpy(&cross, cfg + 0x34, 4);
			extra = 0; memcpy(&fixv, cfg + 0x38, 4);
			memcpy(base, cfg + 0x10, 12);
		}
		psvr2_ct_axis_noise(y, v, fl, own, cross, base, second, extra, fixv);
		bad += !feqn(x, y, 3);
	}
	fail |= report("axis_noise 0x180062880/950", bad, n);
	return fail;
}
