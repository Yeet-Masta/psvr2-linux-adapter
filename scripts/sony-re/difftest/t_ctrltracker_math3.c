#include "sony.h"
#include "ct_difftest.h"
#include "psvr2_ct_math.h"

typedef void *(*f_pp)(void *, const void *);
typedef void *(*f_ppp)(void *, const void *, const void *);
typedef void *(*f_pf)(void *, float);
typedef float *(*f_cap)(float *, const float *, float, float);
typedef double *(*f_capd)(double *, const float *, double, double);
typedef float (*f_az)(float *, float *, const float *, const float *);

static double rd(void)
{
	return (rnd() & 7) == 0 ? (double)rf() : ((int32_t)rnd()) * (1.0 / 2147483648.0) * 3.0;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	const int n = 200000;
	int fail = 0, bad;

	bad = 0;
	for (int i = 0; i < n; i++) {
		float a[4], b[4], x[4], y[4], s = rf();
		for (int k = 0; k < 4; k++) { a[k] = rf(); b[k] = rf(); }
		memcpy(x, a, 16); memcpy(y, a, 16);
		((f_pp)SONY(0x18006af00))(x, b);
		psvr2_ct_vec3_sub_inplace(y, b);
		bad += !feqn(x, y, 3);
		memcpy(x, a, 16); memcpy(y, a, 16);
		((f_pf)SONY(0x18006af50))(x, s);
		psvr2_ct_vec3_scale_inplace(y, s);
		bad += !feqn(x, y, 3);
		/* from_to: also near-unit inputs */
		if (i & 1) {
			float la = psvr2_ct_vec3_length(a), lb = psvr2_ct_vec3_length(b);
			for (int k = 0; k < 3; k++) { a[k] /= la; b[k] /= lb; }
		}
		((f_ppp)SONY(0x18006b030))(x, a, b);
		psvr2_ct_quat_from_to(y, a, b);
		bad += !feqn(x, y, 4);
		float sa, sb, oa, ob;
		const float r1 = ((f_az)SONY(0x18006b700))(&sa, &sb, a, b);
		const float r2 = psvr2_ct_azimuth_diff(&oa, &ob, a, b);
		bad += !feq(r1, r2) || !feq(sa, oa) || !feq(sb, ob);
	}
	fail |= report("vec3 sub/scale, quat_from_to, az 0x18006b700", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		double p[4], q[4], x[4], y[4];
		for (int k = 0; k < 4; k++) { p[k] = rd(); q[k] = rd(); }
		((f_pp)SONY(0x18006b250))(x, p);
		psvr2_ct_quatd_normalize(y, p);
		for (int k = 0; k < 4; k++) bad += !deq(x[k], y[k]);
		((f_ppp)SONY(0x18006b310))(p, x, q);
		psvr2_ct_quatd_mul(y, p, q);
		for (int k = 0; k < 4; k++) bad += !deq(x[k], y[k]);
	}
	fail |= report("quatd normalize/mul 0x18006b250/310", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float v[3], x[4], y[4];
		for (int k = 0; k < 3; k++) v[k] = rf();
		const float c = (i & 1) ? rf() : ((int32_t)rnd()) * (1.0f / 2147483648.0f);
		const float r = rf();
		((f_cap)SONY(0x18006b4a0))(x, v, c, r);
		psvr2_ct_quat_cap(y, v, c, r);
		bad += !feqn(x, y, 4);
		double X[4], Y[4];
		((f_capd)SONY(0x18006b5c0))(X, v, (double)c, (double)r);
		psvr2_ct_quat_cap_d(Y, v, (double)c, (double)r);
		for (int k = 0; k < 4; k++) bad += !deq(X[k], Y[k]);
	}
	fail |= report("quat_cap f/d 0x18006b4a0/5c0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float m[3][4], x[4], y[4];
		if (i & 1) {
			float q[4];
			for (int k = 0; k < 4; k++) q[k] = ((int32_t)rnd()) * (1.0f / 2147483648.0f);
			psvr2_ct_quat_normalize(q, q);
			psvr2_ct_quat_to_mat3(m, q);
		} else {
			for (int k = 0; k < 12; k++) m[k / 4][k % 4] = rf();
		}
		((f_pp)SONY(0x180061d40))(x, m);
		psvr2_ct_mat3_to_quat(y, m);
		bad += !feqn(x, y, 4);
	}
	fail |= report("mat3_to_quat 0x180061d40", bad, n);
	return fail;
}
