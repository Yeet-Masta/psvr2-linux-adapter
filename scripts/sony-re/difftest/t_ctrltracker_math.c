#include "sony.h"
#include "ct_difftest.h"
#include "psvr2_ct_math.h"

typedef float *(*f_vqv)(float *, const float *, const float *);
typedef float *(*f_oi)(float *, const float *);
typedef float *(*f_mov)(const float *, float *, const float *);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	const int n = 200000;
	int fail = 0, bad;

	bad = 0;
	for (int i = 0; i < n; i++) {
		float q[4], v[3], a[3], b[3];
		for (int k = 0; k < 4; k++) q[k] = rf();
		for (int k = 0; k < 3; k++) v[k] = rf();
		((f_vqv)SONY(0x180058720))(a, q, v);
		psvr2_ct_quat_rotate(b, q, v);
		bad += !feqn(a, b, 3);
	}
	fail |= report("quat_rotate 0x180058720", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float q[4], a[4], b[4];
		for (int k = 0; k < 4; k++) q[k] = rf();
		((f_oi)SONY(0x180058900))(a, q);
		b[0] = psvr2_ct_quat_length(q);
		bad += !feq(a[0], b[0]);
		((f_oi)SONY(0x180058970))(a, q);
		psvr2_ct_quat_normalize(b, q);
		bad += !feqn(a, b, 4);
	}
	fail |= report("quat_length/normalize 0x180058900/970", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float q[4], a[3][4], b[3][4];
		for (int k = 0; k < 4; k++) q[k] = rf();
		((f_oi)SONY(0x180058a10))(&a[0][0], q);
		psvr2_ct_quat_to_mat3(b, q);
		for (int c = 0; c < 3; c++)
			bad += !feqn(a[c], b[c], 3);
	}
	fail |= report("quat_to_mat3 0x180058a10", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float m[4][4], v[4], a[4], b[4];
		for (int k = 0; k < 16; k++) m[k / 4][k % 4] = rf();
		for (int k = 0; k < 4; k++) v[k] = rf();
		((f_mov)SONY(0x180058e40))(&m[0][0], a, v);
		psvr2_ct_mat4_mul_vec4(b, m, v);
		bad += !feqn(a, b, 4);
		((f_mov)SONY(0x180058f30))(&m[0][0], a, v);
		psvr2_ct_mat43_mul_vec3(b, (const float (*)[4])m, v);
		bad += !feqn(a, b, 4);
	}
	fail |= report("mat4*vec4 / mat43*vec3 0x180058e40/f30", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float m[4][4], a[4][4], b[4][4];
		for (int k = 0; k < 16; k++) m[k / 4][k % 4] = rf();
		((f_oi)SONY(0x180059000))(&a[0][0], &m[0][0]);
		psvr2_ct_mat4_inverse(b, m);
		bad += !feqn(&a[0][0], &b[0][0], 16);
	}
	fail |= report("mat4_inverse 0x180059000", bad, n);
	return fail;
}
