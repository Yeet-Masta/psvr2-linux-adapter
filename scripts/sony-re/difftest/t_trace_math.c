// SPDX-License-Identifier: GPL-2.0
/* Differential test: psvr2_math_* against Sony's math module. */
#include "sony.h"
#include "trace_difftest.h"
#include "psvr2_math.h"

typedef void *(*f_pp)(void *, void *);
typedef void *(*f_ppp)(void *, void *, void *);
typedef void (*f_e)(float *, float, float, float);
typedef void *(*f_fr)(float *, float, float, float, float, float, float);

static float rq(void) { return (rnd() & 7) ? rfs(2.0f) : rf(); }

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	int fail = 0, n = 200000;
	float a[16], b[16], o1[16], o2[16], p[10];

	int bad = 0;
	for (int i = 0; i < n; i++) {
		rfill(a, 9); rfill(b, 9);
		((f_ppp)SONY(0x18016baf0))(a, o1, b);
		psvr2_math_mat3_mul(a, b, o2);
		bad += !feqn(o1, o2, 9);
	}
	fail |= report("mat3_mul 0x18016baf0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		rfill(a, 9); rfill(b, 3);
		((f_ppp)SONY(0x18016bc90))(a, o1, b);
		psvr2_math_mat3_mul_vec3(a, b, o2);
		bad += !feqn(o1, o2, 3);
	}
	fail |= report("mat3_mul_vec3 0x18016bc90", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		rfill(a, 16); rfill(b, 16);
		((f_ppp)SONY(0x18016bd20))(a, o1, b);
		psvr2_math_mat4_mul(a, b, o2);
		bad += !feqn(o1, o2, 16);
	}
	fail |= report("mat4_mul 0x18016bd20", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		rfill(a, 16); rfill(b, 16);
		memcpy(o1, a, 64);
		((f_pp)SONY(0x18016c810))(o1, b);
		memcpy(o2, a, 64);
		psvr2_math_mat4_mul_inplace(o2, b);
		bad += !feqn(o1, o2, 16);
	}
	fail |= report("mat4_mul_inplace 0x18016c810", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		for (int k = 0; k < 4; k++) a[k] = rq();
		((f_pp)SONY(0x18016df80))(a, o1);
		psvr2_math_quat_to_mat4(a, o2);
		bad += !feqn(o1, o2, 16);
	}
	fail |= report("quat_to_mat4 0x18016df80", bad, n);

	bad = 0;
	int bad2 = 0;
	for (int i = 0; i < n; i++) {
		for (int k = 0; k < 10; k++) p[k] = rq();
		((f_pp)SONY(0x18016dc50))(p, o1);
		psvr2_math_pose_to_mat4(p, o2);
		bad += !feqn(o1, o2, 16);
		((f_pp)SONY(0x18016dda0))(p, o1);
		psvr2_math_pose_to_mat4_inv(p, o2);
		bad2 += !feqn(o1, o2, 16);
	}
	fail |= report("pose_to_mat4 0x18016dc50", bad, n);
	fail |= report("pose_to_mat4_inv 0x18016dda0", bad2, n);

	bad = 0;
	for (int k = 0; k < 10; k++) o1[k] = o2[k] = 7.0f;
	((void (*)(float *))SONY(0x18016ebe0))(o1);
	psvr2_math_pose_identity(o2);
	bad += !feqn(o1, o2, 10);
	fail |= report("pose_identity 0x18016ebe0", bad, 1);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float x = rq() * 4, y = rq() * 4, z = rq() * 4;
		((f_e)SONY(0x18016ec10))(o1, x, y, z);
		psvr2_math_euler_to_quat(o2, x, y, z);
		bad += !feqn(o1, o2, 4);
	}
	fail |= report("euler_to_quat 0x18016ec10", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float v[6];
		for (int k = 0; k < 6; k++) v[k] = rq();
		for (int k = 0; k < 16; k++) o1[k] = o2[k] = 9.0f;
		((f_fr)SONY(0x18016ed80))(o1, v[0], v[1], v[2], v[3], v[4], v[5]);
		psvr2_math_frustum(o2, v[0], v[1], v[2], v[3], v[4], v[5]);
		bad += !feqn(o1, o2, 16);
	}
	fail |= report("frustum 0x18016ed80", bad, n);

	/* ---- batch 2 ---- */
#define LOOP for (int i = 0; i < n; i++)
	bad = 0;
	LOOP { rfill(a, 16); rfill(b, 4);
		((f_ppp)SONY(0x18016c130))(a, o1, b); psvr2_math_mat4_mul_vec4(a, b, o2);
		bad += !feqn(o1, o2, 4); }
	fail |= report("mat4_mul_vec4 0x18016c130", bad, n);
	bad = 0;
	LOOP { rfill(a, 9); rfill(b, 9); memcpy(o1, a, 36); memcpy(o2, a, 36);
		((f_pp)SONY(0x18016c630))(o1, b); psvr2_math_mat3_mul_inplace(o2, b);
		bad += !feqn(o1, o2, 9); }
	fail |= report("mat3_mul_inplace 0x18016c630", bad, n);
	bad = 0; bad2 = 0;
	LOOP { for (int k = 0; k < 4; k++) { a[k] = rq(); b[k] = rq(); }
		((f_ppp)SONY(0x18016c220))(a, o1, b); psvr2_math_quat_mul(a, b, o2);
		bad += !feqn(o1, o2, 4);
		memcpy(o1, a, 16); memcpy(o2, a, 16);
		((f_pp)SONY(0x18016cc80))(o1, b); psvr2_math_quat_mul_inplace(o2, b);
		bad2 += !feqn(o1, o2, 4); }
	fail |= report("quat_mul 0x18016c220", bad, n);
	fail |= report("quat_mul_inplace 0x18016cc80", bad2, n);
	bad = 0;
	LOOP { for (int k = 0; k < 4; k++) a[k] = rq(); for (int k = 0; k < 3; k++) b[k] = rq();
		((f_ppp)SONY(0x18016c360))(a, o1, b); psvr2_math_quat_rotate(a, b, o2);
		bad += !feqn(o1, o2, 3); }
	fail |= report("quat_rotate 0x18016c360", bad, n);
	bad = 0;
	LOOP { for (int k = 0; k < 10; k++) { a[k] = rq(); b[k] = rq(); }
		((f_ppp)SONY(0x18016c420))(a, o1, b); psvr2_math_pose_mul(a, b, o2);
		bad += !feqn(o1, o2, 10); }
	fail |= report("pose_mul 0x18016c420", bad, n);
	bad = 0;
	LOOP { for (int k = 0; k < 3; k++) o1[k] = o2[k] = (i & 3) ? rq() : rf() * 1e-6f;
		((void (*)(float *))SONY(0x180170840))(o1); psvr2_math_vec3_normalize(o2);
		bad += !feqn(o1, o2, 3); }
	fail |= report("vec3_normalize 0x180170840", bad, n);
	bad = 0;
	LOOP { for (int k = 0; k < 2; k++) { a[k] = rq(); b[k] = rq(); }
		float r1 = ((float (*)(float *, float *))SONY(0x18016d170))(a, b);
		float r2 = psvr2_math_vec2_angle_deg(a, b);
		bad += !feq(r1, r2); }
	fail |= report("vec2_angle_deg 0x18016d170", bad, n);
	bad = 0; bad2 = 0;
	LOOP { float ang = rq() * 4; for (int k = 0; k < 3; k++) b[k] = rq();
		((void (*)(float *, float, float *))SONY(0x18016d270))(o1, ang, b);
		psvr2_math_axis_angle_to_quat(o2, ang, b);
		bad += !feqn(o1, o2, 4);
		((void (*)(float *, float, float *))SONY(0x18016d9a0))(o1, ang, b);
		psvr2_math_axis_angle_to_mat4(o2, ang, b);
		bad2 += !feqn(o1, o2, 16); }
	fail |= report("axis_angle_to_quat 0x18016d270", bad, n);
	fail |= report("axis_angle_to_mat4 0x18016d9a0", bad2, n);
	bad = 0;
	LOOP { for (int k = 0; k < 4; k++) a[k] = rq();
		((f_pp)SONY(0x18016da40))(o1, a); psvr2_math_quat_to_mat4(a, o2);
		bad += !feqn(o1, o2, 16); }
	fail |= report("quat_to_mat4 (swapped) 0x18016da40", bad, n);
	bad = 0;
	LOOP { for (int k = 0; k < 10; k++) p[k] = rq();
		((void *(*)(float *, float *, float *, float *))SONY(0x18016db00))(o1, p + 7, p + 3, p);
		psvr2_math_trs_to_mat4(o2, p + 7, p + 3, p);
		bad += !feqn(o1, o2, 16); }
	fail |= report("trs_to_mat4 0x18016db00", bad, n);
	bad = 0;
	LOOP { float v[9]; for (int k = 0; k < 9; k++) v[k] = (i & 15) ? rq() : 0.0f;
		((void *(*)(float *, float, float, float, float, float, float, float, float, float))SONY(0x18016d430))
			(o1, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
		psvr2_math_look_at(o2, v, v + 3, v + 6);
		bad += !feqn(o1, o2, 16); }
	fail |= report("look_at 0x18016d430", bad, n);
	bad = 0;
	LOOP { float v[4]; for (int k = 0; k < 4; k++) v[k] = rq() * 50;
		((void *(*)(float *, float, float, float, float))SONY(0x18016d800))(o1, v[0], v[1], v[2], v[3]);
		psvr2_math_perspective(o2, v[0], v[1], v[2], v[3]);
		bad += !feqn(o1, o2, 16); }
	fail |= report("perspective 0x18016d800", bad, n);
	bad = 0;
	LOOP { float v[6]; for (int k = 0; k < 6; k++) v[k] = rq();
		((f_fr)SONY(0x18016d370))(o1, v[0], v[1], v[2], v[3], v[4], v[5]);
		psvr2_math_frustum(o2, v[0], v[1], v[2], v[3], v[4], v[5]);
		bad += !feqn(o1, o2, 16); }
	fail |= report("frustum (dup) 0x18016d370", bad, n);

	/* ---- batch 3 ---- */
	bad = 0; bad2 = 0;
	LOOP { float q[4];
		if (i & 1) { for (int k = 0; k < 4; k++) q[k] = rq(); psvr2_math_quat_to_mat4(q, a);
			if (i & 2) a[(rnd() % 3) * 4 + rnd() % 3] += rfs(0.01f); }
		else rfill(a, 16);
		float m3[9]; for (int c = 0; c < 3; c++) for (int r = 0; r < 3; r++) m3[c * 3 + r] = a[c * 4 + r];
		((f_pp)SONY(0x18016e150))(m3, o1); psvr2_math_mat3_to_quat(m3, o2);
		bad += !feqn(o1, o2, 4);
		((f_pp)SONY(0x18016e3c0))(a, o1); psvr2_math_mat4_to_quat(a, o2);
		bad2 += !feqn(o1, o2, 4); }
	fail |= report("mat3_to_quat 0x18016e150", bad, n);
	fail |= report("mat4_to_quat 0x18016e3c0", bad2, n);
	bad = 0; bad2 = 0;
	LOOP { for (int k = 0; k < 10; k++) p[k] = rq();
		if (i & 1) psvr2_math_pose_to_mat4(p, a); else rfill(a, 16);
		if ((i & 7) == 2) a[(rnd() % 3) * 4] = 0.0f, a[(rnd() % 3) * 4 + 1] = 0.0f, a[(rnd() % 3) * 4 + 2] = 0.0f;
		float s1[3], q1[4], t1[3], s2[3], q2[4], t2[3];
		int r1 = (int)((uint64_t (*)(float *, float *, float *, float *))SONY(0x18016e630))(a, s1, q1, t1) & 0xff;
		int r2 = psvr2_math_mat4_decompose(a, s2, q2, t2);
		bad += r1 != r2 || !feqn(s1, s2, 3) || !feqn(q1, q2, 4) || !feqn(t1, t2, 3);
		for (int k = 0; k < 10; k++) o1[k] = o2[k] = 5.0f;
		r1 = (int)((uint64_t (*)(float *, float *))SONY(0x18016e910))(o1, a);
		r2 = psvr2_math_mat4_to_pose(o2, a);
		bad2 += r1 != r2 || !feqn(o1, o2, 10); }
	fail |= report("mat4_decompose 0x18016e630", bad, n);
	fail |= report("mat4_to_pose 0x18016e910", bad2, n);
	bad = 0;
	LOOP { float v[9]; for (int k = 0; k < 9; k++) v[k] = (i & 15) ? rq() : 0.0f;
		((void *(*)(float *, float, float, float, float, float, float, float, float, float))SONY(0x18016ee40))
			(o1, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
		psvr2_math_look_at(o2, v, v + 3, v + 6);
		bad += !feqn(o1, o2, 16); }
	fail |= report("look_at (dup) 0x18016ee40", bad, n);
	bad = 0;
	LOOP { float v[4]; for (int k = 0; k < 4; k++) v[k] = rq() * 50;
		((void *(*)(float *, float, float, float, float))SONY(0x18016f210))(o1, v[0], v[1], v[2], v[3]);
		psvr2_math_perspective(o2, v[0], v[1], v[2], v[3]);
		bad += !feqn(o1, o2, 16); }
	fail |= report("perspective (dup) 0x18016f210", bad, n);
	bad = 0;
	LOOP { float ang = rq() * 4; for (int k = 0; k < 3; k++) b[k] = rq();
		((void (*)(float *, float *, float))SONY(0x18016f2d0))(o1, b, ang);
		psvr2_math_axis_angle_to_quat(o2, ang, b);
		bad += !feqn(o1, o2, 4); }
	fail |= report("axis_angle_to_quat (dup) 0x18016f2d0", bad, n);
	{
		static const uint64_t va[3] = { 0x18016f350, 0x18016f3e0, 0x18016f470 };
		void (*ours[3])(float *, float) = { psvr2_math_mat4_rot_x_deg, psvr2_math_mat4_rot_y_deg, psvr2_math_mat4_rot_z_deg };
		for (int w = 0; w < 3; w++) {
			bad = 0;
			LOOP { float d = rq() * 400; for (int k = 0; k < 16; k++) o1[k] = o2[k] = 3.0f;
				((void *(*)(float *, float))SONY(va[w]))(o1, d); ours[w](o2, d);
				bad += !feqn(o1, o2, 16); }
			char nm[64]; snprintf(nm, sizeof nm, "mat4_rot_%c_deg 0x%llx", 'x' + w, (unsigned long long)va[w]);
			fail |= report(nm, bad, n);
		}
	}
	bad = 0;
	LOOP { float v[3]; for (int k = 0; k < 3; k++) v[k] = rq(); for (int k = 0; k < 16; k++) o1[k] = o2[k] = 3.0f;
		((void *(*)(float *, float, float, float))SONY(0x18016f540))(o1, v[0], v[1], v[2]);
		psvr2_math_mat4_translation(o2, v[0], v[1], v[2]);
		bad += !feqn(o1, o2, 16); }
	fail |= report("mat4_translation 0x18016f540", bad, n);

	bad = 0;
	LOOP { for (int k = 0; k < 16; k++) o1[k] = o2[k] = (i % 101) ? rfs(4.0f) : (float)(rnd() % 3);
		((float *(*)(float *))SONY(0x18016f590))(o1);
		psvr2_math_mat4_invert(o2);
		bad += !feqn(o1, o2, 16); }
	fail |= report("mat4_invert 0x18016f590", bad, n);

	bad = 0;
	LOOP { double r[3], R1[12], R2[9];
		for (int k = 0; k < 3; k++) r[k] = (i & 7) ? (double)rfs(4.0f) + rfs(1e-4f) : (double)rf() * 1e-17;
		for (int k = 0; k < 12; k++) R1[k] = 7.0;
		((uint64_t (*)(double *, double *))SONY(0x180168e40))(r, R1);
		psvr2_math_rodrigues_to_mat3d(r, R2);
		for (int k = 0; k < 9; k++) if (!deq(R1[k], R2[k])) { if (bad++ < 6) printf("r=%a %a %a k=%d sony=%a ours=%a\n", r[0], r[1], r[2], k, R1[k], R2[k]); } }
	fail |= report("rodrigues_to_mat3d 0x180168e40 (elements)", bad, 9 * n);
	return fail;
}
