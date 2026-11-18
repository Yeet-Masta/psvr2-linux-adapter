#include "sony.h"
#include "irt_difftest.h"
#include "psvr2_ins_math.h"

#define N 200000
typedef float *(*f_p)(float *);
typedef float (*f_len)(const float *);
typedef void (*f_v)(float *);

/* In-place unary op on n floats. */
static int t_unary(const char *name, uint64_t va, void *ours, int n, int kind)
{
	int bad = 0;
	for (int i = 0; i < N; i++) {
		float a[16], b[16];
		rfill(a, 16);
		memcpy(b, a, sizeof a);
		if (kind == 0) { ((f_p)SONY(va))(a); ((f_p)ours)(b); }
		else { ((f_v)SONY(va))(a); ((f_v)ours)(b); }
		if (!feqn(a, b, n) && bad++ < 3)
			printf("  %s i=%d %g/%g %g/%g\n", name, i, a[0], b[0], a[1], b[1]);
	}
	return report(name, bad, N);
}

static int t_len(const char *name, uint64_t va, f_len ours)
{
	int bad = 0;
	for (int i = 0; i < N; i++) {
		float a[4]; rfill(a, 4);
		float x = ((f_len)SONY(va))(a), y = ours(a);
		if (!feq(x, y)) bad++;
	}
	return report(name, bad, N);
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	int st = 0;
	/* The conj stub reads the quaternion at +0 of its object. */
	st |= t_unary("quat_conj_inplace 0x180170120", 0x180170120, psvr2_irtracker_quat_conj_inplace, 4, 0);
	st |= t_unary("pose_negate 0x180170160", 0x180170160, psvr2_irtracker_pose_negate, 7, 0);
	st |= t_len("vec2_len 0x180170640", 0x180170640, psvr2_irtracker_vec2_len);
	st |= t_len("vec3_len 0x180170670", 0x180170670, psvr2_irtracker_vec3_len);
	st |= t_unary("quat_normalize 0x180170700", 0x180170700, psvr2_irtracker_quat_normalize, 4, 1);
	st |= t_unary("vec2_normalize 0x1801707c0", 0x1801707c0, psvr2_irtracker_vec2_normalize, 2, 1);
	st |= t_unary("vec3_normalize 0x180170840", 0x180170840, psvr2_irtracker_vec3_normalize, 3, 1);
	st |= t_unary("mat4_identity 0x180170cb0", 0x180170cb0, psvr2_irtracker_mat4_identity, 16, 0);
	st |= t_unary("mat4_transpose 0x180171010", 0x180171010, psvr2_irtracker_mat4_transpose, 16, 0);
	{
		typedef float *(*f)(float *, const float *, const float *, float);
		int bad = 0;
		for (int i = 0; i < N; i++) {
			float a[3], b[3], o1[3], o2[3], t = rf();
			rfill(a, 3); rfill(b, 3);
			((f)SONY(0x1801701c0))(o1, a, b, t); psvr2_irtracker_vec3_lerp(o2, a, b, t);
			if (!feqn(o1, o2, 3)) bad++;
		}
		st |= report("vec3_lerp 0x1801701c0", bad, N);
	}
	{
		typedef float *(*f)(const float *, float *, float, const float *);
		int bad = 0, worst = 0;
		for (int i = 0; i < N; i++) {
			float obj[4], v[3], o1[3], o2[3], ang = rfs(8.0f);
			for (int k = 0; k < 4; k++) obj[k] = rfs(2.0f);
			for (int k = 0; k < 3; k++) v[k] = rfs(10.0f);
			if (i & 1) { ang = rf(); obj[1] = rf(); v[0] = rf(); }
			((f)SONY(0x180170a10))(obj, o1, ang, v); psvr2_irtracker_axis_angle_rotate(obj, o2, ang, v);
			if (!feqn(o1, o2, 3)) { bad++; int u = fulpn(o1, o2, 3); if (u > worst) worst = u; }
		}
		printf("  (worst %d ulp)\n", worst);
		st |= report("axis_angle_rotate 0x180170a10", bad, N);
	}
	{
		typedef float *(*f)(const float *, float *, float, float *);
		int bad = 0, worst = 0;
		for (int i = 0; i < N; i++) {
			float a[4], b1[4], b2[4], o1[4], o2[4], t = (i & 3) ? rfs(1.0f) + 0.5f : rf();
			for (int k = 0; k < 4; k++) { a[k] = rfs(1.0f); b1[k] = rfs(1.0f); }
			if ((i & 7) == 0) { b1[0] = a[0]; b1[1] = a[1] + 1e-3f; }
			if ((i & 15) == 1) rfill(b1, 4);
			memcpy(b2, b1, sizeof b1);
			((f)SONY(0x180170dc0))(a, o1, t, b1); psvr2_irtracker_quat_slerp(a, o2, t, b2);
			if (!feqn(o1, o2, 4) || !feqn(b1, b2, 4)) { if (bad < 4) printf("  a %a %a %a %a b %a %a %a %a t %a o %a/%a %a/%a\n", a[0], a[1], a[2], a[3], b2[0], b2[1], b2[2], b2[3], t, o1[0], o2[0], o1[1], o2[1]); bad++; int u = fulpn(o1, o2, 4); if (u > worst) worst = u; }
		}
		printf("  (worst %d ulp)\n", worst);
		st |= report("quat_slerp 0x180170dc0", bad, N);
	}
	{
		typedef uint64_t (*f)(float **, float *, const float *, float, float, const float *, float, float);
		int bad = 0;
		for (int i = 0; i < N; i++) {
			float ma[3], mb[3], m1[3] = { 7, 7, 7 }, m2[3] = { 7, 7, 7 }, s1 = 9, s2 = 9;
			float *p1 = m1, *p2 = m2;
			for (int k = 0; k < 3; k++) { ma[k] = rfs(3.0f); mb[k] = rfs(3.0f); }
			float sa = rfs(1.0f), sb = rfs(1.0f), wa = rfs(2.0f) + 2.0f, wb = rfs(2.0f) + 2.0f;
			if (i & 1) { ma[0] = rf(); sa = rf(); wb = rf(); }
			int r1 = (int)(((f)SONY(0x1801711f0))(&p1, &s1, ma, sa, wa, mb, sb, wb) & 0xff);
			int r2 = psvr2_irtracker_gauss_merge(&p2, &s2, ma, sa, wa, mb, sb, wb);
			if (r1 != r2 || !feqn(m1, m2, 3) || !feq(s1, s2)) bad++;
		}
		st |= report("gauss_merge 0x1801711f0", bad, N);
	}
	return st;
}
