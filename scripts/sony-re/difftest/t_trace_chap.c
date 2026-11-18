// SPDX-License-Identifier: GPL-2.0
/*
 * Universe / chaperone helpers against Sony: pose compose 0x18011e190,
 * quaternion -> 3x3 0x180121360, yaw 0x18011f420 and the default play
 * area 0x18011ef90. For the last one Sony's headset-pose fetch
 * (0x18011f1b0, which reads shared memory) is patched with a jump to a
 * stub that returns the test pose.
 */
#include <math.h>
#include "sony.h"
#include "trace_difftest.h"
#include "psvr2_chaperone.h"

static float g_pose[7];

static float *stub_pose(float *out)
{
	memcpy(out, g_pose, sizeof(g_pose));
	return out;
}

static void patch_jump(void *at, void *to)
{
	uint8_t code[12] = { 0x48, 0xb8 }; /* mov rax, imm64; jmp rax */
	DWORD old;

	memcpy(code + 2, &to, 8);
	code[10] = 0xff;
	code[11] = 0xe0;
	VirtualProtect(at, sizeof(code), PAGE_EXECUTE_READWRITE, &old);
	memcpy(at, code, sizeof(code));
	VirtualProtect(at, sizeof(code), old, &old);
	FlushInstructionCache(GetCurrentProcess(), at, sizeof(code));
}

static void quiet_log(void) { }

static void rand_quat(float *q, int i)
{
	for (int k = 0; k < 4; k++)
		q[k] = (i % 7) ? rfs(1.0f) : rf();
	if (i % 3 == 0) { /* exact axis-aligned cases hit the branch edges */
		for (int k = 0; k < 4; k++)
			q[k] = (float)((int)(rnd() % 3) - 1) * ((rnd() & 1) ? 0.70710677f : 1.0f);
	}
	if (i % 5 == 1) {
		float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		if (n > 0) for (int k = 0; k < 4; k++) q[k] /= n;
	}
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	int fail = 0, n = 200000, bad;

	/* Sony's VrTracker2 logger (0x1801aa6d0) blocks without its log thread. */
	patch_jump(SONY(0x1801aa6d0), (void *)quiet_log);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float a[7], b[7], o1[7], o2[7];
		for (int k = 0; k < 3; k++) { a[k] = rfs(10.0f); b[k] = (i % 9) ? rfs(10.0f) : rf(); }
		rand_quat(a + 3, i);
		rand_quat(b + 3, i + 1);
		((float *(*)(const float *, float *, const float *))SONY(0x18011e190))(a, o1, b);
		psvr2_chap_pose_mul(a, b, o2);
		bad += !feqn(o1, o2, 7);
	}
	fail |= report("chap pose_mul 0x18011e190", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float a[7], o1[7], o2[7];
		for (int k = 0; k < 3; k++) a[k] = (i % 9) ? rfs(10.0f) : rf();
		rand_quat(a + 3, i);
		((void (*)(const float *, float *))SONY(0x1801613a0))(a, o1);
		psvr2_chap_pose_inv(a, o2);
		bad += !feqn(o1, o2, 7);
	}
	fail |= report("chap pose_inv 0x1801613a0", bad, n);

	bad = 0;
	for (int i = 0; i < n; i++) {
		float q[4], R1[9], R2[9];
		rand_quat(q, i);
		((float *(*)(const float *, float *))SONY(0x180121360))(q, R1);
		psvr2_chap_quat_to_mat3(q, R2);
		bad += !feqn(R1, R2, 9);
	}
	fail |= report("chap quat_to_mat3 0x180121360", bad, n);

	bad = 0;
	int ny = 200000;
	for (int i = 0; i < ny; i++) {
		float q[4] __attribute__((aligned(16)));
		rand_quat(q, i);
		float y1 = ((float (*)(const float *))SONY(0x18011f420))(q);
		float y2 = psvr2_chap_yaw(q);
		if (!feq(y1, y2) && bad++ < 5)
			printf("yaw q=%a %a %a %a sony %a ours %a\n", q[0], q[1], q[2], q[3], y1, y2);
	}
	fail |= report("chap yaw 0x18011f420", bad, ny);

	patch_jump(SONY(0x18011f1b0), (void *)stub_pose);
	bad = 0;
	int na = 5000;
	for (int i = 0; i < na; i++) {
		static struct psvr2_chap_area s1, s2;
		for (int k = 0; k < 3; k++) g_pose[k] = rfs(3.0f);
		rand_quat(g_pose + 3, i);
		memset(&s1, 0x5a, sizeof(s1));
		memset(&s2, 0x5a, sizeof(s2));
		((void (*)(void *, void *))SONY(0x18011ef90))(NULL, &s1);
		psvr2_chap_default_area(g_pose, &s2);
		if (memcmp(&s1, &s2, sizeof(s1)) && bad++ < 5) {
			const uint8_t *p1 = (const uint8_t *)&s1, *p2 = (const uint8_t *)&s2;
			size_t d = 0;
			while (p1[d] == p2[d]) d++;
			printf("area diff at 0x%zx q=%a %a %a %a sony seat %a stand %a ours %a %a\n", d, g_pose[3], g_pose[4], g_pose[5], g_pose[6], s1.seated[3], s1.standing[3], s2.seated[3], s2.standing[3]);
		}
	}
	fail |= report("chap default_area 0x18011ef90", bad, na);
	return fail;
}
