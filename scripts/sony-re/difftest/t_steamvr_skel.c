/* Hand skeleton bind pose (0x18012b6a0). */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"
#include <math.h>

static float rf(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() & 0xffffff) / 16777216.0f; }

/* Positions are copies and must match exactly. Quaternion parts come from
 * sinf/cosf, which may differ from Sony's CRT by 1 ulp; there we allow an
 * absolute error, counted in units of 2^-24 (half an ulp of 1.0). */
static double maxerr;
static int cmp(const psvr2_bone *a, const psvr2_bone *b)
{
	int bad = 0;
	for (int i = 0; i < PSVR2_SKEL_BONES; i++) {
		if (memcmp(a[i].pos, b[i].pos, sizeof a[i].pos) || memcmp(a[i].scale, b[i].scale, sizeof a[i].scale))
			bad = 1;
		for (int k = 0; k < 4; k++) {
			double e = fabs((double)a[i].q[k] - (double)b[i].q[k]) * 16777216.0;
			if (e > maxerr)
				maxerr = e;
			if (e > 8) {
				if (getenv("SKDBG"))
					printf("  bone %d q%d sony %a ours %a\n", i, k, a[i].q[k], b[i].q[k]);
				bad = 1;
			}
		}
	}
	return bad;
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	void (*bind)(int, void *, const float *) = (void (*)(int, void *, const float *))SONY(0x18012b6a0);
	static float tab[2 * PSVR2_SKEL_BONES * 6];
	psvr2_bone s[PSVR2_SKEL_BONES], o[PSVR2_SKEL_BONES];
	int bad = 0, n = 0;

	for (int it = 0; it < 20004; it++) {
		const float *t = tab;
		const int side = it & 1;
		if (it < 2)
			t = (const float *)SONY(0x18034d2b0);
		else if (it < 4)
			t = (const float *)SONY(0x18034d880);
		else
			for (int k = 0; k < 2 * PSVR2_SKEL_BONES * 6; k++)
				tab[k] = (k % 6) < 3 ? rf(-0.3f, 0.3f) : rf(-400, 400);
		memset(s, 0, sizeof s);
		memset(o, 0, sizeof o);
		bind(side, s, t);
		psvr2_steamvr_skel_bind(side, t, o);
		n++;
		if (it < 4 && memcmp(s, o, sizeof s)) {
			printf("skel bind: Sony table %d not bit-exact\n", it);
			bad++;
		}
		if (cmp(s, o)) {
			if (bad++ < 5)
				printf("skel bind it=%d side=%d mismatch\n", it, side);
		}
	}
	printf("skel bind: %d/%d mismatches (max error %.1f x 2^-24 on quaternions)\n", bad, n, maxerr);
	int total = bad;

	/* Open-hand finger curl (0x1801296d0) on a zeroed skeleton object. */
	void (*curl)(void *, const void *) = (void (*)(void *, const void *))SONY(0x1801296d0);
	static uint8_t obj[0x1510];
	psvr2_skel_input in;
	bad = 0;
	maxerr = 0;
	n = 200000;
	for (int it = 0; it < n; it++) {
		const int side = rnd() & 1;
		memset(obj, 0, sizeof obj);
		*(int32_t *)(obj + 8) = side ? (int32_t)(1 + rnd() % 3) : 0;
		for (int k = 0; k < (int)sizeof in; k++)
			((uint8_t *)&in)[k] = (uint8_t)rnd();
		if (it < 512) {	/* edges: 0 and 255 values */
			uint8_t *b = (uint8_t *)&in;
			b[8 + side] = b[0x10 + side] = b[0x12 + side] = (it & 1) ? 255 : 0;
		}
		for (int k = 0; k < PSVR2_SKEL_BONES * 10; k++)
			((float *)(obj + 0xe94))[k] = rf(-1, 1);
		memcpy(o, obj + 0xe94, sizeof o);
		curl(obj, &in);
		psvr2_steamvr_skel_open_curl(*(int32_t *)(obj + 8), &in, o);
		memcpy(s, obj + 0xe94, sizeof s);
		if (cmp(s, o)) {
			if (bad++ < 5)
				printf("skel open curl it=%d side=%d mismatch\n", it, side);
		}
	}
	printf("skel open curl: %d/%d mismatches (max error %.1f x 2^-24 on quaternions)\n", bad, n, maxerr);
	total += bad;

	/* Open-hand thumb (0x18012a510). */
	void (*thumb)(void *, const void *) = (void (*)(void *, const void *))SONY(0x18012a510);
	bad = 0;
	maxerr = 0;
	for (int it = 0; it < n; it++) {
		const int side = rnd() & 1;
		memset(obj, 0, sizeof obj);
		*(int32_t *)(obj + 8) = side ? (int32_t)(1 + rnd() % 3) : 0;
		for (int k = 0; k < (int)sizeof in; k++)
			((uint8_t *)&in)[k] = (uint8_t)rnd();
		/* make each branch likely: clear most of the touch/button bits */
		in.touch &= rnd() | rnd();
		in.buttons &= rnd() | rnd();
		if (it & 2)
			in.touch &= ~0xf006u;
		obj[0x1508] = (uint8_t)(rnd() & 1);
		for (int k = 0; k < PSVR2_SKEL_BONES * 10; k++)
			((float *)(obj + 0xe94))[k] = rf(-1, 1);
		thumb(obj, &in);
		psvr2_steamvr_skel_open_thumb(*(int32_t *)(obj + 8), &in, obj[0x1508], o);
		memcpy(s, obj + 0xe94, sizeof s);
		if (cmp(s, o)) {
			if (bad++ < 5)
				printf("skel open thumb it=%d side=%d mismatch\n", it, side);
		}
	}
	printf("skel open thumb: %d/%d mismatches (max error %.1f x 2^-24 on quaternions)\n", bad, n, maxerr);
	total += bad;

	/* With-controller thumb (0x180129c20). */
	void (*cthumb)(void *, const void *) = (void (*)(void *, const void *))SONY(0x180129c20);
	bad = 0;
	maxerr = 0;
	for (int it = 0; it < n; it++) {
		const int side = rnd() & 1;
		memset(obj, 0, sizeof obj);
		*(int32_t *)(obj + 8) = side ? (int32_t)(1 + rnd() % 3) : 0;
		for (int k = 0; k < (int)sizeof in; k++)
			((uint8_t *)&in)[k] = (uint8_t)rnd();
		/* make each branch likely: clear most of the touch/button bits */
		in.touch &= rnd() | rnd();
		in.buttons &= rnd() | rnd();
		if (it & 2)
			in.touch &= ~0xf006u;
		if (it & 4)
			in.buttons &= ~0xf00fu;
		obj[0x1508] = (uint8_t)(rnd() & 1);
		for (int k = 0; k < PSVR2_SKEL_BONES * 10; k++)
			((float *)(obj + 0xe94))[k] = rf(-1, 1);
		cthumb(obj, &in);
		psvr2_steamvr_skel_ctrl_thumb(*(int32_t *)(obj + 8), &in, obj[0x1508], o);
		memcpy(s, obj + 0xe94, sizeof s);
		if (cmp(s, o)) {
			if (bad++ < 5)
				printf("skel ctrl thumb it=%d side=%d mismatch\n", it, side);
		}
	}
	printf("skel ctrl thumb: %d/%d mismatches (max error %.1f x 2^-24 on quaternions)\n", bad, n, maxerr);
	total += bad;

	/* With-controller index/middle fingers (0x1801293b0). */
	void (*fing)(void *, const void *) = (void (*)(void *, const void *))SONY(0x1801293b0);
	bad = 0;
	maxerr = 0;
	for (int it = 0; it < n; it++) {
		const int side = rnd() & 1;
		psvr2_skel_input prev;
		memset(obj, 0, sizeof obj);
		*(int32_t *)(obj + 8) = side ? (int32_t)(1 + rnd() % 3) : 0;
		for (int k = 0; k < (int)sizeof in; k++) {
			((uint8_t *)&in)[k] = (uint8_t)rnd();
			((uint8_t *)&prev)[k] = (uint8_t)rnd();
		}
		uint8_t *b = (uint8_t *)&in;
		if (it & 1)	/* near the thresholds */
			b[0x12 + side] = (uint8_t)(3 + rnd() % 5), b[8 + side] = (uint8_t)(rnd() % 3),
			b[0x10 + side] = (uint8_t)(3 + rnd() % 5);
		const uint64_t now = ((uint64_t)rnd() << 32) | rnd();
		memcpy(obj + 0x1370, &prev, sizeof prev);
		memcpy(obj + 0x1408, &now, 8);
		for (int k = 0; k < PSVR2_SKEL_BONES * 10; k++)
			((float *)(obj + 0xe94))[k] = rf(-1, 1);
		/* slerp needs unit quaternions to be meaningful */
		for (int k = 0; k < PSVR2_SKEL_BONES; k++) {
			float *q = (float *)(obj + 0xea0 + k * 0x28);
			float l = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
			for (int j = 0; j < 4; j++)
				q[j] /= l;
		}
		memcpy(o, obj + 0xe94, sizeof o);
		fing(obj, &in);
		uint32_t m = psvr2_steamvr_skel_ctrl_fingers(*(int32_t *)(obj + 8), &in, &prev, o);
		memcpy(s, obj + 0xe94, sizeof s);
		int fail = cmp(s, o);
		for (int k = 0; k < PSVR2_SKEL_BONES; k++) {
			uint64_t t;
			memcpy(&t, obj + 0x1410 + k * 8, 8);
			if (obj[0x13e8 + k] != ((m >> k) & 1) || t != (((m >> k) & 1) ? now : 0))
				fail = 1;
		}
		if (fail && bad++ < 5)
			printf("skel ctrl fingers it=%d side=%d mismatch\n", it, side);
	}
	printf("skel ctrl fingers: %d/%d mismatches (max error %.1f x 2^-24 on quaternions)\n", bad, n, maxerr);
	total += bad;
	return total != 0;
}
