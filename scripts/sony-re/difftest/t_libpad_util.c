#include "sony.h"
#include "psvr2_libpad.h"

typedef uint64_t (*pid_fn)(int16_t, int16_t, int32_t);
typedef uint32_t (*crc_fn)(uint8_t, const uint8_t *, int);
typedef uint32_t (*crcu_fn)(uint32_t, const uint8_t *, int);
typedef void (*quat_fn)(const float *, float *, const float *);
typedef void (*lv_fn)(const uint8_t *, uint8_t *);

static int pid_ours(int which, uint16_t v, uint16_t p, int32_t x)
{
	switch (which) {
	case 0: return psvr2_pad_is_vr(v, p);
	case 1: return psvr2_pad_is_vr_left(v, p, (uint8_t)x);
	case 2: return psvr2_pad_is_vr_right(v, p, (uint8_t)x);
	case 3: return psvr2_pad_is_dualsense_std(v, p);
	case 4: return psvr2_pad_is_ps5_family(v, p);
	case 5: return psvr2_pad_is_0ba0(v, p);
	case 6: return psvr2_pad_is_edge(v, p);
	case 7: return psvr2_pad_is_ds4_new(v, p, x);
	case 8: return psvr2_pad_is_ds4v2_fw(v, p, (uint16_t)x);
	default: return psvr2_pad_is_dualsense(v, p);
	}
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	static const uint64_t pid_va[10] = {
		0x1801c9440, 0x1801c9470, 0x1801c94b0, 0x1801c9500, 0x1801c9530,
		0x1801c9590, 0x1801c95c0, 0x1801c95f0, 0x1801c9630, 0x1801c9690,
	};
	static const uint16_t pids[] = { 0x0ce7, 0x0e45, 0x0e46, 0x0ce6, 0x0df2, 0x05c4, 0x09cc, 0x0ba0, 0x0e44, 0x0e47 };
	int bad = 0, n = 0;
	for (int w = 0; w < 10; w++) {
		pid_fn f = (pid_fn)SONY(pid_va[w]);
		for (int i = 0; i < 20000; i++, n++) {
			uint16_t v = rnd() % 4 ? 0x054c : (uint16_t)rnd();
			uint16_t p = rnd() % 4 ? pids[rnd() % 10] : (uint16_t)rnd();
			int32_t x = (int32_t)(rnd() % 3 ? rnd() % 64 : rnd());
			if (w == 8)
				x = rnd() & 0xffff;
			if ((int)(f(v, p, x) & 0xff) != pid_ours(w, v, p, x)) {
				if (bad < 5)
					printf("pid %d: %04x %04x %x\n", w, v, p, x);
				bad++;
			}
		}
	}
	printf("pid predicates: %d/%d mismatches\n", bad, n);
	int fails = bad;

	crc_fn crc = (crc_fn)SONY(0x1801d4390), chk = (crc_fn)SONY(0x1801d43f0);
	crcu_fn upd = (crcu_fn)SONY(0x1801d4490);
	bad = 0;
	for (int i = 0; i < 100000; i++) {
		uint8_t b[96];
		for (int k = 0; k < 96; k++)
			b[k] = (uint8_t)rnd();
		int len = 4 + rnd() % 92;
		uint8_t seed = (uint8_t)rnd();
		uint32_t c = psvr2_pad_crc32(seed, b, len);
		if (rnd() % 2)
			memcpy(b + len - 4, &c, 4);
		if (crc(seed, b, len) != c)
			bad++;
		if ((chk(seed, b, len) & 0xff) != (uint32_t)psvr2_pad_crc32_bad(seed, b, len))
			bad++;
		uint32_t st = rnd();
		if (upd(st, b, len) != psvr2_pad_crc32_update(st, b, len))
			bad++;
	}
	printf("crc32: %d/300000 mismatches\n", bad);
	fails += bad;

	quat_fn q = (quat_fn)SONY(0x1801d44d0);
	bad = 0;
	for (int i = 0; i < 100000; i++) {
		float a[4], b[4], r1[4], r2[4];
		for (int k = 0; k < 4; k++) {
			a[k] = (float)((int32_t)rnd()) / 1e9f;
			b[k] = (float)((int32_t)rnd()) / 1e9f;
		}
		q(a, r1, b);
		psvr2_pad_quat_mul(a, r2, b);
		if (memcmp(r1, r2, sizeof r1))
			bad++;
	}
	printf("quat mul: %d/100000 mismatches\n", bad);
	fails += bad;

	lv_fn lv = (lv_fn)SONY(0x1801c6940);
	bad = 0;
	for (int i = 0; i < 100000; i++) {
		uint8_t in[6], o1[10], o2[10];
		for (int k = 0; k < 6; k++)
			in[k] = (uint8_t)rnd();
		lv(in, o1);
		psvr2_pad_trigger_levels(in, o2);
		if (memcmp(o1, o2, 10))
			bad++;
	}
	printf("level unpack: %d/100000 mismatches\n", bad);
	fails += bad;
	return fails != 0;
}
