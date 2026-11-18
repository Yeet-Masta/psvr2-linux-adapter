#include "sony.h"
#include "psvr2_libpad.h"

typedef void (*scale_fn)(uint8_t *, uint8_t *, uint8_t *, int, char);
typedef int32_t (*uns_fn)(const uint8_t *, uint8_t *, uint8_t *);
typedef void (*fill_fn)(uint8_t *, psvr2_pad *);

static void rand_effects(uint8_t *blk)
{
	static const uint8_t modes[6] = { 0x05, 0x21, 0x22, 0x25, 0x26, 0x00 };
	blk[0] = rnd() % 4;
	for (int s = 0; s < 2; s++) {
		uint8_t *e = blk + 1 + 0xb * s;
		e[0] = modes[rnd() % 6];
		for (int i = 1; i < 11; i++)
			e[i] = (uint8_t)rnd();
		/* Mostly valid masks; the all-zero mask reads past an array in Sony. */
		if (!(e[1] | (e[2] & 3)))
			e[1] = 1 << (rnd() % 8);
		if (rnd() % 3 == 0) {	/* uniform levels */
			uint32_t lv = rnd() % 8, bits = 0;
			for (int n = 0; n < 10; n++)
				bits |= lv << (3 * n);
			memcpy(e + 3, &bits, 4);
		}
	}
}

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	scale_fn s_scale = (scale_fn)SONY(0x1801c28a0);
	uns_fn s_uns = (uns_fn)SONY(0x1801c6a40);
	fill_fn s_ds = (fill_fn)SONY(0x1801c6330), s_ds4 = (fill_fn)SONY(0x1801c6550);
	int bad = 0, n = 0, fails = 0;

	for (int k = 0; k < 400000; k++, n++) {
		uint8_t a[3], b[3];
		for (int i = 0; i < 3; i++)
			a[i] = b[i] = (uint8_t)(k % 2 ? rnd() % 40 : rnd());
		const int br = k % 5 ? (int)(rnd() % 300) : (int)rnd() % 70000 - 1000;
		const int nf = rnd() % 2;
		s_scale(&a[0], &a[1], &a[2], br, (char)nf);
		psvr2_pad_lightbar_scale(&b[0], &b[1], &b[2], br, nf);
		bad += memcmp(a, b, 3) != 0;
	}
	printf("lightbar scale: %d/%d mismatches\n", bad, n);
	fails += bad;

	bad = n = 0;
	for (int k = 0; k < 200000; k++, n++) {
		uint8_t blk[0x17];
		rand_effects(blk);
		uint8_t a0 = 9, a1 = 9, b0 = 7, b1 = 7;
		const int32_t ra = s_uns(blk, &a0, &a1), rb = psvr2_pad_effect_unsupported(blk, &b0, &b1);
		bad += ra != rb || a0 != b0 || a1 != b1;
	}
	printf("effect unsupported check: %d/%d mismatches\n", bad, n);
	fails += bad;

	static psvr2_pad pa, pb;
	int cov_rep = 0;
	bad = n = 0;
	for (int k = 0; k < 300000; k++, n++) {
		uint8_t *pp = (uint8_t *)&pa;
		for (size_t i = 0; i < 0x60; i++)
			pp[i] = (uint8_t)rnd();
		pa.vid = 0x054c;
		pa.pid = rnd() % 2 ? 0x0ce6 : 0x0df2;
		pa.fw = rnd() % 2 ? 0x200 + rnd() % 0x200 : rnd();
		pa.player = rnd() % 7 - 1;
		pa.mic_led = rnd() % 5;
		pa.light_dim = rnd() % 4;
		pa.trig_level = rnd() % 4 != 0;
		for (int i = 0; i < 3; i++) {
			pa.ds_rgb[i] = (uint8_t)rnd();
			pa.ds4_rgb[i] = (uint8_t)(rnd() % 2 ? rnd() : rnd() % 30);
		}
		rand_effects(&pa.eff_mask);
		pb = pa;
		uint8_t ba[0x40], bb[0x40];
		for (int i = 0; i < 0x40; i++)
			ba[i] = bb[i] = (uint8_t)rnd();
		if (k % 2) {
			s_ds(ba, &pa);
			psvr2_ds_output_fill(&pb, bb);
			cov_rep += pb.trig_level && (pb.est_trig[0] || pb.est_trig[1]);
		} else {
			s_ds4(ba, &pa);
			psvr2_ds4_output_fill(&pb, bb);
		}
		if (memcmp(ba, bb, sizeof ba) || memcmp(&pa, &pb, sizeof pa)) {
			if (bad < 5) {
				int o = 0;
				while (o < 0x40 && ba[o] == bb[o])
					o++;
				printf("%s k %d: buf diff 0x%x, pad %s\n", k % 2 ? "ds" : "ds4", k, o,
				       memcmp(&pa, &pb, sizeof pa) ? "differs" : "same");
			}
			bad++;
		}
	}
	printf("DualSense/DS4 output fill: %d/%d mismatches (replaced effects %d)\n", bad, n, cov_rep);
	fails += bad;
	return fails != 0;
}
