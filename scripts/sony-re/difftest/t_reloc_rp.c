#include "sony.h"
#include "psvr2_reloc_rp.h"

typedef void (*ref_fn)(void *out, const void *rp);

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	ref_fn ref = (ref_fn)SONY(0x1801ab410);
	uint8_t *rp = malloc(PSVR2_RP_PACKET_SIZE), *a = malloc(PSVR2_RP_FRAME_SIZE), *b = malloc(PSVR2_RP_FRAME_SIZE);
	int bad = 0, n = 300;
	for (int i = 0; i < n; i++) {
		for (int k = 0; k < PSVR2_RP_PACKET_SIZE; k++)
			rp[k] = (uint8_t)rnd();
		/* Edge cases: anchor count 0..3 and huge, zero flags. */
		uint32_t cnt = i % 5 == 4 ? rnd() : (uint32_t)(i % 5);
		memcpy(rp + 0x9b01c, &cnt, 4);
		if (i & 8) memset(rp + 0x36e8c, 0, 4);
		if (i & 16) memset(rp + 0x9aec0, 0, 4);
		for (int k = 0; k < PSVR2_RP_FRAME_SIZE; k++)
			a[k] = (uint8_t)rnd();
		memcpy(b, a, PSVR2_RP_FRAME_SIZE);
		ref(a, rp);
		psvr2_reloc_rp_unpack(b, rp);
		if (memcmp(a, b, PSVR2_RP_FRAME_SIZE)) {
			if (bad++ < 3)
				for (int k = 0; k < PSVR2_RP_FRAME_SIZE; k++)
					if (a[k] != b[k]) { printf("  first diff at 0x%x\n", k); break; }
		}
	}
	printf("reloc RP unpack: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
