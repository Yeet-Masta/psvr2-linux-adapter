/* usbmon pcap -> optical trace F lines (LD packets on EP 0x89).
 * usage: pcap2f usb.pcap realtime_minus_monotonic_ns > f.txt
 * F us counter dev_us no_det period frame_host_us | cam n: x,y,w,h,weight,id,flags ... */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "psvr2_ld.h"
static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
int main(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "rb");
	long long off = atoll(argv[2]);
	uint8_t gh[24], h[16];
	static uint8_t d[300000];
	static psvr2_blob_set bs;
	if (!f || fread(gh, 1, 24, f) != 24) return 1;
	while (fread(h, 1, 16, f) == 16) {
		uint32_t incl = rd32(h + 8);
		if (incl > sizeof(d) || fread(d, 1, incl, f) != incl) break;
		if (d[8] != 'C' || d[10] != 0x89) continue;
		uint32_t len = rd32(d + 36);
		const uint8_t *p = d + 64;
		if (len < PSVR2_LD_PACKET_SIZE || p[0] != 'L' || p[1] != 'D') continue;
		int64_t sec; int32_t usec;
		memcpy(&sec, d + 16, 8); memcpy(&usec, d + 24, 4);
		long long mono_us = (sec * 1000000000LL + usec * 1000LL - off) / 1000;
		uint16_t ver; memcpy(&ver, p + 2, 2);
		printf("F %lld %u %u %d 0 %lld", mono_us, rd32(p + 0x14), rd32(p + 8), p[0x35], mono_us);
		for (int c = 0; c < PSVR2_LD_CAMERAS; c++) {
			uint32_t n = rd32(p + 0x40 + c * 0x2404);
			if (n > PSVR2_LD_MAX_BLOBS) n = PSVR2_LD_MAX_BLOBS;
			int cnt = 0;
			if (!p[0x35])
				for (uint32_t i = 0; i < n; i++)
					psvr2_ld_decode_blob(p + 0x40 + c * 0x2404 + 4 + i * PSVR2_LD_BLOB_SIZE, ver > 0xff, &bs, &cnt, NULL);
			printf(" | %d %d:", c, cnt);
			for (int i = 0; i < cnt; i++)
				printf(" %.1f,%.1f,%d,%d,%.3f,%u,%x", bs.x[i], bs.y[i], bs.w[i], bs.h[i], bs.weight[i], bs.id[i], bs.flags[i]);
		}
		putchar('\n');
	}
	return 0;
}
