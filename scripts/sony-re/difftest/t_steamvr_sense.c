/* SenseDevice::Activate (0x1801a5760): properties and input components. */
#include "t_steamvr_host.h"
#include "psvr2_steamvr.h"

int main(int argc, char **argv)
{
	sony_load(argc > 1 ? argv[1] : "sony.dll");
	host_install();
	hook(0x18016b660, (void *)stub_ret0);   /* thread start */
	int64_t (*act)(void *, uint32_t) = (int64_t (*)(void *, uint32_t))SONY(0x1801a5760);
	static uint8_t dev[0x800];
	int bad = 0, n = 0;

	for (int it = 0; it < 64; it++) {
		const int side = it & 1;
		const uint32_t id = 1 + rnd() % 15;
		memset(dev, 0, sizeof dev);
		*(uint32_t *)(dev + 0xc) = side;
		g_nprops = 0;
		g_ncomps = 0;
		g_next_handle = 0x500;
		act(dev, id);
		n++;
		psvr2_prop_list l;
		psvr2_component c[PSVR2_SENSE_COMPONENTS];
		psvr2_steamvr_sense_props(side, &l);
		const int nc = psvr2_steamvr_sense_components(side, c);
		int ok = g_nprops == l.n && g_ncomps == nc && *(uint32_t *)(dev + 8) == id;
		for (int k = 0; ok && k < l.n; k++) {
			const prop_rec *a = &g_props[k];
			const psvr2_prop *b = &l.p[k];
			ok = a->container == 0x1000 + id && a->prop == b->prop && a->tag == b->tag &&
			     a->size == b->size && !memcmp(a->data, b->data, b->size);
		}
		for (int k = 0; ok && k < nc; k++) {
			const comp_rec *a = &g_comps[k];
			int kind = c[k].kind == PSVR2_COMP_BOOL ? 0 : c[k].kind == PSVR2_COMP_SCALAR ? 1 + 16 * c[k].scalar_two_sided :
				   c[k].kind == PSVR2_COMP_HAPTIC ? 2 : 3 + 4 * 1;
			uint64_t h;
			memcpy(&h, dev + 8 * c[k].slot, 8);
			ok = a->container == 0x1000 + id && !strcmp(a->path, c[k].path) && a->kind == kind &&
			     h == 0x500 + (uint64_t)k;
			if (ok && c[k].kind == PSVR2_COMP_SKELETON)
				ok = !strcmp(a->skel, c[k].skeleton) && !strcmp(a->base, c[k].base_pose) &&
				     a->ngrip == 0 && a->grip == NULL;
			if (!ok)
				printf("comp %d %s: kind %d/%d slot handle %llx\n", k, a->path, a->kind, kind,
				       (unsigned long long)h);
		}
		if (!ok)
			bad++;
	}
	printf("sense activate: %d/%d mismatches\n", bad, n);
	return bad != 0;
}
