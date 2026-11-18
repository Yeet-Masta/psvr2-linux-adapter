/* Fake OpenVR host for calling Sony's SteamVR device code under Wine.
 * Included only by t_steamvr_*.c (run.sh globs *.c, so this is not a test).
 *
 * - hook(va, fn): overwrite the start of a Sony function with a jump to fn,
 *   so USB/manager singletons can be replaced by stubs.
 * - A fake IVRDriverContext whose GetGenericInterface hands out recording
 *   IVRProperties / IVRSettings / IVRDriverLog / IVRDriverInput objects.
 *   Every property write is appended to g_props. */
#pragma once
#include "sony.h"

static void hook(uintptr_t va, void *fn)
{
	uint8_t *p = SONY(va);
	DWORD old;

	VirtualProtect(p, 12, PAGE_EXECUTE_READWRITE, &old);
	p[0] = 0x48; p[1] = 0xb8;               /* mov rax, imm64 */
	memcpy(p + 2, &fn, 8);
	p[10] = 0xff; p[11] = 0xe0;             /* jmp rax */
	VirtualProtect(p, 12, old, &old);
	FlushInstructionCache(GetCurrentProcess(), p, 12);
}

static void *stub_ret0(void) { return 0; }

/* ---- recorded property writes ---- */
typedef struct {
	uint64_t container;
	uint32_t prop, tag, size, write_type;
	uint8_t data[4096];
} prop_rec;
static prop_rec g_props[256];
static int g_nprops;

typedef struct {
	uint32_t prop, write_type;
	int32_t set_error;
	uint32_t pad;
	void *buf;
	uint32_t size, tag;
	int32_t error;
	uint32_t pad2;
} PropertyWrite_t;

typedef struct {
	uint32_t prop, pad;
	void *buf;
	uint32_t size, tag;
	uint32_t required;
	int32_t error;
} PropertyRead_t;

static const char *g_user_config_path = "C:\\steam\\config";

static int __cdecl p_read(void *self, uint64_t c, PropertyRead_t *b, uint32_t n)
{
	(void)self; (void)c;
	for (uint32_t i = 0; i < n; i++) {
		if (b[i].prop == 6000 && g_user_config_path) {
			uint32_t len = (uint32_t)strlen(g_user_config_path) + 1;
			b[i].tag = 5;
			b[i].required = len;
			if (len > b[i].size) { b[i].error = 3; continue; }
			memcpy(b[i].buf, g_user_config_path, len);
			b[i].error = 0;
		} else {
			b[i].error = 1;
			b[i].tag = 0;
		}
	}
	return 0;
}

static int __cdecl p_write(void *self, uint64_t c, PropertyWrite_t *b, uint32_t n)
{
	(void)self;
	for (uint32_t i = 0; i < n; i++) {
		prop_rec *r = &g_props[g_nprops < 255 ? g_nprops++ : 255];
		r->container = c;
		r->prop = b[i].prop;
		r->write_type = b[i].write_type;
		r->tag = b[i].tag;
		r->size = b[i].size;
		memset(r->data, 0, sizeof r->data);
		if (b[i].buf && b[i].size <= sizeof r->data)
			memcpy(r->data, b[i].buf, b[i].size);
		b[i].error = 0;
	}
	return 0;
}

static const char *__cdecl p_errname(void *self, int e) { (void)self; (void)e; return "err"; }
static uint64_t __cdecl p_container(void *self, uint32_t id) { (void)self; return 0x1000 + id; }
static void *g_props_vt[] = { (void *)p_read, (void *)p_write, (void *)p_errname, (void *)p_container };
static struct { void **vt; } g_props_obj = { g_props_vt };

/* ---- settings ---- */
static float g_set_float[8];        /* by key: see s_key() */
static int g_set_bool[8];
static float g_set_float_written;
static int g_set_float_writes;

static int s_key(const char *sec, const char *key)
{
	if (!strcmp(key, "preferredRefreshRate")) return 0;
	if (!strcmp(key, "ipd")) return 1;
	if (!strcmp(sec, "direct_mode") && !strcmp(key, "enable")) return 2;
	return 7;
}
static const char *__cdecl s_errname(void *s, int e) { (void)s; (void)e; return ""; }
static void __cdecl s_setbool(void *s, const char *a, const char *b, int v, int *e) { (void)s; g_set_bool[s_key(a, b)] = v; if (e) *e = 0; }
static void __cdecl s_setint(void *s, const char *a, const char *b, int v, int *e) { (void)s; (void)a; (void)b; (void)v; if (e) *e = 0; }
static void __cdecl s_setfloat(void *s, const char *a, const char *b, float v, int *e)
{
	(void)s;
	g_set_float[s_key(a, b)] = v;
	g_set_float_written = v;
	g_set_float_writes++;
	if (e) *e = 0;
}
static void __cdecl s_setstr(void *s, const char *a, const char *b, const char *v, int *e) { (void)s; (void)a; (void)b; (void)v; if (e) *e = 0; }
static int __cdecl s_getbool(void *s, const char *a, const char *b, int *e) { (void)s; if (e) *e = 0; return g_set_bool[s_key(a, b)]; }
static int __cdecl s_getint(void *s, const char *a, const char *b, int *e) { (void)s; (void)a; (void)b; if (e) *e = 0; return 0; }
static float __cdecl s_getfloat(void *s, const char *a, const char *b, int *e) { (void)s; if (e) *e = 0; return g_set_float[s_key(a, b)]; }
static void __cdecl s_getstr(void *s, const char *a, const char *b, char *v, uint32_t n, int *e) { (void)s; (void)a; (void)b; if (n) v[0] = 0; if (e) *e = 0; }
static void __cdecl s_rm(void *s, const char *a, int *e) { (void)s; (void)a; if (e) *e = 0; }
static void __cdecl s_rmkey(void *s, const char *a, const char *b, int *e) { (void)s; (void)a; (void)b; if (e) *e = 0; }
static void *g_set_vt[] = { (void *)s_errname, (void *)s_setbool, (void *)s_setint, (void *)s_setfloat,
	(void *)s_setstr, (void *)s_getbool, (void *)s_getint, (void *)s_getfloat, (void *)s_getstr,
	(void *)s_rm, (void *)s_rmkey };
static struct { void **vt; } g_set_obj = { g_set_vt };

/* ---- log ---- */
static int g_log_verbose;
static void __cdecl l_log(void *s, const char *m) { (void)s; if (g_log_verbose) printf("[sony] %s", m); }
static void *g_log_vt[] = { (void *)l_log };
static struct { void **vt; } g_log_obj = { g_log_vt };

/* ---- driver input: hands out sequential handles ---- */
typedef struct { uint64_t container; char path[128]; int kind; char skel[64], base[64]; uint32_t ngrip; void *grip; } comp_rec;
static comp_rec g_comps[128];
static int g_ncomps;
static uint64_t g_next_handle = 0x500;

static int add_comp(uint64_t c, const char *name, uint64_t *h, int kind)
{
	if (g_ncomps < 128) {
		memset(&g_comps[g_ncomps], 0, sizeof g_comps[0]);
		g_comps[g_ncomps].container = c;
		snprintf(g_comps[g_ncomps].path, 128, "%s", name);
		g_comps[g_ncomps].kind = kind;
		g_ncomps++;
	}
	*h = g_next_handle++;
	return 0;
}
static int __cdecl i_bool(void *s, uint64_t c, const char *n, uint64_t *h) { (void)s; return add_comp(c, n, h, 0); }
static int __cdecl i_upd_bool(void *s, uint64_t h, int v, double t) { (void)s; (void)h; (void)v; (void)t; return 0; }
static int __cdecl i_scalar(void *s, uint64_t c, const char *n, uint64_t *h, int type, int units) { (void)s; return add_comp(c, n, h, 1 + type * 4 + units * 16); }
static int __cdecl i_upd_scalar(void *s, uint64_t h, float v, double t) { (void)s; (void)h; (void)v; (void)t; return 0; }
static int __cdecl i_haptic(void *s, uint64_t c, const char *n, uint64_t *h) { (void)s; return add_comp(c, n, h, 2); }
static int __cdecl i_skel(void *s, uint64_t c, const char *n, const char *sk, const char *base, int lvl, void *grip, uint32_t ng, uint64_t *h)
{
	(void)s;
	int r = add_comp(c, n, h, 3 + lvl * 4);
	comp_rec *cr = &g_comps[g_ncomps - 1];
	snprintf(cr->skel, 64, "%s", sk ? sk : "(null)");
	snprintf(cr->base, 64, "%s", base ? base : "(null)");
	cr->ngrip = ng;
	cr->grip = grip;
	return r;
}
static int __cdecl i_upd_skel(void *s, uint64_t h, int mode, void *t, uint32_t n) { (void)s; (void)h; (void)mode; (void)t; (void)n; return 0; }
static void *g_in_vt[] = { (void *)i_bool, (void *)i_upd_bool, (void *)i_scalar, (void *)i_upd_scalar,
	(void *)i_haptic, (void *)i_skel, (void *)i_upd_skel };
static struct { void **vt; } g_in_obj = { g_in_vt };

/* ---- server driver host: records pose updates ---- */
static uint8_t g_pose[0x118];
static int g_npose;
static uint32_t g_pose_id;
static int __cdecl h_added(void *s, const char *a, int b, void *c) { (void)s; (void)a; (void)b; (void)c; return 1; }
static void __cdecl h_pose(void *s, uint32_t id, const void *pose, uint32_t size)
{
	(void)s;
	g_pose_id = id;
	memcpy(g_pose, pose, size < sizeof g_pose ? size : sizeof g_pose);
	g_npose++;
}
static void *g_host_vt[] = { (void *)h_added, (void *)h_pose };
static struct { void **vt; } g_host_obj = { g_host_vt };

/* ---- driver context ---- */
static void *__cdecl c_get(void *s, const char *name, int *err)
{
	(void)s;
	if (err) *err = 0;
	if (!strncmp(name, "IVRProperties_", 14)) return &g_props_obj;
	if (!strncmp(name, "IVRSettings_", 12)) return &g_set_obj;
	if (!strncmp(name, "IVRDriverLog_", 13)) return &g_log_obj;
	if (!strncmp(name, "IVRDriverInput_", 15)) return &g_in_obj;
	if (!strncmp(name, "IVRServerDriverHost_", 20)) return &g_host_obj;
	if (err) *err = 1;
	return 0;
}
static uint64_t __cdecl c_handle(void *s) { (void)s; return 0x77; }
static void *g_ctx_vt[] = { (void *)c_get, (void *)c_handle };
static struct { void **vt; } g_ctx_obj = { g_ctx_vt };

/* Sony's cached interface pointers (globals in .data). */
#define G_CTX		0x180354cb8
#define G_PROPS_A	0x180354cc0
#define G_PROPS_B	0x180354cc8
#define G_SETTINGS	0x180354cd0
#define G_PROPS_C	0x180354cd8
#define G_LOG		0x180354cf8
#define G_INPUT		0x180354d10
#define G_HOST		0x180354ce0
#define G_CAESAR	0x180354d40

static void host_install(void)
{
	*(void **)SONY(G_CTX) = &g_ctx_obj;
	*(void **)SONY(G_PROPS_A) = &g_props_obj;
	*(void **)SONY(G_PROPS_B) = &g_props_obj;
	*(void **)SONY(G_PROPS_C) = &g_props_obj;
	*(void **)SONY(G_SETTINGS) = &g_set_obj;
	*(void **)SONY(G_LOG) = &g_log_obj;
	*(void **)SONY(G_INPUT) = &g_in_obj;
	*(void **)SONY(G_HOST) = &g_host_obj;
	/* Non-null so FUN_180124c90 does not construct the USB manager; every
	 * manager method a test reaches must be hooked. */
	*(void **)SONY(G_CAESAR) = (void *)0x1;
}
