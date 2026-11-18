/* SPDX-License-Identifier: GPL-2.0 */
/* Helpers for scripts/sony-re/difftest/t_trace_*.c only. */
#ifndef PSVR2_TRACE_DIFFTEST_H
#define PSVR2_TRACE_DIFFTEST_H
#include <stdint.h>
#include <string.h>

/* Random float: mostly moderate values, sometimes zero, tiny, huge or any bit pattern. */
static float rf(void)
{
	uint32_t r = rnd();
	switch (r & 15) {
	case 0: return 0.0f;
	case 1: { float f; uint32_t b = rnd(); memcpy(&f, &b, 4); return f; }
	case 2: return ((int32_t)rnd()) * 1e-12f;
	case 3: return ((int32_t)rnd()) * 1e-3f;
	default: return ((int32_t)rnd()) * (1.0f / 2147483648.0f) * 4.0f;
	}
}

/* Moderate random float in [-s, s). */
static float rfs(float s) { return ((int32_t)rnd()) * (1.0f / 2147483648.0f) * s; }

static void rfill(float *p, int n) { for (int i = 0; i < n; i++) p[i] = rf(); }

/* Bitwise equality, any NaN equals any NaN. */
static int feq(float a, float b)
{
	if (a != a && b != b)
		return 1;
	return memcmp(&a, &b, 4) == 0;
}

static int deq(double a, double b)
{
	if (a != a && b != b)
		return 1;
	return memcmp(&a, &b, 8) == 0;
}

static int feqn(const float *a, const float *b, int n)
{
	for (int i = 0; i < n; i++)
		if (!feq(a[i], b[i]))
			return 0;
	return 1;
}

/* Distance in ulps between two finite floats (large for NaN mismatch). */
static int fulp(float a, float b)
{
	if (a != a || b != b)
		return (a != a && b != b) ? 0 : 1 << 30;
	int32_t ia, ib; memcpy(&ia, &a, 4); memcpy(&ib, &b, 4);
	if (ia < 0) ia = INT32_MIN - ia;
	if (ib < 0) ib = INT32_MIN - ib;
	int64_t d = (int64_t)ia - ib;
	if (d < 0) d = -d;
	return d > (1 << 30) ? 1 << 30 : (int)d;
}

static int fulpn(const float *a, const float *b, int n)
{
	int m = 0;
	for (int i = 0; i < n; i++) { int d = fulp(a[i], b[i]); if (d > m) m = d; }
	return m;
}

/*
 * Route the libm calls of the code under test to ucrtbase.dll, the CRT that
 * Sony's DLL imports, so a test isolates our arithmetic from libm accuracy.
 * These definitions win over the import library at link time.
 */
#include <windows.h>
static HMODULE tr_ucrt;
static FARPROC tr_libm(const char *n)
{
	if (!tr_ucrt)
		tr_ucrt = LoadLibraryA("ucrtbase.dll");
	return GetProcAddress(tr_ucrt, n);
}
#define TR_F1(name) float name(float x) { static float (*p)(float); if (!p) p = (float (*)(float))tr_libm(#name); return p(x); }
#define TR_D1(name) double name(double x) { static double (*p)(double); if (!p) p = (double (*)(double))tr_libm(#name); return p(x); }
TR_F1(sinf) TR_F1(cosf) TR_F1(tanf) TR_F1(asinf) TR_F1(acosf) TR_F1(atanf) TR_F1(expf) TR_F1(logf)
TR_D1(sin) TR_D1(cos) TR_D1(tan) TR_D1(acos) TR_D1(log)
float atan2f(float y, float x) { static float (*p)(float, float); if (!p) p = (float (*)(float, float))tr_libm("atan2f"); return p(y, x); }
double pow(double x, double y) { static double (*p)(double, double); if (!p) p = (double (*)(double, double))tr_libm("pow"); return p(x, y); }
void sincosf(float x, float *s, float *c) { *s = sinf(x); *c = cosf(x); }
void sincos(double x, double *s, double *c) { *s = sin(x); *c = cos(x); }

static int report(const char *name, int bad, int n)
{
	printf("%-44s %d/%d mismatches\n", name, bad, n);
	return bad != 0;
}
#endif
