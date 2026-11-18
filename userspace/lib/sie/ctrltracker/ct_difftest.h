/* SPDX-License-Identifier: GPL-2.0 */
/* Helpers shared by scripts/sony-re/difftest/t_ctrltracker_*.c only. */
#ifndef PSVR2_CT_DIFFTEST_H
#define PSVR2_CT_DIFFTEST_H
#include <stdint.h>
#include <string.h>

/* Random float: mostly moderate values, sometimes tiny/huge/zero. */
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

/* Bitwise equality, but any NaN equals any NaN. */
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

/*
 * Sony's DLL imports its libm from the UCRT (api-ms-win-crt-math). The test
 * binary would otherwise use mingw's own implementations, which differ by up
 * to 1 ulp. Route ours to the same UCRT entry points so the comparison covers
 * the arithmetic around the libm calls exactly.
 */
static FARPROC ucrt_fn(const char *name)
{
	static HMODULE m;
	if (!m)
		m = LoadLibraryA("ucrtbase.dll");
	return GetProcAddress(m, name);
}
#define UCRT_F1(fn) float fn(float x) { static float (*p)(float); \
	if (!p) p = (float (*)(float))ucrt_fn(#fn); return p(x); }
#define UCRT_D1(fn) double fn(double x) { static double (*p)(double); \
	if (!p) p = (double (*)(double))ucrt_fn(#fn); return p(x); }
UCRT_F1(sinf) UCRT_F1(cosf) UCRT_F1(acosf) UCRT_F1(asinf) UCRT_F1(atanf)
UCRT_F1(tanf) UCRT_F1(expf) UCRT_F1(logf)
UCRT_D1(sin) UCRT_D1(cos) UCRT_D1(acos) UCRT_D1(log)
float atan2f(float y, float x)
{
	static float (*p)(float, float);
	if (!p)
		p = (float (*)(float, float))ucrt_fn("atan2f");
	return p(y, x);
}
double pow(double x, double y)
{
	static double (*p)(double, double);
	if (!p)
		p = (double (*)(double, double))ucrt_fn("pow");
	return p(x, y);
}

static int report(const char *name, int bad, int n)
{
	printf("%-40s %d/%d mismatches\n", name, bad, n);
	return bad != 0;
}
#endif
