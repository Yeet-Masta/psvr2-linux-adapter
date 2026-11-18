#pragma once
/* Differential-test helpers: load the reference DLL under Wine and call
 * internal functions by their address in the documented build. */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
static HMODULE g_sony;
#define SONY(va) ((void *)((uintptr_t)g_sony + ((uintptr_t)(va) - 0x180000000ull)))
static void sony_load(const char *p) {
    g_sony = LoadLibraryA(p);
    if (!g_sony) { printf("load failed %lu\n", GetLastError()); exit(2); }
}
static uint32_t rng_state = 12345;
static uint32_t rnd(void) { rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5; return rng_state; }
