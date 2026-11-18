/* SPDX-License-Identifier: GPL-2.0 */
#ifndef PSVR2_SIE_RELOC_SHA256_H
#define PSVR2_SIE_RELOC_SHA256_H
#include <stdint.h>
#include <stddef.h>

void psvr2_sha256(const void *data, size_t len, uint8_t out[32]);
#endif
