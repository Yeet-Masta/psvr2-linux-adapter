// SPDX-License-Identifier: GPL-2.0
// The C helpers from userspace/lib/sie, usable from C++ (their headers use
// C11 _Static_assert).
#pragma once

#define _Static_assert static_assert
extern "C" {
#include "sie/irtracker/psvr2_ld.h"
#include "sie/libpad/psvr2_libpad.h"
}
#undef _Static_assert
