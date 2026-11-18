// SPDX-License-Identifier: GPL-2.0
/*
 * Refresh-rate / EDID-type choice at init, see docs/sony-re/steamvr.md
 * ("Refresh rate and EDID type").
 */
#include <math.h>
#include "psvr2_steamvr.h"

static int near(float a, float b)
{
	return fabsf(a - b) < 0.1f;
}

psvr2_edid_choice psvr2_steamvr_edid_select(float preferred, uint8_t cur_edid)
{
	psvr2_edid_choice c = { preferred, 0, -1 };

	if (fabsf(preferred - 90.0f) > 0.1f && fabsf(preferred - 120.0f) > 0.1f) {
		c.rate = 90.0f;
		c.store_setting = 1;
	}
	/* The EDID only changes when it does not offer the wanted rate. */
	if (cur_edid == PSVR2_EDID_90_ONLY && near(c.rate, 120.0f))
		c.set_edid = PSVR2_EDID_90_120;
	else if (cur_edid == PSVR2_EDID_90_120 && near(c.rate, 90.0f))
		c.set_edid = PSVR2_EDID_90_ONLY;
	return c;
}
