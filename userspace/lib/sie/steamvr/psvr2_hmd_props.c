// SPDX-License-Identifier: GPL-2.0
/*
 * HMD properties written on Activate, see docs/sony-re/steamvr.md
 * ("HMD properties").
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "psvr2_steamvr.h"

static psvr2_prop *add(psvr2_prop_list *l, uint32_t prop, uint32_t tag, uint32_t size)
{
	psvr2_prop *p = &l->p[l->n++];

	memset(p, 0, sizeof(*p));
	p->prop = prop;
	p->tag = tag;
	p->size = size;
	p->data = p->inl;
	return p;
}

static void add_i32(psvr2_prop_list *l, uint32_t prop, int32_t v)
{
	memcpy(add(l, prop, PSVR2_TAG_INT32, 4)->inl, &v, 4);
}

static void add_f(psvr2_prop_list *l, uint32_t prop, float v)
{
	memcpy(add(l, prop, PSVR2_TAG_FLOAT, 4)->inl, &v, 4);
}

static void add_b(psvr2_prop_list *l, uint32_t prop, int v)
{
	add(l, prop, PSVR2_TAG_BOOL, 1)->inl[0] = (uint8_t)(v != 0);
}

static void add_s(psvr2_prop_list *l, uint32_t prop, const char *s)
{
	add(l, prop, PSVR2_TAG_STRING, (uint32_t)strlen(s) + 1)->data = s;
}

static float bits_f(uint32_t b)
{
	float f;

	memcpy(&f, &b, 4);
	return f;
}

/* Display timing per refresh rate, seconds. */
#define T90_VSYNC_PHOTONS	bits_f(0x3ab24207)	/* 1.36 ms */
#define T90_PHOTONS_VBLANK	bits_f(0x3c28c155)	/* 10.3 ms */
#define T120_VSYNC_PHOTONS	bits_f(0x3a85b185)	/* 1.02 ms */
#define T120_PHOTONS_VBLANK	bits_f(0x3bfd4bf1)	/* 7.73 ms */

void psvr2_steamvr_hmd_props(const psvr2_hmd_prop_in *in, psvr2_prop_list *l)
{
	static const struct { uint32_t prop; const char *path; } icons[] = {
		{ 5004, "{playstation_vr2}/icons/headset_status_ready@2x.png" },
		{ 5001, "{playstation_vr2}/icons/headset_status_off@2x.png" },
		{ 5002, "{playstation_vr2}/icons/headset_status_searching@2x.gif" },
		{ 5003, "{playstation_vr2}/icons/headset_status_searching_alert@2x.gif" },
		{ 5005, "{playstation_vr2}/icons/headset_status_ready_alert@2x.png" },
		{ 5006, "{playstation_vr2}/icons/headset_status_error@2x.png" },
		{ 5007, "{playstation_vr2}/icons/headset_status_standby@2x.png" },
		{ 5009, "{playstation_vr2}/icons/headset_status_standby_alert@2x.png" },
	};
	float vsync, vblank;

	l->n = 0;
	add_s(l, 1005, "SIE");                          /* ManufacturerName */
	add_i32(l, 2011, 0xd94d);                       /* EdidVendorID: "SNY" */
	/* EdidProductID: config value unless 0 */
	add_i32(l, 2015, in->edid_product_id ? (int32_t)in->edid_product_id : 0xc207);
	add_i32(l, 2110, 17);                           /* reserved, undocumented */
	add_i32(l, 2111, 4);
	add_i32(l, 2112, 160);
	add_f(l, 2003, in->ipd_setting);                /* UserIpdMeters */
	if (in->hmd_sw_version <= 0x06000000) {
		add_f(l, 2002, 90.0f);                  /* DisplayFrequency */
	} else {
		psvr2_prop *p = add(l, 2080, PSVR2_TAG_FLOAT, 8);   /* DisplayAvailableFrameRates */
		const float rates[2] = { 90.0f, 120.0f };

		memcpy(p->inl, rates, 8);
		add_f(l, 2002, in->refresh_rate);
	}
	if (in->vsync_to_photons_us < 1 && in->photons_to_vblank_us < 1) {
		const int at90 = fabsf(in->refresh_rate - 90.0f) < 0.1f;  /* NaN: 120 Hz timing */

		vsync = at90 ? T90_VSYNC_PHOTONS : T120_VSYNC_PHOTONS;
		vblank = at90 ? T90_PHOTONS_VBLANK : T120_PHOTONS_VBLANK;
	} else {
		vsync = (float)((double)in->vsync_to_photons_us * 1e-6);
		vblank = (float)((double)in->photons_to_vblank_us * 1e-6);
	}
	add_f(l, 2001, vsync);                          /* SecondsFromVsyncToPhotons */
	add_f(l, 2042, vblank);                         /* SecondsFromPhotonsToVblank */
	add_f(l, 2100, bits_f(0x3d71a9fc));             /* IpdUIRangeMinMeters 0.059 */
	add_f(l, 2101, bits_f(0x3d9374bc));             /* IpdUIRangeMaxMeters 0.072 */
	add_f(l, 2003, (float)in->ipd_mm * bits_f(0x3a83126f));  /* mm -> m */
	add_i32(l, 2075, 3);                            /* HmdTrackingStyle: inside-out cameras */
	add_b(l, 2007, 0);                              /* IsOnDesktop */
	add_s(l, 1001, "PS VR2");                       /* ModelNumber */
	add_b(l, 1025, 1);                              /* ContainsProximitySensor */
	add_s(l, 1037, "{playstation_vr2}/input/playstation_vr2_profile.json");
	for (unsigned i = 0; i < sizeof(icons) / sizeof(icons[0]); i++)
		add_s(l, icons[i].prop, icons[i].path);
	snprintf(l->chaperone_path, sizeof(l->chaperone_path), "%s\\chaperone_info.vrchap",
		 in->user_config_path ? in->user_config_path : "");
	add_s(l, 2048, l->chaperone_path);              /* DriverProvidedChaperonePath */
	add_i32(l, 2056, 80);                           /* DistortionMeshResolution */
}
