// SPDX-License-Identifier: GPL-2.0
/*
 * Sense controller properties and input components, see
 * docs/sony-re/steamvr.md ("Sense controller").
 */
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

static void add_u64(psvr2_prop_list *l, uint32_t prop, uint64_t v)
{
	memcpy(add(l, prop, 3, 8)->inl, &v, 8);
}

static void add_s(psvr2_prop_list *l, uint32_t prop, const char *s)
{
	add(l, prop, PSVR2_TAG_STRING, (uint32_t)strlen(s) + 1)->data = s;
}

void psvr2_steamvr_sense_props(int side, psvr2_prop_list *l)
{
	static const struct { uint32_t prop; const char *name; } icons[] = {
		{ 5004, "ready@2x.png" }, { 5001, "off@2x.png" }, { 5002, "searching@2x.gif" },
		{ 5003, "searching_alert@2x.gif" }, { 5005, "ready_alert@2x.png" },
		{ 5006, "error@2x.png" }, { 5007, "standby@2x.png" }, { 5008, "ready_low@2x.png" },
		{ 5009, "standby_alert@2x.png" },
	};
	static char icon_path[2][9][96];
	const int r = side != PSVR2_SIDE_LEFT;

	l->n = 0;
	add_i32(l, 2075, 3);                    /* HmdTrackingStyle: inside-out */
	add_u64(l, 2004, 2);                    /* CurrentUniverseId */
	add_i32(l, 3002, 2);                    /* Axis0Type: joystick */
	add_i32(l, 3003, 3);                    /* Axis1Type: trigger */
	add_i32(l, 3004, 3);                    /* Axis2Type: trigger */
	/* SupportedButtons: System, ApplicationMenu, Grip, A, Axis0..2 */
	add_u64(l, 3001, 0x0000000700000087ull);
	add_s(l, 1001, r ? "PlayStation VR2 Sense Right" : "PlayStation VR2 Sense Left");
	add_s(l, 1003, r ? "{playstation_vr2}playstation_vr2_sense_right" :
			   "{playstation_vr2}playstation_vr2_sense_left");
	add_i32(l, 3007, r ? 2 : 1);            /* ControllerRoleHint */
	add_s(l, 1037, "{playstation_vr2}/input/playstation_vr2_sense_controller_profile.json");
	for (int i = 0; i < 9; i++) {
		snprintf(icon_path[r][i], sizeof(icon_path[r][i]), "{playstation_vr2}/icons/%s_controller_status_%s",
			 r ? "right" : "left", icons[i].name);
		add_s(l, icons[i].prop, icon_path[r][i]);
	}
}

int psvr2_steamvr_sense_components(int side, psvr2_component *c)
{
	/* Names differ per side. Trigger, grip, stick and PS slots are shared
	 * between the hands; the face buttons have their own slots. */
	static const struct { const char *l, *r; int kind, two, slot; } t[] = {
		{ "/output/haptic", "/output/haptic", PSVR2_COMP_HAPTIC, 0, 0x36 },
		{ "/input/square/click", "/input/cross/click", PSVR2_COMP_BOOL, 0, 0 },
		{ "/input/square/touch", "/input/cross/touch", PSVR2_COMP_BOOL, 0, 0 },
		{ "/input/triangle/click", "/input/circle/click", PSVR2_COMP_BOOL, 0, 0 },
		{ "/input/triangle/touch", "/input/circle/touch", PSVR2_COMP_BOOL, 0, 0 },
		{ "/input/create/click", "/input/options/click", PSVR2_COMP_BOOL, 0, 0 },
		{ "/input/create/touch", "/input/options/touch", PSVR2_COMP_BOOL, 0, 0 },
		{ "/input/l2/touch", "/input/r2/touch", PSVR2_COMP_BOOL, 0, 0x40 },
		{ "/input/l2/value", "/input/r2/value", PSVR2_COMP_SCALAR, 0, 0x3f },
		{ "/input/l2_sensor/value", "/input/r2_sensor/value", PSVR2_COMP_SCALAR, 0, 0x41 },
		{ "/input/l1/touch", "/input/r1/touch", PSVR2_COMP_BOOL, 0, 0x43 },
		{ "/input/l1/value", "/input/r1/value", PSVR2_COMP_SCALAR, 0, 0x42 },
		{ "/input/l1_sensor/value", "/input/r1_sensor/value", PSVR2_COMP_SCALAR, 0, 0x44 },
		{ "/input/left_stick/click", "/input/right_stick/click", PSVR2_COMP_BOOL, 0, 0x4b },
		{ "/input/left_stick/touch", "/input/right_stick/touch", PSVR2_COMP_BOOL, 0, 0x4c },
		{ "/input/left_stick/x", "/input/right_stick/x", PSVR2_COMP_SCALAR, 1, 0x4d },
		{ "/input/left_stick/y", "/input/right_stick/y", PSVR2_COMP_SCALAR, 1, 0x4e },
		{ "/input/left_ps/click", "/input/right_ps/click", PSVR2_COMP_BOOL, 0, 0x45 },
		{ "/input/left_ps/touch", "/input/right_ps/touch", PSVR2_COMP_BOOL, 0, 0x46 },
		{ "/input/skeleton/left", "/input/skeleton/right", PSVR2_COMP_SKELETON, 0, 0x4f },
	};
	/* Face-button slots: left square/triangle/create = 0x3b..0x3e, 0x49, 0x4a;
	 * right cross/circle/options = 0x37..0x3a, 0x47, 0x48. */
	static const int face[2][6] = { { 0x3b, 0x3c, 0x3d, 0x3e, 0x49, 0x4a },
					{ 0x37, 0x38, 0x39, 0x3a, 0x47, 0x48 } };
	const int r = side != PSVR2_SIDE_LEFT;

	for (int i = 0; i < PSVR2_SENSE_COMPONENTS; i++) {
		memset(&c[i], 0, sizeof(c[i]));
		c[i].path = r ? t[i].r : t[i].l;
		c[i].kind = t[i].kind;
		c[i].scalar_two_sided = t[i].two;
		c[i].slot = (i >= 1 && i <= 6) ? face[r][i - 1] : t[i].slot;
	}
	c[19].skeleton = r ? "/skeleton/hand/right" : "/skeleton/hand/left";
	c[19].base_pose = "/pose/raw";
	return PSVR2_SENSE_COMPONENTS;
}
