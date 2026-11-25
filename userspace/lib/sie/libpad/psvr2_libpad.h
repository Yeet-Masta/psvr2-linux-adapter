/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sense / DualSense / DS4 controller library: behaviour reimplemented from
 * docs/sony-re/controllers.md part A and checked against Sony's driver by
 * scripts/sony-re/difftest/t_libpad_*.c.
 *
 * State records keep the byte layout given in the doc so the tests can
 * compare them byte for byte with the reference.
 */
#ifndef PSVR2_SIE_LIBPAD_H
#define PSVR2_SIE_LIBPAD_H
#include <stddef.h>
#include <stdint.h>

#define PSVR2_PAD_OK		0
#define PSVR2_PAD_ERR_LOCK	((int32_t)0x80010002)
#define PSVR2_PAD_ERR_NODEV	((int32_t)0x8001002d)

/* ---- USB ids (doc: "Device families") ---- */
int psvr2_pad_is_vr(uint16_t vid, uint16_t pid);
int psvr2_pad_is_dualsense(uint16_t vid, uint16_t pid);	/* 0ce6 or Edge 0df2 */
int psvr2_pad_is_dualsense_std(uint16_t vid, uint16_t pid);	/* 0ce6 only */
int psvr2_pad_is_edge(uint16_t vid, uint16_t pid);
int psvr2_pad_is_ds4_new(uint16_t vid, uint16_t pid, int32_t hid_ver);
int psvr2_pad_is_ps5_family(uint16_t vid, uint16_t pid);	/* DualSense or VR */
int psvr2_pad_is_vr_left(uint16_t vid, uint16_t pid, uint8_t hand);
int psvr2_pad_is_vr_right(uint16_t vid, uint16_t pid, uint8_t hand);
int psvr2_pad_is_0ba0(uint16_t vid, uint16_t pid);
int psvr2_pad_is_ds4v2_fw(uint16_t vid, uint16_t pid, uint16_t fw);

/* ---- Bluetooth report CRC32 (doc: "Report CRC") ---- */
uint32_t psvr2_pad_crc32_update(uint32_t state, const uint8_t *p, int n);
uint32_t psvr2_pad_crc32(uint8_t seed, const uint8_t *p, int len);
int psvr2_pad_crc32_bad(uint8_t seed, const uint8_t *p, int len);

void psvr2_pad_quat_mul(const float a[4], float out[4], const float b[4]);

/* ---- HID_SYNC: controller clock model (doc: "Controller clock fit") ---- */
#define PSVR2_SYNC_WINDOW_MAX 30
typedef struct {
	uint32_t rtt;		/* +0x00 host_lo - echo */
	uint32_t pad0;
	int64_t mid_minus_ctrl;	/* +0x08 */
	uint32_t ctrl_us;	/* +0x10 */
	uint32_t pad1;
	int64_t ctrl_minus_mid;	/* +0x18 */
	int64_t mid;		/* +0x20 */
	uint64_t host_hi;	/* +0x28 */
	uint64_t host;		/* +0x30 */
	uint32_t echo;		/* +0x38 */
	uint32_t ticks;		/* +0x3c */
} psvr2_sync_sample;

typedef struct {
	uint32_t idx;			/* 0x000 */
	uint32_t nfit;			/* 0x004 */
	int64_t simple_off;		/* 0x008 */
	int64_t fit_off;		/* 0x010 */
	int64_t drift_ppb;		/* 0x018 */
	uint64_t ref_ctrl_us;		/* 0x020 */
	int16_t imu_gap_us;		/* 0x028 IMU time since the previous sample */
	uint8_t gap_runs;		/* 0x02a gaps >= 10 ms, wraps to 1 */
	uint8_t gap_count;		/* 0x02b gaps >= 10 ms */
	uint8_t r02c[0x0c];
	uint32_t last_echo;		/* 0x038 */
	uint32_t last_ticks;		/* 0x03c */
	uint32_t last_ticks2;		/* 0x040 */
	uint8_t wrap;			/* 0x044 */
	uint8_t wrap_idx;		/* 0x045 */
	uint8_t fit_thresh;		/* 0x046 */
	uint8_t r047;
	int64_t prev_delta;		/* 0x048 */
	psvr2_sync_sample s[PSVR2_SYNC_WINDOW_MAX]; /* 0x050 */
	int32_t started;		/* 0x7d0 */
	uint32_t r7d4;
	uint64_t first_host;		/* 0x7d8 */
	uint8_t resets;			/* 0x7e0 */
	uint8_t r7e1;
	uint16_t calls;			/* 0x7e2 */
	uint8_t r7e4[0x0e];
	uint8_t force_reset;		/* 0x7f2 */
	uint8_t warmup;			/* 0x7f3 */
	uint8_t r7f4[0x810 - 0x7f4];
	uint8_t hand;			/* 0x810 */
	uint8_t bt;			/* 0x811 */
	uint8_t r812[6];
	int64_t key_hist[3];		/* 0x818 */
	uint32_t key_hist_idx;		/* 0x830 */
	uint32_t r834;
	int64_t prev_key;		/* 0x838 */
	int64_t key_delta[3];		/* 0x840 */
	uint8_t jump_count;		/* 0x858 */
	uint8_t r859[7];
	int64_t jump_since;		/* 0x860 */
	uint8_t stats[0x920 - 0x868];	/* 0x868 perf statistics */
	uint32_t best_rtt;		/* 0x920 */
	uint8_t mode_req;		/* 0x924 */
	uint8_t mode_cur;		/* 0x925 */
	uint8_t r926[0x0a];
	int64_t n_slow;			/* 0x930 */
	int64_t n_fast;			/* 0x938 */
	int64_t n_mid;			/* 0x940 */
	uint8_t run_slow, run_fast, run_mid, r94b; /* 0x948 */
	int32_t usb_bias;		/* 0x94c */
	int32_t n_period_clamp;		/* 0x950 */
	int32_t n_cycle_hi;		/* 0x954 */
	int32_t n_cycle_lo;		/* 0x958 */
	int32_t n_period_clamp1;	/* 0x95c last-stage clamps (psvr2_sense_output_clamp) */
	int32_t n_cycle_hi1;		/* 0x960 */
	int32_t n_cycle_lo1;		/* 0x964 */
	uint8_t max_period;		/* 0x968 */
	uint8_t r969[3];
	uint32_t max_cycle;		/* 0x96c */
	uint32_t min_cycle;		/* 0x970 */
	uint8_t max_period1;		/* 0x974 */
	uint8_t r975[3];
	uint32_t max_cycle1;		/* 0x978 */
	uint32_t min_cycle1;		/* 0x97c */
} psvr2_sync_state;
_Static_assert(sizeof(psvr2_sync_state) == 0x980, "sync state size");
_Static_assert(offsetof(psvr2_sync_state, best_rtt) == 0x920, "sync layout");
_Static_assert(offsetof(psvr2_sync_state, max_period) == 0x968, "sync layout");

/* Shared by all pads: per transport (0 USB, 1 BT). */
typedef struct {
	uint32_t rtt_max[2];
	int32_t latency[2];
} psvr2_sync_globals;
extern psvr2_sync_globals psvr2_sync_g;
extern uint8_t psvr2_sync_fit_enabled;
extern uint64_t (*psvr2_sync_headset_time)(void); /* NULL: none */

void psvr2_sync_init(psvr2_sync_state *st, uint8_t bt, uint8_t hand);
void psvr2_sync_stats_reset(psvr2_sync_state *st);
void psvr2_sync_update(psvr2_sync_state *st, uint64_t host_us, uint32_t echo,
		       uint32_t ticks, uint32_t ticks2, uint8_t bt);
uint32_t psvr2_sync_ticks_to_us(uint32_t ticks);
int32_t psvr2_sync_ctrl_to_host(const psvr2_sync_state *st, uint32_t ctrl_us, int64_t *host);
int32_t psvr2_sync_host_to_ctrl(const psvr2_sync_state *st, int64_t host, int32_t *ctrl);
int32_t psvr2_sync_clamp_base(int32_t half, int32_t v);
uint32_t psvr2_sync_clamp_cycle(psvr2_sync_state *st, uint32_t ns);
uint8_t psvr2_sync_clamp_period(psvr2_sync_state *st, uint8_t id);

/* ---- LED sync record (pad +0x33e0; doc: "LED-sync record") ---- */
typedef struct {
	uint8_t phase;		/* 0x00 */
	uint8_t seq;		/* 0x01 */
	uint8_t period_id;	/* 0x02 */
	uint8_t r03;
	int32_t base_time;	/* 0x04 */
	uint32_t cycle_ns;	/* 0x08 */
	uint8_t led[4];		/* 0x0c */
	uint64_t last_resync;	/* 0x10 */
	uint8_t sync_mode;	/* 0x18 */
	uint8_t r19[3];
	int32_t width;		/* 0x1c */
	int32_t step;		/* 0x20 */
	int32_t half;		/* 0x24 */
	uint8_t probes;		/* 0x28 */
	uint8_t r29[7];
} psvr2_ledsync;
_Static_assert(sizeof(psvr2_ledsync) == 0x30, "ledsync size");

void psvr2_ledsync_set_mode(psvr2_ledsync *ls, uint8_t mode);
void psvr2_ledsync_command(psvr2_sync_state *st, psvr2_ledsync *ls,
			   const uint8_t *cmd, uint8_t len);

/* ---- Sense input report (doc: "Sense input report") ---- */
typedef struct {
	uint64_t host_us;	/* 0x00 receive time */
	uint64_t ctrl_host_us;	/* 0x08 sample time in host clock */
	uint8_t counter;	/* 0x10 */
	uint8_t battery;	/* 0x11 */
	uint8_t status[2];	/* 0x12 DualSense status bytes */
	uint32_t buttons;	/* 0x14 */
	uint8_t lx, ly, rx, ry;	/* 0x18 */
	uint8_t l2, r2;		/* 0x1c */
	uint8_t r1e[2];
	uint32_t imu_ticks;	/* 0x20 */
	uint8_t temperature;	/* 0x24 */
	uint8_t r25;
	int16_t gyro[3];	/* 0x26 */
	int16_t accel[3];	/* 0x2c */
	uint8_t touch_counter;	/* 0x32 DualSense touchpad */
	uint8_t touch_id[2];	/* 0x33 bit 7 set: finger up */
	uint8_t r35;
	uint16_t touch_xy[2][2]; /* 0x36 */
	uint8_t trig_state_r;	/* 0x3e */
	uint8_t trig_state_l;	/* 0x3f */
	uint8_t trig_mode;	/* 0x40 */
	uint8_t r41[3];
	uint32_t edge_buttons;	/* 0x44 DualSense Edge only */
	uint8_t edge_axes[12];	/* 0x48 */
	uint8_t edge_profile;	/* 0x54 */
	uint8_t edge_b31_7;	/* 0x55 */
	uint8_t edge_b31_4;	/* 0x56 3 bits */
	uint8_t edge_b31_3;	/* 0x57 */
	uint8_t edge_b31_2;	/* 0x58 */
	uint8_t r59[2];
	uint8_t edge_b34_1;	/* 0x5b 2 bits */
	uint8_t left;		/* 0x5c */
	uint8_t touch;		/* 0x5d */
	uint8_t trig_prox;	/* 0x5e */
	uint8_t grip_prox;	/* 0x5f */
	uint32_t ctrl_ticks;	/* 0x60 */
	uint32_t echo;		/* 0x64 */
	uint8_t flag0;		/* 0x68 */
	uint8_t flag3;		/* 0x69 */
	uint8_t r6a[6];
} psvr2_sense_input;
_Static_assert(sizeof(psvr2_sense_input) == 0x70, "sense input size");

/* ---- Sense output report (doc: "Sense output report") ---- */
typedef struct {
	uint8_t conn;		/* pad +0x06 connection kind */
	uint8_t vol_haptic;	/* pad +0x0a */
	uint8_t vol_trigger;	/* pad +0x0b */
	uint8_t hand;		/* pad +0x54, 1 = left */
	uint16_t fw;		/* pad +0x50 */
	int32_t link;		/* pad +0x68 */
	uint32_t dev_fw;	/* device firmware (device slot +0x1d0) */
	uint8_t g_rumble_a, g_rumble_b;	/* library-wide switches */
	uint8_t g_flag2_10, g_flag2_02;
	/* Optional capability probe (Set 0x80 / Get 0x81); returns the sync
	 * mode to use (2 on success, 0 otherwise). NULL: probe fails. */
	uint8_t (*probe)(void *ctx);
	void *probe_ctx;
} psvr2_sense_out_ctx;

void psvr2_ledsync_resync(const psvr2_sync_state *st, psvr2_ledsync *ls, uint64_t now);
void psvr2_sense_output_fill(const psvr2_sense_out_ctx *c, const psvr2_sync_state *st,
			     psvr2_ledsync *ls, uint8_t *buf, uint64_t *last_out,
			     uint64_t now, uint32_t stamp, uint64_t resync_now);

/* Last check of the LED-sync fields just before a Sense report is sent. */
void psvr2_sense_output_clamp(uint8_t *buf, psvr2_sync_state *st);

/* Wire framing of an output buffer for the HID write (doc "Output
 * framing"). Returns the byte count to write in *towrite and the value
 * the writer returns on success, or an error code (negative). */
enum { PSVR2_DEV_DS4 = 0, PSVR2_DEV_DUALSENSE = 1, PSVR2_DEV_SENSE = 2 };
int32_t psvr2_pad_frame_output(int kind, int edge, uint16_t report_len, const uint8_t *buf,
			       uint16_t len, uint8_t *dst, uint32_t *towrite);

/* Vibration amplitude into an output buffer. fresh: buffer was just
 * zeroed and is sent now (else it waits behind the one in flight). */
void psvr2_sense_set_vibration(uint8_t *buf, int fresh, uint8_t conn,
			       uint8_t g_rumble_a, uint8_t g_rumble_b, uint8_t amplitude);

/* Trigger-effect strength change (old -> level, pad +0x0b). effect: the
 * stored 11-byte effect (mode + 10 params) of this controller's hand, or
 * NULL when the hand is unknown. */
void psvr2_sense_set_trigger_level(uint8_t *buf, int fresh, uint8_t old, uint8_t level,
				   uint8_t vol_haptic, const uint8_t *effect);

/* Set trigger effect from a wire-format block: mask (bit 0 L2, bit 1 R2),
 * L2 mode + 10 params, R2 mode + 10 params. Only VR left/right matter. */
void psvr2_sense_set_trigger_effect(uint8_t *buf, int is_left, int is_right,
				    const uint8_t wire[23]);

/* Full-state output (sent on (re)connect): trigger effect of this hand,
 * status LED on, vibration-off and global bits in byte 4. effect: the
 * stored 11-byte block of the hand. */
void psvr2_sense_fill_state(uint8_t *buf, uint8_t trigger_level, uint8_t vib_level,
			    const uint8_t effect[11], uint8_t g_enable, uint8_t g_value);

void psvr2_sense_input_decode(uint8_t hand, const uint8_t *report, uint8_t bt,
			      psvr2_sense_input *out);
void psvr2_pad_trigger_levels(const uint8_t *in, uint8_t out[10]);

/* scePad trigger-effect parameter block: mask byte, then two commands of
 * 0x38 bytes {u32 mode; u32 pad; u8 data[48]} for L2 (0) and R2 (1). */
enum {
	PSVR2_TRIG_OFF = 0, PSVR2_TRIG_FEEDBACK = 1, PSVR2_TRIG_WEAPON = 2,
	PSVR2_TRIG_VIBRATION = 3, PSVR2_TRIG_MULTI_FEEDBACK = 4,
	PSVR2_TRIG_SLOPE_FEEDBACK = 5, PSVR2_TRIG_MULTI_VIBRATION = 6,
};
/* Estimate the trigger-effect state from the trigger value (DualSense
 * path; VR controllers report it in input r[0x28]). */
uint8_t psvr2_pad_trigger_state(uint8_t wire_mode, const uint8_t *wire, uint8_t value,
				uint8_t prev);
int32_t psvr2_pad_trigger_effect_decode(uint8_t wire_mode, const uint8_t *wire,
					uint8_t *param, uint8_t slot);

/* ---- Motion: gyro still detector, IMU conversion, orientation (doc: "Motion") ---- */
typedef struct {
	int32_t enabled;	/* 0x00 */
	float range[3];		/* 0x04 raw LSB per deg/s */
	float bias[3];		/* 0x10 */
	int32_t flags;		/* 0x1c bit 0 fast, bit 1 slow */
	float slow_max[3];	/* 0x20 */
	float slow_thr[3];	/* 0x2c */
	float k_slow;		/* 0x38 */
	int32_t acc_sum[3];	/* 0x3c */
	uint32_t acc_n;		/* 0x48 */
	uint32_t windows;	/* 0x4c */
	float fast_max[3];	/* 0x50 */
	float fast_thr[3];	/* 0x5c */
	float k_fast;		/* 0x68 */
	int32_t max[3];		/* 0x6c */
	int32_t min[3];		/* 0x78 */
	int32_t sum[3];		/* 0x84 */
	uint16_t win_len;	/* 0x90 */
	uint16_t count;		/* 0x92 */
} psvr2_gyro_still;
_Static_assert(sizeof(psvr2_gyro_still) == 0x94, "still size");

void psvr2_gyro_still_reset(psvr2_gyro_still *s);
void psvr2_gyro_still_thresholds(psvr2_gyro_still *s);
void psvr2_gyro_still_init_ps5(psvr2_gyro_still *s, const int16_t c[42]);
void psvr2_gyro_still_init_ds4(psvr2_gyro_still *s, const int16_t c[8]);
int psvr2_gyro_still_update(psvr2_gyro_still *s, int16_t x, int16_t y, int16_t z,
			    float bias[3], float corr[3]);

/* Pad motion block, pad +0x6c .. +0x154. */
typedef struct {
	uint8_t flags;		/* +0x6c bit 0 auto bias, 1 deadband, 2 tilt correction */
	uint8_t r1[3];
	uint8_t calibrated;	/* +0x70 */
	uint8_t r5[3];
	float gyro_bias[3];	/* +0x74 */
	float gyro_scale[3];	/* +0x80 */
	float accel_off[3];	/* +0x8c */
	float accel_scale[3];	/* +0x98 */
	float tick_scale;	/* +0xa4 ticks -> us */
	uint32_t axis;		/* +0xa8 bit 0 rotate accel, bit 1 rotate gyro */
	psvr2_gyro_still still;	/* +0xac */
	float q[4];		/* +0x140 x y z w */
	uint32_t count;		/* +0x150 */
} psvr2_motion;
_Static_assert(sizeof(psvr2_motion) == 0xe8, "motion size");

/* Motion fields of a pad sample record (0xa0 bytes, pad +0x178 ring). */
typedef struct {
	uint32_t ts;		/* +0x58 */
	float gyro[3];		/* +0x5c rad/s */
	float accel[3];		/* +0x68 g */
	uint8_t temperature;	/* +0x74 */
	uint8_t r75[3];
	float q[4];		/* +0x78 */
} psvr2_motion_out;

extern float psvr2_imu_axes[12];	/* 3x3, rows of 4 floats */
void psvr2_mat3_inverse(float out[12], const float m[12]);
void psvr2_tilt_correct(float q[4], const float a[3], float dt);
int32_t psvr2_motion_cal_ds(psvr2_motion *m, uint16_t vid, uint16_t pid, int32_t hid_ver,
			    int16_t cal[18]);
void psvr2_motion_update(psvr2_motion *m, psvr2_motion_out *o, const psvr2_motion_out *prev,
			 int is_dualsense, const psvr2_sense_input *in);
/* ---- Pad slot and sample ring (doc: "Sample ring") ---- */
#define PSVR2_PAD_RING 64

/* One input sample as kept in the pad's ring. */
typedef struct {
	uint64_t host_us;	/* 0x00 receive time */
	uint64_t ctrl_host_us;	/* 0x08 sample time in host clock */
	uint8_t r10[4];
	uint32_t buttons;	/* 0x14 */
	uint8_t lx, ly, rx, ry;	/* 0x18 */
	uint8_t l2, r2;		/* 0x1c */
	uint8_t counter;	/* 0x1e */
	uint8_t touch_counter;	/* 0x1f */
	uint8_t r20;
	uint8_t touch_n;	/* 0x21 fingers down */
	struct {
		uint8_t id, r;
		uint16_t x, y;
	} touch_pt[2];		/* 0x22 */
	uint8_t r2e[0x2a];
	psvr2_motion_out motion; /* 0x58 */
	uint8_t battery;	/* 0x88 */
	uint8_t status[2];	/* 0x89 */
	uint8_t trig_state[2];	/* 0x8b pairs with l2 / r2 */
	uint8_t trig_mode;	/* 0x8d 0xf0 when estimated on the host */
	uint8_t r8e[2];
	uint8_t touch;		/* 0x90 */
	uint8_t trig_prox;	/* 0x91 */
	uint8_t grip_prox;	/* 0x92 */
	uint8_t flag0;		/* 0x93 */
	uint8_t flag3;		/* 0x94 */
	uint8_t r95[0x0b];
} psvr2_pad_sample;
_Static_assert(sizeof(psvr2_pad_sample) == 0xa0, "pad sample size");

/* The parts of a pad slot (0x3410 bytes) this library touches. */
typedef struct {
	uint8_t in_use;		/* 0x0000 */
	uint8_t r0001;
	uint16_t vid;		/* 0x0002 */
	uint16_t pid;		/* 0x0004 */
	uint8_t conn;		/* 0x0006 */
	uint8_t r0007;
	uint8_t out_flags;	/* 0x0008 see "Sample ring" */
	uint8_t mic_led;	/* 0x0009 0 off, 1 on, 2 pulse */
	uint8_t vib_level;	/* 0x000a */
	uint8_t trig_level;	/* 0x000b non-zero: controller reports trigger state */
	uint8_t light_dim;	/* 0x000c lightbar brightness: 0 full, 1 half, else low */
	uint8_t est_trig[2];	/* 0x000d DualSense: estimate trigger state on the host */
	uint8_t r000f;
	int32_t out_handle;	/* 0x0010 */
	int32_t player;		/* 0x0014 player LEDs 1..4 (0: DS4 lightbar unset) */
	uint8_t r0018[4];
	int32_t handle;		/* 0x001c */
	int32_t hid_ver;	/* 0x0020 */
	uint8_t r0024[0x2c];
	uint16_t fw;		/* 0x0050 */
	uint8_t r0052[2];
	uint8_t hand;		/* 0x0054 */
	uint8_t r0055[0x13];
	int32_t link;		/* 0x0068 connection type */
	psvr2_motion motion;	/* 0x006c */
	uint8_t r0154[0x0c];
	int32_t ring_n;		/* 0x0160 entries filled, up to 64 */
	int32_t ring_oldest;	/* 0x0164 */
	int32_t ring_idx;	/* 0x0168 next entry to write */
	uint8_t r016c[4];
	psvr2_pad_sample *last;	/* 0x0170 */
	psvr2_pad_sample ring[PSVR2_PAD_RING]; /* 0x0178 */
	uint8_t r2978[0x98];
	uint8_t ds4_rgb[3];	/* 0x2a10 */
	uint8_t r2a13;
	uint8_t ds_rgb[3];	/* 0x2a14 */
	uint8_t r2a17;
	uint8_t eff_mask;	/* 0x2a18 effect block: mask, slot 0, slot 1 */
	uint8_t eff_mode0;	/* 0x2a19 effect slot 0: wire mode, then 10 bytes */
	uint8_t eff0[10];	/* 0x2a1a */
	uint8_t eff_mode1;	/* 0x2a24 effect slot 1 */
	uint8_t eff1[10];	/* 0x2a25 */
	uint8_t r2a2f[0x31];
	psvr2_sync_state sync;	/* 0x2a60 */
	psvr2_ledsync ledsync;	/* 0x33e0 */
} psvr2_pad;
_Static_assert(sizeof(psvr2_pad) == 0x3410, "pad size");
_Static_assert(offsetof(psvr2_pad, eff_mode1) == 0x2a24, "pad layout");
_Static_assert(offsetof(psvr2_pad, eff_mask) == 0x2a18, "pad layout");
_Static_assert(offsetof(psvr2_pad, sync) == 0x2a60, "pad layout");

uint32_t psvr2_pad_read_bits(const uint8_t *p, int bit, int n, int sign);
void psvr2_ds_input_decode(const uint8_t *report, uint8_t off, int edge, psvr2_sense_input *o);
void psvr2_pad_imu_cal_parse(int16_t out[18], const uint8_t *rep, int grouped);
void psvr2_pad_push_sample(psvr2_pad *p, psvr2_sense_input *in, uint8_t bt);
void psvr2_pad_lightbar_scale(uint8_t *r, uint8_t *g, uint8_t *b, int bright, int no_floor);
int32_t psvr2_pad_effect_unsupported(const uint8_t *blk, uint8_t *slot0, uint8_t *slot1);
void psvr2_ds_output_fill(psvr2_pad *p, uint8_t *buf);

/* Process-wide output switches (Sony globals; see doc "Global state"). */
typedef struct {
	uint8_t haptics_b;	/* 0x1805b1573 */
	uint8_t flag02;		/* 0x1805e349c */
	uint8_t flag10;		/* 0x1805b157b */
	uint8_t state_en;	/* 0x1805e349d */
	uint8_t state_val;	/* low byte of 0x1805e3498 */
} psvr2_pad_switches;

/* Reports sent when a pad slot is opened; returns how many (0..2). */
int psvr2_pad_open_reports(psvr2_pad *p, const psvr2_pad_switches *sw,
			   uint8_t rep[2][0x30], uint8_t len[2]);
void psvr2_ds4_output_fill(const psvr2_pad *p, uint8_t *buf);
int psvr2_pad_push_sample_ds(psvr2_pad *p, const psvr2_sense_input *in);
#endif
