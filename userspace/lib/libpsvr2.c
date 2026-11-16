// SPDX-License-Identifier: GPL-2.0
/*
 * libpsvr2 — userspace access to the PSVR2 kernel module's device nodes.
 *
 * Copyright (C) 2026 PSVR2 Linux project
 */
#include <dirent.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/input.h>

#include "libpsvr2.h"
#include "psvr2_uapi.h"		/* kernel wire structs (-I kernel/) */

#define POSE_DEV "/dev/psvr2-pose"
#define GAZE_DEV "/dev/psvr2-gaze"
#define IIO_DIR  "/sys/bus/iio/devices"
#define V4L_DIR  "/sys/class/video4linux"
#define DRV_DIR  "/sys/bus/usb/drivers/psvr2"
#define INPUT_NAME "PlayStation VR2 Headset Controls"

struct psvr2 {
	int	pose_fd;
	int	gaze_fd;
	char	imu_dir[300];		/* IIO device dir, "" if none */
	double	accel_scale;
	double	gyro_scale;
	char	cam_path[300];		/* "/dev/videoN", "" if none */
	int	controls_fd;
	char	sysfs_dir[300];		/* a bound interface dir, "" if none */
};

/* ---- small sysfs helpers ------------------------------------------------ */

static int read_file_str(const char *path, char *buf, size_t len)
{
	int fd = open(path, O_RDONLY);
	ssize_t n;

	if (fd < 0)
		return -1;
	n = read(fd, buf, len - 1);
	close(fd);
	if (n < 0)
		return -1;
	buf[n] = '\0';
	while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == ' '))
		buf[--n] = '\0';
	return 0;
}

static int name_matches(const char *dir, const char *entry, const char *want)
{
	char path[512], name[128];

	snprintf(path, sizeof(path), "%s/%s/name", dir, entry);
	if (read_file_str(path, name, sizeof(name)))
		return 0;
	return strcmp(name, want) == 0;
}

static float f_from_le(uint32_t le)
{
	uint32_t host = le32toh(le);
	float f;

	memcpy(&f, &host, sizeof(f));
	return f;
}

/* ---- discovery ---------------------------------------------------------- */

static void find_imu(struct psvr2 *p)
{
	DIR *d = opendir(IIO_DIR);
	struct dirent *e;

	if (!d)
		return;
	while ((e = readdir(d))) {
		char path[600], val[64];

		if (strncmp(e->d_name, "iio:device", 10))
			continue;
		if (!name_matches(IIO_DIR, e->d_name, "psvr2_imu"))
			continue;
		snprintf(p->imu_dir, sizeof(p->imu_dir), "%s/%s", IIO_DIR,
			 e->d_name);
		snprintf(path, sizeof(path), "%s/in_accel_scale", p->imu_dir);
		if (!read_file_str(path, val, sizeof(val)))
			p->accel_scale = strtod(val, NULL);
		snprintf(path, sizeof(path), "%s/in_anglvel_scale", p->imu_dir);
		if (!read_file_str(path, val, sizeof(val)))
			p->gyro_scale = strtod(val, NULL);
		break;
	}
	closedir(d);
}

static void find_camera(struct psvr2 *p)
{
	DIR *d = opendir(V4L_DIR);
	struct dirent *e;

	if (!d)
		return;
	while ((e = readdir(d))) {
		if (strncmp(e->d_name, "video", 5))
			continue;
		if (!name_matches(V4L_DIR, e->d_name, "PlayStation VR2 Cameras"))
			continue;
		snprintf(p->cam_path, sizeof(p->cam_path), "/dev/%s", e->d_name);
		break;
	}
	closedir(d);
}

/* Every bound interface carries the same device attributes; pick the first. */
static void find_sysfs(struct psvr2 *p)
{
	DIR *d = opendir(DRV_DIR);
	struct dirent *e;

	if (!d)
		return;
	while ((e = readdir(d))) {
		char path[400];

		if (e->d_name[0] == '.' || strlen(e->d_name) > 64)
			continue;
		snprintf(path, sizeof(path), "%s/%s/brightness", DRV_DIR,
			 e->d_name);
		if (access(path, F_OK) == 0) {
			snprintf(p->sysfs_dir, sizeof(p->sysfs_dir), "%s/%s",
				 DRV_DIR, e->d_name);
			break;
		}
	}
	closedir(d);
}

static int write_attr_int(const struct psvr2 *p, const char *attr, int value)
{
	char path[400], buf[16];
	int fd, n;

	if (!p || !p->sysfs_dir[0])
		return -1;
	snprintf(path, sizeof(path), "%s/%s", p->sysfs_dir, attr);
	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -1;
	n = snprintf(buf, sizeof(buf), "%d\n", value);
	n = (write(fd, buf, n) == n) ? 0 : -1;
	close(fd);
	return n;
}

static void find_controls(struct psvr2 *p)
{
	DIR *d = opendir("/dev/input");
	struct dirent *e;

	p->controls_fd = -1;
	if (!d)
		return;
	while ((e = readdir(d))) {
		char path[300], name[128] = "";
		int fd;

		if (strncmp(e->d_name, "event", 5))
			continue;
		snprintf(path, sizeof(path), "/dev/input/%s", e->d_name);
		fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0)
			continue;
		if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 &&
		    !strcmp(name, INPUT_NAME)) {
			p->controls_fd = fd;
			break;
		}
		close(fd);
	}
	closedir(d);
}

/* ---- lifecycle ---------------------------------------------------------- */

psvr2_t *psvr2_open(void)
{
	struct psvr2 *p = calloc(1, sizeof(*p));

	if (!p)
		return NULL;

	p->pose_fd = open(POSE_DEV, O_RDONLY | O_NONBLOCK);
	p->gaze_fd = open(GAZE_DEV, O_RDONLY | O_NONBLOCK);
	find_imu(p);
	find_camera(p);
	find_sysfs(p);
	find_controls(p);
	return p;
}

void psvr2_close(psvr2_t *p)
{
	if (!p)
		return;
	if (p->pose_fd >= 0)
		close(p->pose_fd);
	if (p->gaze_fd >= 0)
		close(p->gaze_fd);
	if (p->controls_fd >= 0)
		close(p->controls_fd);
	free(p);
}

int psvr2_has_pose(const psvr2_t *p) { return p && p->pose_fd >= 0; }
int psvr2_has_gaze(const psvr2_t *p) { return p && p->gaze_fd >= 0; }
int psvr2_has_imu(const psvr2_t *p) { return p && p->imu_dir[0] != '\0'; }

const char *psvr2_camera_path(const psvr2_t *p)
{
	return (p && p->cam_path[0]) ? p->cam_path : NULL;
}

int psvr2_pose_fd(const psvr2_t *p) { return p ? p->pose_fd : -1; }
int psvr2_gaze_fd(const psvr2_t *p) { return p ? p->gaze_fd : -1; }

/* ---- streams ------------------------------------------------------------ */

static int read_sample(int fd, void *buf, size_t sz, int block)
{
	ssize_t n;

	if (fd < 0)
		return -1;
	if (block) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN };

		if (poll(&pfd, 1, -1) < 0)
			return -1;
	}
	n = read(fd, buf, sz);
	if (n == (ssize_t)sz)
		return 1;
	if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		return 0;
	return -1;
}

int psvr2_read_pose(psvr2_t *p, struct psvr2_pose *out, int block)
{
	struct psvr2_pose_sample s;
	int r;

	if (!p || !out)
		return -1;
	r = read_sample(p->pose_fd, &s, sizeof(s), block);
	if (r != 1)
		return r;

	out->timestamp_ns = s.timestamp_ns;
	out->device_vts_us = s.device_vts_us;
	out->valid = (s.flags & PSVR2_POSE_FLAG_VALID) != 0;
	for (int i = 0; i < 3; i++)
		out->position[i] = f_from_le(s.position[i]);
	for (int i = 0; i < 4; i++)
		out->orientation[i] = f_from_le(s.orientation[i]);
	return 1;
}

int psvr2_read_gaze(psvr2_t *p, struct psvr2_gaze *out, int block)
{
	struct psvr2_gaze_sample s;
	int r;

	if (!p || !out)
		return -1;
	r = read_sample(p->gaze_fd, &s, sizeof(s), block);
	if (r != 1)
		return r;

	out->timestamp_ns = s.timestamp_ns;
	out->device_timestamp_us = s.device_timestamp_us;
	out->valid = (s.flags & PSVR2_GAZE_FLAG_VALID) != 0;

#define COPY_EYE(dst, src)						\
	do {								\
		(dst).gaze_point_valid = (src).gaze_point_valid;	\
		(dst).gaze_direction_valid = (src).gaze_direction_valid;\
		(dst).pupil_diameter_valid = (src).pupil_diameter_valid;\
		(dst).pupil_diameter_mm =				\
			f_from_le((src).pupil_diameter_mm);		\
		(dst).blink_valid = (src).blink_valid;			\
		(dst).blink = (src).blink;				\
		for (int i = 0; i < 3; i++) {				\
			(dst).gaze_point_mm[i] =			\
				f_from_le((src).gaze_point_mm[i]);	\
			(dst).gaze_direction[i] =			\
				f_from_le((src).gaze_direction[i]);	\
		}							\
	} while (0)

	COPY_EYE(out->left, s.left);
	COPY_EYE(out->right, s.right);
#undef COPY_EYE

	out->combined.gaze_point_valid = s.combined.gaze_point_valid;
	out->combined.gaze_direction_valid = s.combined.gaze_direction_valid;
	for (int i = 0; i < 3; i++) {
		out->combined.gaze_point_mm[i] =
			f_from_le(s.combined.gaze_point_mm[i]);
		out->combined.gaze_direction[i] =
			f_from_le(s.combined.gaze_direction[i]);
	}
	return 1;
}

int psvr2_read_imu(psvr2_t *p, struct psvr2_imu *out)
{
	static const char *accel[3] = {
		"in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw" };
	static const char *gyro[3] = {
		"in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw" };

	if (!p || !out || !p->imu_dir[0])
		return -1;

	for (int i = 0; i < 3; i++) {
		char path[400], val[32];

		snprintf(path, sizeof(path), "%s/%s", p->imu_dir, accel[i]);
		if (read_file_str(path, val, sizeof(val)))
			return -1;
		out->accel_m_s2[i] = strtol(val, NULL, 10) * p->accel_scale;

		snprintf(path, sizeof(path), "%s/%s", p->imu_dir, gyro[i]);
		if (read_file_str(path, val, sizeof(val)))
			return -1;
		out->gyro_rad_s[i] = strtol(val, NULL, 10) * p->gyro_scale;
	}
	return 0;
}

int psvr2_read_controls(psvr2_t *p, struct psvr2_controls *out)
{
	struct input_absinfo abs;
	unsigned long sw[(SW_MAX + 1 + 8 * sizeof(long) - 1) / (8 * sizeof(long))] = {0};
	unsigned long key[(KEY_MAX + 1 + 8 * sizeof(long) - 1) / (8 * sizeof(long))] = {0};
	const size_t bits = 8 * sizeof(long);

	if (!p || p->controls_fd < 0)
		return -1;
	if (ioctl(p->controls_fd, EVIOCGABS(ABS_MISC), &abs) < 0 ||
	    ioctl(p->controls_fd, EVIOCGSW(sizeof(sw)), sw) < 0 ||
	    ioctl(p->controls_fd, EVIOCGKEY(sizeof(key)), key) < 0)
		return -1;
	out->ipd_mm = abs.value;
	out->worn = !!(sw[SW_FRONT_PROXIMITY / bits] & (1UL << (SW_FRONT_PROXIMITY % bits)));
	out->function_button = !!(key[BTN_MODE / bits] & (1UL << (BTN_MODE % bits)));
	return 0;
}

int psvr2_set_brightness(psvr2_t *p, int level)
{
	if (level < 0)
		level = 0;
	if (level > 31)
		level = 31;
	return write_attr_int(p, "brightness", level);
}

int psvr2_set_rumble(psvr2_t *p, int hz)
{
	if (hz < 0)
		hz = 0;
	if (hz > 25)
		hz = 25;
	return write_attr_int(p, "rumble", hz);
}

/*
 * Lens calibration -> distortion parameters. The block layout and the
 * conversion are from Monado's psvr2_setup_distortion_and_fovs (BSL-1.0):
 * u8 version, 7 unknown bytes, then 32 float32 parameters.
 */
#define LENS_CAL_LEN 136

int psvr2_get_lens_distortion(psvr2_t *p, float out[8])
{
	char path[400], hex[2 * LENS_CAL_LEN + 8];
	uint8_t raw[LENS_CAL_LEN];
	float prm[32];
	int i;

	memset(out, 0, 8 * sizeof(float));
	out[0] = -0.09919293f;	/* firmware without a calibration block */
	out[2] = 0.09919293f;
	out[4] = out[6] = 1.0f;

	if (!p || !p->sysfs_dir[0])
		return -1;
	snprintf(path, sizeof(path), "%s/lens_calibration", p->sysfs_dir);
	if (read_file_str(path, hex, sizeof(hex)) ||
	    strlen(hex) != 2 * LENS_CAL_LEN)
		return -1;
	for (i = 0; i < LENS_CAL_LEN; i++) {
		unsigned int b;

		if (sscanf(hex + 2 * i, "%2x", &b) != 1)
			return -1;
		raw[i] = (uint8_t)b;
	}
	if (raw[0] < 4)
		return 1;

	for (i = 0; i < 32; i++) {
		uint32_t le;

		memcpy(&le, raw + 8 + 4 * i, 4);
		prm[i] = f_from_le(le);
	}
	out[0] = (((-prm[0] - prm[6]) * 29.9f + 14.95f) / 1000.0f - 3.22f) / 32.46199f;
	out[1] = (((-prm[1] * 29.9f) + 14.95f) / 1000.0f) / 32.46199f;
	out[2] = (((prm[6] - prm[2]) * 29.9f + 14.95f) / 1000.0f + 3.22f) / 32.46199f;
	out[3] = (((-prm[3] * 29.9f) + 14.95f) / 1000.0f) / 32.46199f;
	out[4] = cosf(-prm[4] * (float)M_PI / 180.0f);
	out[5] = sinf(-prm[4] * (float)M_PI / 180.0f);
	out[6] = cosf(-prm[5] * (float)M_PI / 180.0f);
	out[7] = sinf(-prm[5] * (float)M_PI / 180.0f);
	return 0;
}
