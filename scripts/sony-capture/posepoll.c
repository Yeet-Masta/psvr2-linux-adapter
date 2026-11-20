// Records SteamVR device poses with CLOCK_MONOTONIC time (same clock as
// bpftrace nsecs), full rotation matrices and angular velocity. vrcmd
// --pollposes prints only Euler angles with no time, which is ambiguous.
// No OpenVR client header here, so this uses the C function table of
// IVRSystem_022 by index:
//   11 GetDeviceToAbsoluteTrackingPose, 17 GetTrackedDeviceIndexForControllerRole.
// usage: posepoll seconds > out.txt
// out: R left right (device indices), then per pose:
//   t_ns dev valid result m00 m01 m02 m03 m10 .. m23 vx vy vz wx wy wz
#include <dlfcn.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct { float m[3][4]; } Mat34;
typedef struct { float v[3]; } Vec3;
typedef struct {
	Mat34 mat;
	Vec3 vel, avel;
	int result;
	bool valid, connected;
} Pose;
typedef void (*GetPoses)(int origin, float predict, Pose *poses, uint32_t n);
typedef uint32_t (*RoleIndex)(int role);

static uint64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

int main(int argc, char **argv)
{
	double secs = argc > 1 ? atof(argv[1]) : 120;
	void *lib = dlopen("libopenvr_api.so", RTLD_NOW);
	if (!lib) { fprintf(stderr, "%s\n", dlerror()); return 1; }
	intptr_t (*init)(int *, int, const char *) = dlsym(lib, "VR_InitInternal2");
	void *(*iface)(const char *, int *) = dlsym(lib, "VR_GetGenericInterface");
	void (*shutdown)(void) = dlsym(lib, "VR_ShutdownInternal");
	int err = 0;
	init(&err, 3 /* VRApplication_Background */, NULL);
	if (err) { fprintf(stderr, "VR_InitInternal2 error %d\n", err); return 1; }
	void **fn = iface("FnTable:IVRSystem_022", &err);
	if (!fn || err) { fprintf(stderr, "IVRSystem_022 error %d\n", err); return 1; }
	GetPoses get = (GetPoses)fn[11];
	RoleIndex role = (RoleIndex)fn[17];
	printf("R %u %u\n", role(1), role(2));

	enum { N = 16 };
	Pose p[N], last[N];
	memset(last, 0, sizeof(last));
	uint64_t end = now_ns() + (uint64_t)(secs * 1e9);
	struct timespec nap = { 0, 1000000 };
	while (now_ns() < end) {
		get(1 /* Standing */, 0, p, N);
		uint64_t t = now_ns();
		for (int d = 0; d < N; d++) {
			if (!p[d].connected || !memcmp(&p[d].mat, &last[d].mat, sizeof(Mat34)))
				continue;
			last[d] = p[d];
			printf("%llu %d %d %d", (unsigned long long)t, d, p[d].valid, p[d].result);
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 4; c++)
					printf(" %.6f", p[d].mat.m[r][c]);
			for (int k = 0; k < 3; k++) printf(" %.5f", p[d].vel.v[k]);
			for (int k = 0; k < 3; k++) printf(" %.5f", p[d].avel.v[k]);
			putchar('\n');
		}
		nanosleep(&nap, NULL);
	}
	shutdown();
	return 0;
}
