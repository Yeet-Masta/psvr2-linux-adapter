// SPDX-License-Identifier: GPL-2.0
//
// VK_LAYER_PSVR2_mode_order: a Vulkan implicit layer for SteamVR's
// vrcompositor on Linux.
//
// Why: vrcompositor picks the HMD display mode closest to the wanted rate
// from vkGetDisplayModePropertiesKHR. It writes the chosen rate into its
// frame timing only when the winner is NOT the first mode with the HMD's
// resolution; otherwise its 90.0 Hz default stays. The PSVR2's 90/120 Hz
// EDID lists 119.88 Hz first, so at 120 Hz SteamVR scans out 119.88 Hz but
// paces frames for 90 Hz ("Updated HMD Prop_DisplayFrequency_Float to
// 90.000000" in vrcompositor.txt), which judders and tears.
//
// What: inside vrcompositor only, this layer returns the modes of each
// resolution in ascending refresh order. The 90/120 EDID then reads
// 89.91, 119.88: at 120 Hz the winner is second and its rate is used; at
// 90 Hz the winner is first and the 90.0 default is within 0.1 %, as with
// the 90-only EDID. The mode handles move with their properties, so the
// selected index still names the right mode. Every other call passes
// straight through.
//
// It is an implicit layer that the loader enables only when
// _STEAMVR_LOGGING_READY=1, which SteamVR's own launch scripts export
// (vrstartup.sh, vrcompositor-launcher.sh); vrserver's environment does not
// reach the compositor, which is started through steam-runtime-launch-client
// with only LD_LIBRARY_PATH passed. The layer also checks the process name,
// so other SteamVR processes see no change. PSVR2_VK_MODE_ORDER_DISABLE=1
// turns it off.

// No Vulkan headers are needed; the few types used are declared here as the
// Vulkan and loader-layer ABIs define them.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#define VKAPI __attribute__( ( visibility( "default" ) ) )

typedef int32_t VkResult;
typedef void *VkInstance;
typedef void *VkPhysicalDevice;
typedef void *VkDevice;
typedef uint64_t VkDisplayKHR;
typedef uint64_t VkDisplayModeKHR;
typedef void ( *PFN_vkVoidFunction )( void );
typedef PFN_vkVoidFunction ( *PFN_vkGetInstanceProcAddr )( VkInstance, const char * );
typedef PFN_vkVoidFunction ( *PFN_vkGetDeviceProcAddr )( VkDevice, const char * );

enum { VK_SUCCESS = 0, VK_ERROR_INITIALIZATION_FAILED = -3 };
enum {
	VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO = 47,
	VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO = 48,
};
enum { VK_LAYER_LINK_INFO = 0 };

typedef struct {
	int32_t sType;
	const void *pNext;
} VkBaseInStructure;

typedef struct VkLayerInstanceLink {
	struct VkLayerInstanceLink *pNext;
	PFN_vkGetInstanceProcAddr pfnNextGetInstanceProcAddr;
	PFN_vkVoidFunction pfnNextGetPhysicalDeviceProcAddr;
} VkLayerInstanceLink;

typedef struct {
	int32_t sType;
	const void *pNext;
	int32_t function;
	union {
		VkLayerInstanceLink *pLayerInfo;
		void *other;
	} u;
} VkLayerInstanceCreateInfo;

typedef struct VkLayerDeviceLink {
	struct VkLayerDeviceLink *pNext;
	PFN_vkGetInstanceProcAddr pfnNextGetInstanceProcAddr;
	PFN_vkGetDeviceProcAddr pfnNextGetDeviceProcAddr;
} VkLayerDeviceLink;

typedef struct {
	int32_t sType;
	const void *pNext;
	int32_t function;
	union {
		VkLayerDeviceLink *pLayerInfo;
		void *other;
	} u;
} VkLayerDeviceCreateInfo;

typedef struct {
	VkDisplayModeKHR displayMode;
	uint32_t width, height; // visibleRegion
	uint32_t refreshRate;   // mHz
} VkDisplayModePropertiesKHR;

typedef struct {
	int32_t sType;
	void *pNext;
	VkDisplayModePropertiesKHR displayModeProperties;
} VkDisplayModeProperties2KHR;

typedef struct {
	uint32_t sType;
	uint32_t loaderLayerInterfaceVersion;
	PFN_vkGetInstanceProcAddr pfnGetInstanceProcAddr;
	PFN_vkGetDeviceProcAddr pfnGetDeviceProcAddr;
	PFN_vkVoidFunction pfnGetPhysicalDeviceProcAddr;
} VkNegotiateLayerInterface;

typedef VkResult ( *PFN_vkCreateInstance )( const void *, const void *, VkInstance * );
typedef void ( *PFN_vkDestroyInstance )( VkInstance, const void * );
typedef VkResult ( *PFN_vkCreateDevice )( VkPhysicalDevice, const void *, const void *, VkDevice * );
typedef VkResult ( *PFN_vkGetDisplayModePropertiesKHR )( VkPhysicalDevice, VkDisplayKHR, uint32_t *,
                                                         VkDisplayModePropertiesKHR * );
typedef VkResult ( *PFN_vkGetDisplayModeProperties2KHR )( VkPhysicalDevice, VkDisplayKHR, uint32_t *,
                                                          VkDisplayModeProperties2KHR * );

// One entry per instance, keyed by the loader dispatch pointer that the
// instance and its physical devices share.
struct inst {
	void *key;
	PFN_vkGetInstanceProcAddr gipa;
	PFN_vkDestroyInstance destroy;
	PFN_vkGetDisplayModePropertiesKHR modes;
	PFN_vkGetDisplayModeProperties2KHR modes2;
};

#define MAX_INST 16
static struct inst g_inst[MAX_INST];
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_active = -1; // reorder only inside vrcompositor

static void *Key( void *dispatchable ) { return *(void **)dispatchable; }

static struct inst *Find( void *key )
{
	struct inst *r = NULL;
	pthread_mutex_lock( &g_lock );
	for ( int i = 0; i < MAX_INST; i++ )
		if ( g_inst[i].key == key ) { r = &g_inst[i]; break; }
	pthread_mutex_unlock( &g_lock );
	return r;
}

static int Active( void )
{
	if ( g_active < 0 )
	{
		char comm[32] = { 0 };
		FILE *f = fopen( "/proc/self/comm", "r" );
		if ( f )
		{
			if ( !fgets( comm, sizeof( comm ), f ) ) comm[0] = 0;
			fclose( f );
		}
		g_active = strncmp( comm, "vrcompositor", 12 ) == 0;
		if ( getenv( "PSVR2_MODE_ORDER_ALL" ) ) g_active = 1; // for testing
	}
	return g_active;
}

// Ascending refresh within each resolution; entries of other resolutions keep
// their slots. n is tiny (< 50), so a plain O(n^2) pass is fine.
static void Order( VkDisplayModePropertiesKHR **p, uint32_t n )
{
	for ( uint32_t i = 0; i < n; i++ )
		for ( uint32_t j = i + 1; j < n; j++ )
			if ( p[i]->width == p[j]->width && p[i]->height == p[j]->height &&
			     p[i]->refreshRate > p[j]->refreshRate )
			{
				VkDisplayModePropertiesKHR t = *p[i];
				*p[i] = *p[j];
				*p[j] = t;
			}
}

static VkResult GetModes( VkPhysicalDevice pd, VkDisplayKHR d, uint32_t *count, VkDisplayModePropertiesKHR *props )
{
	struct inst *in = Find( Key( pd ) );
	if ( !in || !in->modes ) return VK_ERROR_INITIALIZATION_FAILED;
	VkResult r = in->modes( pd, d, count, props );
	if ( r >= 0 && props && Active() )
	{
		VkDisplayModePropertiesKHR *p[64];
		uint32_t n = *count < 64 ? *count : 64;
		for ( uint32_t i = 0; i < n; i++ ) p[i] = &props[i];
		Order( p, n );
	}
	return r;
}

static VkResult GetModes2( VkPhysicalDevice pd, VkDisplayKHR d, uint32_t *count, VkDisplayModeProperties2KHR *props )
{
	struct inst *in = Find( Key( pd ) );
	if ( !in || !in->modes2 ) return VK_ERROR_INITIALIZATION_FAILED;
	VkResult r = in->modes2( pd, d, count, props );
	if ( r >= 0 && props && Active() )
	{
		VkDisplayModePropertiesKHR *p[64];
		uint32_t n = *count < 64 ? *count : 64;
		for ( uint32_t i = 0; i < n; i++ ) p[i] = &props[i].displayModeProperties;
		Order( p, n );
	}
	return r;
}

static VkResult CreateInstance( const void *ci, const void *alloc, VkInstance *out )
{
	VkLayerInstanceCreateInfo *li = NULL;
	for ( const VkBaseInStructure *s = ( (const VkBaseInStructure *)ci )->pNext; s; s = s->pNext )
		if ( s->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO &&
		     ( (const VkLayerInstanceCreateInfo *)s )->function == VK_LAYER_LINK_INFO )
		{
			li = (VkLayerInstanceCreateInfo *)s;
			break;
		}
	if ( !li ) return VK_ERROR_INITIALIZATION_FAILED;

	PFN_vkGetInstanceProcAddr gipa = li->u.pLayerInfo->pfnNextGetInstanceProcAddr;
	li->u.pLayerInfo = li->u.pLayerInfo->pNext;
	PFN_vkCreateInstance create = (PFN_vkCreateInstance)gipa( NULL, "vkCreateInstance" );
	if ( !create ) return VK_ERROR_INITIALIZATION_FAILED;
	VkResult r = create( ci, alloc, out );
	if ( r != VK_SUCCESS ) return r;

	struct inst e = {
		.key = Key( *out ),
		.gipa = gipa,
		.destroy = (PFN_vkDestroyInstance)gipa( *out, "vkDestroyInstance" ),
		.modes = (PFN_vkGetDisplayModePropertiesKHR)gipa( *out, "vkGetDisplayModePropertiesKHR" ),
		.modes2 = (PFN_vkGetDisplayModeProperties2KHR)gipa( *out, "vkGetDisplayModeProperties2KHR" ),
	};
	pthread_mutex_lock( &g_lock );
	int slot = -1;
	for ( int i = 0; i < MAX_INST && slot < 0; i++ )
		if ( !g_inst[i].key || g_inst[i].key == e.key ) slot = i;
	if ( slot < 0 ) slot = 0; // table full: reuse; the oldest instance loses the intercept
	g_inst[slot] = e;
	pthread_mutex_unlock( &g_lock );
	return r;
}

static void DestroyInstance( VkInstance instance, const void *alloc )
{
	if ( !instance ) return;
	struct inst *in = Find( Key( instance ) );
	PFN_vkDestroyInstance destroy = in ? in->destroy : NULL;
	if ( in )
	{
		pthread_mutex_lock( &g_lock );
		memset( in, 0, sizeof( *in ) );
		pthread_mutex_unlock( &g_lock );
	}
	if ( destroy ) destroy( instance, alloc );
}

// Device chain: nothing is intercepted, but the loader wants a working
// vkGetDeviceProcAddr, so vkCreateDevice is passed down and GDPA forwards to
// the next one recorded for that device (it differs per driver).
struct dev {
	void *key;
	PFN_vkGetDeviceProcAddr gdpa;
};
static struct dev g_dev[MAX_INST];

static PFN_vkGetDeviceProcAddr DevNext( void *key )
{
	PFN_vkGetDeviceProcAddr r = NULL;
	pthread_mutex_lock( &g_lock );
	for ( int i = 0; i < MAX_INST; i++ )
		if ( g_dev[i].key == key ) { r = g_dev[i].gdpa; break; }
	pthread_mutex_unlock( &g_lock );
	return r;
}

static VkResult CreateDevice( VkPhysicalDevice pd, const void *ci, const void *alloc, VkDevice *out )
{
	VkLayerDeviceCreateInfo *li = NULL;
	for ( const VkBaseInStructure *s = ( (const VkBaseInStructure *)ci )->pNext; s; s = s->pNext )
		if ( s->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO &&
		     ( (const VkLayerDeviceCreateInfo *)s )->function == VK_LAYER_LINK_INFO )
		{
			li = (VkLayerDeviceCreateInfo *)s;
			break;
		}
	if ( !li ) return VK_ERROR_INITIALIZATION_FAILED;

	PFN_vkGetInstanceProcAddr gipa = li->u.pLayerInfo->pfnNextGetInstanceProcAddr;
	PFN_vkGetDeviceProcAddr gdpa = li->u.pLayerInfo->pfnNextGetDeviceProcAddr;
	li->u.pLayerInfo = li->u.pLayerInfo->pNext;
	PFN_vkCreateDevice create = (PFN_vkCreateDevice)gipa( NULL, "vkCreateDevice" );
	if ( !create ) return VK_ERROR_INITIALIZATION_FAILED;
	VkResult r = create( pd, ci, alloc, out );
	if ( r != VK_SUCCESS ) return r;

	void *key = Key( *out );
	pthread_mutex_lock( &g_lock );
	int slot = -1;
	for ( int i = 0; i < MAX_INST && slot < 0; i++ )
		if ( !g_dev[i].key || g_dev[i].key == key ) slot = i;
	if ( slot < 0 ) slot = 0;
	g_dev[slot].key = key;
	g_dev[slot].gdpa = gdpa;
	pthread_mutex_unlock( &g_lock );
	return r;
}

VKAPI PFN_vkVoidFunction vkGetDeviceProcAddr( VkDevice dev, const char *name )
{
	if ( !strcmp( name, "vkGetDeviceProcAddr" ) ) return (PFN_vkVoidFunction)vkGetDeviceProcAddr;
	if ( !dev ) return NULL;
	PFN_vkGetDeviceProcAddr next = DevNext( Key( dev ) );
	return next ? next( dev, name ) : NULL;
}

VKAPI PFN_vkVoidFunction vkGetInstanceProcAddr( VkInstance instance, const char *name )
{
	if ( !strcmp( name, "vkGetInstanceProcAddr" ) ) return (PFN_vkVoidFunction)vkGetInstanceProcAddr;
	if ( !strcmp( name, "vkCreateInstance" ) ) return (PFN_vkVoidFunction)CreateInstance;
	if ( !strcmp( name, "vkDestroyInstance" ) ) return (PFN_vkVoidFunction)DestroyInstance;
	if ( !strcmp( name, "vkCreateDevice" ) ) return (PFN_vkVoidFunction)CreateDevice;
	if ( !strcmp( name, "vkGetDeviceProcAddr" ) ) return (PFN_vkVoidFunction)vkGetDeviceProcAddr;
	if ( !strcmp( name, "vkGetDisplayModePropertiesKHR" ) ) return (PFN_vkVoidFunction)GetModes;
	if ( !strcmp( name, "vkGetDisplayModeProperties2KHR" ) ) return (PFN_vkVoidFunction)GetModes2;
	if ( !instance ) return NULL;
	struct inst *in = Find( Key( instance ) );
	return in ? in->gipa( instance, name ) : NULL;
}

VKAPI VkResult vkNegotiateLoaderLayerInterfaceVersion( VkNegotiateLayerInterface *v )
{
	if ( v->loaderLayerInterfaceVersion > 2 ) v->loaderLayerInterfaceVersion = 2;
	v->pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
	v->pfnGetDeviceProcAddr = vkGetDeviceProcAddr;
	v->pfnGetPhysicalDeviceProcAddr = NULL;
	return VK_SUCCESS;
}
