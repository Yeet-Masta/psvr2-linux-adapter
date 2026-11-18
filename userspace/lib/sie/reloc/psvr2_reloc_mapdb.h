/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PSVR2 relocaliser room-map database ("MD" image, sceMapDb.bin).
 *
 * The map is one flat fixed-size image (layout in docs/sony-re/reloc.md,
 * "Room map file"). These helpers operate on such an image; results and
 * every byte they write match the reference driver (checked by
 * scripts/sony-re/difftest/t_reloc_mapdb.c).
 */
#ifndef PSVR2_SIE_RELOC_MAPDB_H
#define PSVR2_SIE_RELOC_MAPDB_H
#include <stdint.h>
#include <stddef.h>

#define PSVR2_MAPDB_SIZE        0x12d18a0u
#define PSVR2_MAPDB_V1_END      0x12cb788u  /* end of a version-1 file */
#define PSVR2_MAPDB_SNAPSHOT    0x12d18a8u  /* image + u32 seed + u32 counter */

#define PSVR2_MAPDB_KF_MAX      97
#define PSVR2_MAPDB_FEAT_MAX    256
#define PSVR2_MAPDB_EDGE_MAX    4848

/* Image offsets. */
#define PSVR2_MAPDB_MAPID_SIG   0x28
#define PSVR2_MAPDB_MAPID       0x34
#define PSVR2_MAPDB_QLOG        0x38
#define PSVR2_MAPDB_ELOG        0x57b8
#define PSVR2_MAPDB_ELOG_IDX    0xdaf8
#define PSVR2_MAPDB_QLOG_IDX    0xdafc
#define PSVR2_MAPDB_EVENT_NO    0xdb00
#define PSVR2_MAPDB_KF          0xdb08
#define PSVR2_MAPDB_KF_STRIDE   0x2ed20
#define PSVR2_MAPDB_FEAT        0xf0      /* within a keyframe slot */
#define PSVR2_MAPDB_FEAT_STRIDE 0x2ec
#define PSVR2_MAPDB_KF_INTR     0x2ecf0   /* within a keyframe slot */
#define PSVR2_MAPDB_KF_BITS     0x11cb428
#define PSVR2_MAPDB_ID          0x11cb450
#define PSVR2_MAPDB_ID_BITS     0x11cba60
#define PSVR2_MAPDB_MARK_BITS   0x11cba80
#define PSVR2_MAPDB_EDGE        0x11cba98
#define PSVR2_MAPDB_EDGE_SIZE   0xd8
#define PSVR2_MAPDB_EDGE_BITS   0x12cb518
#define PSVR2_MAPDB_TRAILER     0x12cb788
#define PSVR2_MAPDB_KF_STATS    0x12cb7a0

#define PSVR2_MAPDB_EINVAL      (-5)
#define PSVR2_MAPDB_ENOENT      (-12)
#define PSVR2_MAPDB_EBADFILE    (-11)

/* Wall-clock time as the reference stores it: unix seconds, microseconds. */
typedef struct {
	uint64_t sec, usec;
} psvr2_mapdb_time;

/* Database context: the image plus the reference's process-wide state. */
typedef struct {
	uint8_t *img;               /* PSVR2_MAPDB_SIZE bytes */
	uint32_t id_seed;           /* map-id base */
	uint32_t id_count;          /* map ids handed out */
	char obf[128];              /* log-text scratch, kept between calls */
	psvr2_mapdb_time opened;    /* time of last load / save */
	/* Scratch list: live keyframes {serial, index}. */
	uint32_t live_n;
	uint32_t live[PSVR2_MAPDB_KF_MAX][2];
	/* Scratch list: deleted keyframes. */
	uint32_t dead_n;
	uint32_t dead[PSVR2_MAPDB_KF_MAX];
} psvr2_mapdb;

/* Whole-image operations. */
void psvr2_mapdb_reset(psvr2_mapdb *db, const psvr2_mapdb_time *now);
void psvr2_mapdb_new_map_id(psvr2_mapdb *db);
void psvr2_mapdb_bump_map_id(psvr2_mapdb *db);
int psvr2_mapdb_get_map_id(const psvr2_mapdb *db, uint32_t *id);
int psvr2_mapdb_check_hash(const psvr2_mapdb *db, size_t body_end);
void psvr2_mapdb_seal(psvr2_mapdb *db);
int psvr2_mapdb_export(psvr2_mapdb *db, uint8_t *buf, uint32_t size);
int psvr2_mapdb_import(psvr2_mapdb *db, const uint8_t *buf, uint32_t size);
int psvr2_mapdb_load(psvr2_mapdb *db, const uint8_t *file, size_t len, const psvr2_mapdb_time *now);
void psvr2_mapdb_account_time(psvr2_mapdb *db, const psvr2_mapdb_time *now);
uint64_t psvr2_mapdb_log(psvr2_mapdb *db, const char *text);
int64_t psvr2_mapdb_last_event(const psvr2_mapdb *db);
int64_t psvr2_mapdb_oldest_query(const psvr2_mapdb *db);

/* Keyframes. */
int psvr2_mapdb_kf_alloc(psvr2_mapdb *db, const uint8_t *kf, const float *intr, const float *stats,
			 uint32_t group, uint32_t nfeat, uint64_t ts, const uint32_t *tag,
			 uint32_t *index, const psvr2_mapdb_time *now);
int psvr2_mapdb_kf_set_feature(psvr2_mapdb *db, uint32_t kf, int32_t i, const uint8_t *rec);
int psvr2_mapdb_kf_get_feature(const psvr2_mapdb *db, uint32_t kf, uint32_t i, float pos[3],
			       uint32_t *word, float cov[9]);
int psvr2_mapdb_kf_get_intrinsics(const psvr2_mapdb *db, uint32_t kf, float *a1, float *a2,
				  float *a0, float k1[9], float k2[9]);
int psvr2_mapdb_kf_get_time(const psvr2_mapdb *db, uint32_t kf, uint64_t *ts);
int psvr2_mapdb_kf_get_tag(const psvr2_mapdb *db, uint32_t kf, uint32_t *tag);
int psvr2_mapdb_kf_get_state(const psvr2_mapdb *db, uint32_t kf, uint32_t *state);
int psvr2_mapdb_kf_get_group(const psvr2_mapdb *db, uint32_t kf, uint32_t *group);
int psvr2_mapdb_kf_set_group(psvr2_mapdb *db, uint32_t kf, uint32_t group);
int psvr2_mapdb_kf_get_pose(const psvr2_mapdb *db, uint32_t kf, float t[3], float r[9]);
int psvr2_mapdb_kf_get_info(const psvr2_mapdb *db, uint32_t kf, uint8_t *out);
int psvr2_mapdb_kf_set_info(psvr2_mapdb *db, uint32_t kf, const uint8_t *in);
int psvr2_mapdb_kf_dump(const psvr2_mapdb *db, uint32_t kf, uint8_t *out);
int psvr2_mapdb_kf_get_stats(const psvr2_mapdb *db, uint32_t kf, uint8_t *out);
uint32_t psvr2_mapdb_kf_nfeat(const psvr2_mapdb *db, uint32_t kf);
void psvr2_mapdb_kf_count_hit(psvr2_mapdb *db, uint32_t kf);
void psvr2_mapdb_kf_count_match(psvr2_mapdb *db, uint32_t kf, int ok);
void psvr2_mapdb_kf_set_state(psvr2_mapdb *db, uint32_t kf, uint32_t state);
void psvr2_mapdb_kf_set_link(psvr2_mapdb *db, uint32_t kf, uint64_t link);
int psvr2_mapdb_kf_count(const psvr2_mapdb *db);
int psvr2_mapdb_kf_count_state(const psvr2_mapdb *db, int32_t state, int32_t tag);
void psvr2_mapdb_kf_purge(psvr2_mapdb *db);

/* Keyframe-id table. */
int psvr2_mapdb_id_alloc(psvr2_mapdb *db, uint32_t kf, uint32_t *index);
int psvr2_mapdb_id_get_serial(const psvr2_mapdb *db, uint32_t id, uint32_t *serial);
int psvr2_mapdb_id_get_kf(const psvr2_mapdb *db, uint32_t id, uint32_t *kf);
int psvr2_mapdb_id_find_any(const psvr2_mapdb *db, int32_t kf, uint32_t *id);
void psvr2_mapdb_id_find(const psvr2_mapdb *db, int32_t kf, uint32_t *id);
void psvr2_mapdb_id_mark(psvr2_mapdb *db, uint32_t n, const uint32_t *ids);
int psvr2_mapdb_id_count_active(const psvr2_mapdb *db, int32_t *n);
int psvr2_mapdb_id_count_live(psvr2_mapdb *db, int32_t *n);
int psvr2_mapdb_id_purge(psvr2_mapdb *db);

/* Pose-graph edges. */
int psvr2_mapdb_edge_alloc(psvr2_mapdb *db, const uint8_t *edge, uint32_t *index);
int psvr2_mapdb_edge_get(const psvr2_mapdb *db, uint32_t e, uint8_t *out);
void psvr2_mapdb_edge_set_removed(psvr2_mapdb *db, uint32_t e);
void psvr2_mapdb_edge_free(psvr2_mapdb *db, uint32_t e);
void psvr2_mapdb_edge_purge(psvr2_mapdb *db);
int psvr2_mapdb_edge_count(const psvr2_mapdb *db);
int psvr2_mapdb_edge_count_kf(const psvr2_mapdb *db, int32_t kf);
void psvr2_mapdb_edge_exists(const psvr2_mapdb *db, int32_t a, int32_t b, int32_t type, uint8_t *found);
void psvr2_mapdb_edge_find(const psvr2_mapdb *db, int32_t a, int32_t b, int32_t type, uint32_t *e);
int psvr2_mapdb_summary(const psvr2_mapdb *db, uint8_t *out);
void psvr2_mapdb_edge_copy(uint8_t *dst, const uint8_t *src);

/* Text obfuscation used by the logs (see reloc.md). */
const char *psvr2_mapdb_obfuscate(psvr2_mapdb *db, const char *text);
#endif
