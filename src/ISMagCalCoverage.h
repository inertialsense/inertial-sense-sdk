/**
 * @file ISMagCalCoverage.h
 * @brief Helpers for the magnetometer calibration coverage map (DID_MAG_CAL_COVERAGE, mag_cal_coverage_t).
 *
 * Shared by firmware (fills the map) and hosts (cltool, EvalTool: render it). Header-only so it can be used on
 * embedded targets without pulling in SDK sources.
 *
 * The map divides the sphere of field directions (sensor/body frame) into MAG_CAL_CELL_COUNT equal-area cells:
 * MAG_CAL_CELL_Z_BANDS bands of equal sin(latitude) width, each split into MAG_CAL_CELL_LON_SECTORS longitude sectors.
 * Equal width in z means equal area on a sphere (Archimedes), so every cell has the same solid angle (4*pi/128 sr).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */
#ifndef IS_MAG_CAL_COVERAGE_H
#define IS_MAG_CAL_COVERAGE_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAG_CAL_COV_PI      3.14159265358979f
#define MAG_CAL_COV_TWO_PI  6.28318530717959f

/** Cell index (0..MAG_CAL_CELL_COUNT-1) of a UNIT vector in the sensor frame. */
static inline unsigned magCalCellIndex(const float v[3])
{
    int iz = (int)((v[2] + 1.0f) * (MAG_CAL_CELL_Z_BANDS * 0.5f));
    if (iz > MAG_CAL_CELL_Z_BANDS - 1) iz = MAG_CAL_CELL_Z_BANDS - 1;
    if (iz < 0) iz = 0;
    int il = (int)((atan2f(v[1], v[0]) + MAG_CAL_COV_PI) * (MAG_CAL_CELL_LON_SECTORS / MAG_CAL_COV_TWO_PI));
    if (il > MAG_CAL_CELL_LON_SECTORS - 1) il = MAG_CAL_CELL_LON_SECTORS - 1;
    if (il < 0) il = 0;
    return (unsigned)(iz * MAG_CAL_CELL_LON_SECTORS + il);
}

/** Cell index of any non-zero vector (normalizes first). Returns false (and index 0) for a zero or non-finite vector. */
static inline int magCalCellIndexOfVector(const float v[3], unsigned *index)
{
    float n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (!(n2 > 1.0e-12f) || n2 != n2) { *index = 0; return 0; }
    float inv = 1.0f / sqrtf(n2), u[3] = { v[0] * inv, v[1] * inv, v[2] * inv };
    *index = magCalCellIndex(u);
    return 1;
}

/** Unit vector (sensor frame) at the centre of a cell. */
static inline void magCalCellCenter(unsigned index, float out[3])
{
    unsigned iz = index / MAG_CAL_CELL_LON_SECTORS, il = index % MAG_CAL_CELL_LON_SECTORS;
    float z = ((iz + 0.5f) * (2.0f / MAG_CAL_CELL_Z_BANDS)) - 1.0f;
    float lon = ((il + 0.5f) * (MAG_CAL_COV_TWO_PI / MAG_CAL_CELL_LON_SECTORS)) - MAG_CAL_COV_PI;
    float r = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    out[0] = r * cosf(lon); out[1] = r * sinf(lon); out[2] = z;
}

/** Raw 4-bit cell accessors on a MAG_CAL_CELL_COUNT/2 byte array (firmware keeps just the cells, not a whole mag_cal_coverage_t). */
static inline unsigned magCalCellsGet(const uint8_t *cells, unsigned i)
{
    return (cells[i >> 1] >> ((i & 1u) << 2)) & 0xFu;
}

static inline void magCalCellsSet(uint8_t *cells, unsigned i, unsigned count)
{
    if (count > MAG_CAL_CELL_SATURATED) count = MAG_CAL_CELL_SATURATED;
    unsigned shift = (i & 1u) << 2;
    cells[i >> 1] = (uint8_t)((cells[i >> 1] & ~(0xFu << shift)) | (count << shift));
}

/** Add one sample to a cell (saturating). */
static inline void magCalCellsAdd(uint8_t *cells, unsigned i)
{
    unsigned n = magCalCellsGet(cells, i);
    if (n < MAG_CAL_CELL_SATURATED) magCalCellsSet(cells, i, n + 1);
}

/** Sample count (0..MAG_CAL_CELL_SATURATED) held in a cell. */
static inline unsigned magCalCellGet(const mag_cal_coverage_t *m, unsigned i) { return magCalCellsGet(m->cells, i); }

/** Set a cell's count (clamped to the 4-bit range). */
static inline void magCalCellSet(mag_cal_coverage_t *m, unsigned i, unsigned count) { magCalCellsSet(m->cells, i, count); }

/** Add one sample to a cell (saturating). */
static inline void magCalCellAdd(mag_cal_coverage_t *m, unsigned i) { magCalCellsAdd(m->cells, i); }

/** Whether a cell is flagged as required for this calibration. */
static inline int magCalCellRequired(const mag_cal_coverage_t *m, unsigned i)
{
    return (m->required[i >> 3] >> (i & 7u)) & 1;
}

static inline void magCalCellSetRequired(mag_cal_coverage_t *m, unsigned i, int required)
{
    if (required) m->required[i >> 3] |= (uint8_t)(1u << (i & 7u));
    else          m->required[i >> 3] &= (uint8_t)~(1u << (i & 7u));
}

/** Mark every cell required (multi-axis, or whenever the required region is unknown). */
static inline void magCalRequireAll(mag_cal_coverage_t *m)
{
    memset(m->required, 0xFF, sizeof(m->required));
}

/** Coverage statistics against MAG_CAL_COVERAGE_MIN_COUNT. */
typedef struct
{
    unsigned requiredCells;      //!< cells flagged required
    unsigned requiredCovered;    //!< required cells with at least MAG_CAL_COVERAGE_MIN_COUNT samples
    unsigned coveredCells;       //!< all cells with at least MAG_CAL_COVERAGE_MIN_COUNT samples
} mag_cal_cov_stats_t;

static inline mag_cal_cov_stats_t magCalCoverageStats(const mag_cal_coverage_t *m)
{
    mag_cal_cov_stats_t s = { 0, 0, 0 };
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        int covered = magCalCellGet(m, i) >= MAG_CAL_COVERAGE_MIN_COUNT;
        int req = magCalCellRequired(m, i);
        s.coveredCells += covered;
        s.requiredCells += req;
        s.requiredCovered += (req && covered);
    }
    return s;
}

/** Percent (0..100) of required cells covered. 0 if nothing is required. */
static inline uint8_t magCalProgressPct(const mag_cal_cov_stats_t *s)
{
    return s->requiredCells ? (uint8_t)((100u * s->requiredCovered) / s->requiredCells) : 0;
}

/** Number of missing required cells in the 3x3 neighbourhood of a cell (itself included; longitude wraps). The "gain" of aiming there. */
static inline int magCalGapScore(const mag_cal_coverage_t *m, int cell)
{
    int iz = cell / MAG_CAL_CELL_LON_SECTORS, il = cell % MAG_CAL_CELL_LON_SECTORS, score = 0;
    for (int dz = -1; dz <= 1; dz++)
    {
        int z = iz + dz; if (z < 0 || z >= MAG_CAL_CELL_Z_BANDS) continue;
        for (int dl = -1; dl <= 1; dl++)
        {
            unsigned n = (unsigned)(z * MAG_CAL_CELL_LON_SECTORS + ((il + dl + MAG_CAL_CELL_LON_SECTORS) % MAG_CAL_CELL_LON_SECTORS));
            if (magCalCellRequired(m, n) && magCalCellGet(m, n) < MAG_CAL_COVERAGE_MIN_COUNT) score++;
        }
    }
    return score;
}

/** Angle (degrees) between two unit vectors. */
static inline float magCalAngleDeg(const float a[3], const float b[3])
{
    float c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    if (c > 1.0f) c = 1.0f; else if (c < -1.0f) c = -1.0f;
    return acosf(c) * (180.0f / MAG_CAL_COV_PI);
}

/**
 * Choose the next target cell: the place where one move closes the most gaps.
 *  - Keep the previous target until it is reached (the cell is covered, or the field is within MAG_CAL_GUIDE_REACHED_DEG of its centre
 *    and it will be covered momentarily) so the target does not flip while the user is moving toward it.
 *  - Otherwise pick, among missing required cells whose gap score is within 25% of the best, the one nearest to the current field
 *    direction (largest gain first, shortest move as the tie-break).
 * @param cur   current field direction, unit vector in the sensor frame (NULL = unknown: nearest-ness is ignored)
 * @param prev  previous target cell or -1
 * @return target cell, or -1 when nothing required is missing
 */
static inline int magCalChooseTarget(const mag_cal_coverage_t *m, const float cur[3], int prev)
{
    if (prev >= 0 && prev < MAG_CAL_CELL_COUNT && magCalCellRequired(m, (unsigned)prev) && magCalCellGet(m, (unsigned)prev) < MAG_CAL_COVERAGE_MIN_COUNT)
    {
        float c[3]; magCalCellCenter((unsigned)prev, c);
        if (!cur || magCalAngleDeg(cur, c) > MAG_CAL_GUIDE_REACHED_DEG) return prev;     // still on its way
    }
    int best = 0, scores[MAG_CAL_CELL_COUNT];
    for (int i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        scores[i] = (magCalCellRequired(m, (unsigned)i) && magCalCellGet(m, (unsigned)i) < MAG_CAL_COVERAGE_MIN_COUNT) ? magCalGapScore(m, i) : 0;
        if (scores[i] > best) best = scores[i];
    }
    if (best == 0) return -1;
    int threshold = (3 * best + 3) / 4;     // ceil(0.75 * best)
    int pick = -1; float pickAngle = 1.0e9f;
    for (int i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        if (scores[i] < threshold) continue;
        float c[3]; magCalCellCenter((unsigned)i, c);
        float ang = cur ? magCalAngleDeg(cur, c) : 0.0f;
        if (ang < pickAngle) { pickAngle = ang; pick = i; }
    }
    return pick;
}

/**
 * Rotation to apply to the DEVICE so the field direction seen in the sensor frame moves from cur to target.
 * The field is fixed in the world, so rotating the body by R makes the sensor-frame direction u' = R^T u. The minimal rotation that
 * takes u to d has axis d x u (body frame) and angle acos(u.d). Returned as roll/pitch/yaw degrees about body X/Y/Z (right-hand rule).
 * @return total angle in degrees
 */
static inline float magCalGuideRotationDeg(const float cur[3], const float target[3], float rotDeg[3])
{
    float ax = target[1] * cur[2] - target[2] * cur[1];
    float ay = target[2] * cur[0] - target[0] * cur[2];
    float az = target[0] * cur[1] - target[1] * cur[0];
    float s = sqrtf(ax * ax + ay * ay + az * az), ang = magCalAngleDeg(cur, target);
    if (s < 1.0e-6f)
    {   // Same or opposite direction. Opposite: any axis perpendicular to cur works.
        rotDeg[0] = rotDeg[1] = rotDeg[2] = 0.0f;
        if (ang > 90.0f) { float t[3] = { 1, 0, 0 }; if (fabsf(cur[0]) > 0.9f) { t[0] = 0; t[1] = 1; }
            float px = cur[1] * t[2] - cur[2] * t[1], py = cur[2] * t[0] - cur[0] * t[2], pz = cur[0] * t[1] - cur[1] * t[0];
            float n = sqrtf(px * px + py * py + pz * pz); rotDeg[0] = ang * px / n; rotDeg[1] = ang * py / n; rotDeg[2] = ang * pz / n; }
        return ang;
    }
    rotDeg[0] = ang * ax / s; rotDeg[1] = ang * ay / s; rotDeg[2] = ang * az / s;
    return ang;
}

/** Wire encoding of the target: cell index + 1, 0 = none (so a zeroed packet carries no target). */
static inline uint8_t magCalPackTarget(int cell) { return (cell >= 0 && cell < MAG_CAL_CELL_COUNT) ? (uint8_t)(cell + 1) : (uint8_t)MAG_CAL_GUIDE_NONE; }
static inline int magCalUnpackTarget(uint8_t wire) { return (wire >= 1 && wire <= MAG_CAL_CELL_COUNT) ? (int)wire - 1 : -1; }

/** Pack/unpack helpers for the wire fields. */
static inline int8_t magCalPackRotDeg(float deg)
{
    float v = deg / MAG_CAL_GUIDE_DEG_PER_LSB; if (v > 127.0f) v = 127.0f; if (v < -127.0f) v = -127.0f;
    return (int8_t)lroundf(v);
}
static inline void magCalPackUnit(const float u[3], int8_t out[3])
{
    for (int k = 0; k < 3; k++) out[k] = (int8_t)lroundf(u[k] * 127.0f);
}

#ifdef __cplusplus
}
#endif

#endif // IS_MAG_CAL_COVERAGE_H
