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

#include <stdio.h>
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

/** Unit vector (sensor frame) at the centre of a cell. Table driven (no libm): this runs for every cell, for every candidate pose. */
static inline void magCalCellCenter(unsigned index, float out[3])
{
    static const float lonCos[MAG_CAL_CELL_LON_SECTORS] = { -0.98078528f, -0.83146961f, -0.55557023f, -0.19509032f, 0.19509032f, 0.55557023f, 0.83146961f, 0.98078528f, 0.98078528f, 0.83146961f, 0.55557023f, 0.19509032f, -0.19509032f, -0.55557023f, -0.83146961f, -0.98078528f };
    static const float lonSin[MAG_CAL_CELL_LON_SECTORS] = { -0.19509032f, -0.55557023f, -0.83146961f, -0.98078528f, -0.98078528f, -0.83146961f, -0.55557023f, -0.19509032f, 0.19509032f, 0.55557023f, 0.83146961f, 0.98078528f, 0.98078528f, 0.83146961f, 0.55557023f, 0.19509032f };
    unsigned iz = index / MAG_CAL_CELL_LON_SECTORS, il = index % MAG_CAL_CELL_LON_SECTORS;
    float z = ((iz + 0.5f) * (2.0f / MAG_CAL_CELL_Z_BANDS)) - 1.0f;
    float r = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    out[0] = r * lonCos[il]; out[1] = r * lonSin[il]; out[2] = z;
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

/** Angle (degrees) between two unit vectors. */
static inline float magCalAngleDeg(const float a[3], const float b[3])
{
    float c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    if (c > 1.0f) c = 1.0f; else if (c < -1.0f) c = -1.0f;
    return acosf(c) * (180.0f / MAG_CAL_COV_PI);
}

/** Pack a unit vector into three signed bytes (x127). */
static inline void magCalPackUnit(const float u[3], int8_t out[3])
{
    for (int k = 0; k < 3; k++) out[k] = (int8_t)lroundf(u[k] * 127.0f);
}

/* ---------------------------------------------------------------------------------------------------------------------
 * Gravity-referenced guidance.
 *
 * A field that is fixed in the world, seen from a device that is rotated, moves on the sensor-frame sphere so that its angle to the
 * direction of the floor stays constant: u . g = sin(inclination) for ANY orientation. So with part A of the device pointing at the floor
 * (g = A), turning the device about the vertical sweeps the field direction around a ring at angle acos(sin(inclination)) from A,
 * and the whole ring is covered by one slow turn. The floor is easy for a person to judge and the turn is a single rotation, so the
 * instruction is "point <part> at the floor, then turn slowly all the way round".
 * Poses are the 6 faces of the device (front/rear, right/left side, bottom/top): each can be named on the case and judged by eye.
 * ------------------------------------------------------------------------------------------------------------------- */

/** Pose idx (0..MAG_CAL_POSE_COUNT-1) as a unit vector in the body frame: the six faces, in the order +X, -X, +Y, -Y, +Z, -Z. */
static inline void magCalPoseDir(int idx, float out[3])
{
    out[0] = out[1] = out[2] = 0.0f;
    out[idx / 2] = (idx & 1) ? -1.0f : 1.0f;
}

/** Name of a pose given by its packed direction, the way someone holding the unit would say it: "+Y face (right side)". out needs >= 24 bytes.
 *  X is forward, Y right, Z down in the body frame, so the unit's -Z face is its top, +Z its bottom. */
static inline void magCalPoseName(const int8_t dir[3], char *out, int outSize)
{
    static const char letters[3] = { 'X', 'Y', 'Z' };
    static const char *side[3][2] = { { "front", "rear" }, { "right side", "left side" }, { "bottom", "top" } };   // [axis][0 = positive face, 1 = negative face]
    int n = 0, axis = -1, neg = 0;
    for (int k = 0; k < 3; k++) if (dir[k] >= 30 || dir[k] <= -30) { axis = k; neg = dir[k] < 0; break; }
    if (axis < 0) { if (outSize > 0) out[0] = 0; return; }
    char text[32]; int t = 0;
    text[t++] = neg ? '-' : '+'; text[t++] = letters[axis];
    const char *tail = " face ("; for (int i = 0; tail[i]; i++) text[t++] = tail[i];
    for (const char *p = side[axis][neg]; *p; p++) text[t++] = *p;
    text[t++] = ')'; text[t] = 0;
    for (int i = 0; text[i] && n < outSize - 1; i++) out[n++] = text[i];
    out[n] = 0;
}

/**
 * The ring a turn sweeps. With the floor direction g (unit) and sin(inclination) = field . floor, a device turned about the vertical makes the measured
 * field direction u run round the circle { u : u . g = sinI } (the dot product is invariant under any rotation about the vertical). This returns the cells
 * that circle passes THROUGH, as a bitmask (LSB first, like required[]). A cell belongs to the ring only if the circle spends at least
 * MAG_CAL_RING_MIN_ARC_STEPS steps of MAG_CAL_RING_STEP_DEG inside it: a cell that is merely clipped at a corner would never collect enough samples to count,
 * so progress through the ring could not reach 100%.
 */
#define MAG_CAL_RING_STEP_DEG       2.0f
#define MAG_CAL_RING_MIN_ARC_STEPS  3
#define MAG_CAL_RING_MASK_BYTES     (MAG_CAL_CELL_COUNT / 8)
static inline void magCalRingMask(const float g[3], float sinI, uint8_t mask[MAG_CAL_RING_MASK_BYTES])
{
    uint8_t steps[MAG_CAL_CELL_COUNT]; memset(steps, 0, sizeof(steps)); memset(mask, 0, MAG_CAL_RING_MASK_BYTES);
    float gn = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    if (!(gn > 1.0e-6f)) return;
    const float ax[3] = { g[0] / gn, g[1] / gn, g[2] / gn };
    const float s = sinI > 1.0f ? 1.0f : (sinI < -1.0f ? -1.0f : sinI), cI = sqrtf(1.0f - s * s);
    // two unit vectors perpendicular to the axis
    float h[3] = { 1.0f, 0.0f, 0.0f }; if (fabsf(ax[0]) > 0.9f) { h[0] = 0.0f; h[1] = 1.0f; }
    float d = h[0] * ax[0] + h[1] * ax[1] + h[2] * ax[2];
    float e1[3] = { h[0] - d * ax[0], h[1] - d * ax[1], h[2] - d * ax[2] };
    float en = sqrtf(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]); e1[0] /= en; e1[1] /= en; e1[2] /= en;
    const float e2[3] = { ax[1] * e1[2] - ax[2] * e1[1], ax[2] * e1[0] - ax[0] * e1[2], ax[0] * e1[1] - ax[1] * e1[0] };
    const float step = MAG_CAL_RING_STEP_DEG * (MAG_CAL_COV_PI / 180.0f), cd = cosf(step), sd = sinf(step);
    const int n = (int)(360.0f / MAG_CAL_RING_STEP_DEG + 0.5f);
    float c = 1.0f, sn = 0.0f;
    for (int k = 0; k < n; k++)
    {
        float u[3] = { s * ax[0] + cI * (c * e1[0] + sn * e2[0]), s * ax[1] + cI * (c * e1[1] + sn * e2[1]), s * ax[2] + cI * (c * e1[2] + sn * e2[2]) };
        unsigned cell = magCalCellIndex(u); if (steps[cell] < 255) steps[cell]++;
        float c2 = c * cd - sn * sd; sn = sn * cd + c * sd; c = c2;
    }
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) if (steps[i] >= MAG_CAL_RING_MIN_ARC_STEPS) mask[i >> 3] |= (uint8_t)(1u << (i & 7));
}

typedef struct { int cells; int missing; } mag_cal_ring_t;
static inline int magCalRingHas(const uint8_t mask[MAG_CAL_RING_MASK_BYTES], unsigned cell) { return (mask[cell >> 3] >> (cell & 7)) & 1; }
/** How many cells a ring mask holds, and how many of those are required but not yet covered. */
static inline mag_cal_ring_t magCalRingStats(const mag_cal_coverage_t *m, const uint8_t mask[MAG_CAL_RING_MASK_BYTES])
{
    mag_cal_ring_t r = { 0, 0 };
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        if (!magCalRingHas(mask, i)) continue;
        r.cells++;
        if (magCalCellRequired(m, i) && magCalCellGet(m, i) < MAG_CAL_COVERAGE_MIN_COUNT) r.missing++;
    }
    return r;
}
/** The ring of a pose axis held exactly at the floor (what the pose promises). */
static inline mag_cal_ring_t magCalPoseRing(const mag_cal_coverage_t *m, const float a[3], float sinI)
{
    uint8_t mask[MAG_CAL_RING_MASK_BYTES]; magCalRingMask(a, sinI, mask);
    return magCalRingStats(m, mask);
}

/**
 * Choose the next pose: the one whose ring holds the most missing required cells (largest gain), discounted a little for a longer tilt.
 * @param g      current direction of the floor in the body frame (unit)
 * @param sinI   sin(inclination) = field . floor, estimated by the device
 * @param avoid  a pose that must not be chosen (one that timed out), or -1
 * @return pose index, or -1 when no pose would cover anything missing
 */
static inline int magCalChoosePose(const mag_cal_coverage_t *m, const float g[3], float sinI, int avoid)
{
    int best = -1; float bestScore = 0.0f;
    for (int p = 0; p < MAG_CAL_POSE_COUNT; p++)
    {
        if (p == avoid) continue;
        float a[3]; magCalPoseDir(p, a);
        mag_cal_ring_t r = magCalPoseRing(m, a, sinI);
        if (r.missing < 1) continue;
        float score = (float)r.missing * (1.0f - 0.35f * magCalAngleDeg(g, a) / 180.0f);
        if (score > bestScore) { bestScore = score; best = p; }
    }
    return best;
}

// Terminal bell policy: one chime when the instruction changes to a new pose/phase (so a user fighting cabling knows to move on),
// three when the calibration completes. 'prev' is the last status seen (prevValid=false on the first one: no chime).
static inline int magCalChimeCount(const mag_cal_coverage_t &prev, bool prevValid, const mag_cal_coverage_t &cur)
{
    if (!prevValid) return 0;
    const bool doneNow  = cur.lifecycle  == MAG_CAL_LC_COMPLETE || cur.lifecycle  == MAG_CAL_LC_CALIBRATED;
    const bool donePrev = prev.lifecycle == MAG_CAL_LC_COMPLETE || prev.lifecycle == MAG_CAL_LC_CALIBRATED;
    if (doneNow && !donePrev) return 3;
    if (cur.guidePhase == MAG_CAL_PHASE_NONE) return 0;
    const bool samePose = prev.poseDir[0] == cur.poseDir[0] && prev.poseDir[1] == cur.poseDir[1] && prev.poseDir[2] == cur.poseDir[2];
    return (prev.guidePhase != cur.guidePhase || !samePose) ? 1 : 0;
}

/** sin(inclination) the pose ring is sized with. Uses the firmware's own estimate when it reports one; otherwise falls back to the instantaneous
 *  field . floor from the two direction vectors (noisier: it moves with the centre estimate). Returns 0 when neither can be formed. */
static inline int magCalSinI(const mag_cal_coverage_t *c, float *sinI)
{
    if (c->flags & MAG_CAL_FLAG_INCL_VALID) { *sinI = (float)c->inclSin / 127.0f; return 1; }
    if ((c->fieldDir[0] || c->fieldDir[1] || c->fieldDir[2]) && (c->gravDir[0] || c->gravDir[1] || c->gravDir[2]))
    {
        *sinI = ((float)c->fieldDir[0] * c->gravDir[0] + (float)c->fieldDir[1] * c->gravDir[1] + (float)c->fieldDir[2] * c->gravDir[2]) / (127.0f * 127.0f);
        return 1;
    }
    return 0;
}

/** Plain-words reason the quality is not GOOD (empty when there is none). */
static inline const char *magCalQualityWhyText(unsigned why)
{
    switch (why)
    {
    case MAG_CAL_WHY_FIELD_VARIES: return "the magnetic field is not steady: interference nearby, or the device was moving";
    case MAG_CAL_WHY_NORM_OFF:     return "the field strength here differs from where it was calibrated";
    case MAG_CAL_WHY_NORM_INVALID: return "the field strength is far from the calibration (more than 15%): different place, or a magnet or steel nearby";
    default:                       return "";
    }
}

/**
 * How well the coverage surface matches the unit sphere, in percent: 100 = every required cell holds enough samples, 0 = nothing collected. It is the mean, over the
 * corners of the cell grid, of a per-corner value that leans toward the WORST of the (up to four) cells meeting there, so a few missing cells show clearly instead of
 * being averaged away. Corners are weighted by how much of them is required (and by 1/2 on the two pole rows, which are single points). Defined once here so the
 * firmware (completion hold), EvalTool (display) and cltool all mean the same number.
 * @param cells     MAG_CAL_CELL_COUNT/2 bytes of 4-bit counts
 * @param required  MAG_CAL_CELL_COUNT/8 bytes (bit per cell), or NULL for "every cell is required"
 */
static inline float magCalSphereFitPct(const uint8_t *cells, const uint8_t *required)
{
    double sum = 0.0, wsum = 0.0;
    for (int iz = 0; iz <= MAG_CAL_CELL_Z_BANDS; iz++)
    {
        const double wz = (iz == 0 || iz == MAG_CAL_CELL_Z_BANDS) ? 0.5 : 1.0;
        for (int il = 0; il < MAG_CAL_CELL_LON_SECTORS; il++)
        {
            double sf = 0.0, sr = 0.0, mn = 1.0; int n = 0;
            for (int dz = -1; dz <= 0; dz++)
                for (int dl = -1; dl <= 0; dl++)
                {
                    int z = iz + dz, l = ((il + dl) % MAG_CAL_CELL_LON_SECTORS + MAG_CAL_CELL_LON_SECTORS) % MAG_CAL_CELL_LON_SECTORS;
                    if (z < 0 || z >= MAG_CAL_CELL_Z_BANDS) continue;
                    unsigned cell = (unsigned)(z * MAG_CAL_CELL_LON_SECTORS + l);
                    const int req = required ? ((required[cell >> 3] >> (cell & 7)) & 1) : 1;
                    double f = req ? (double)magCalCellsGet(cells, cell) / (double)MAG_CAL_COVERAGE_MIN_COUNT : 1.0;   // a cell that is not required is as good as full
                    if (f > 1.0) f = 1.0;
                    sf += f; sr += req ? 1.0 : 0.0; if (f < mn) mn = f; n++;
                }
            if (!n) continue;
            const double shape = 0.5 * (sf / n + mn), req = sr / n;
            sum += wz * req * shape; wsum += wz * req;
        }
    }
    return wsum > 0.0 ? (float)(100.0 * sum / wsum) : 100.0f;
}
static inline float magCalSphereFitPctOf(const mag_cal_coverage_t *c)
{
    return magCalSphereFitPct(c->cells, (c->flags & MAG_CAL_FLAG_REQUIRED_VALID) ? c->required : NULL);
}

/** Why the calibration is not advancing, as a short phrase (empty when there is nothing to say). */
static inline const char *magCalReasonText(unsigned reason)
{
    static const char *t[] = { "", "rotate faster", "turn about the vertical axis", "field unsteady / interference", "area full",
                               "fit not accepted yet", "norm check not passed yet", "mode unsupported", "extended mode: keep covering the sphere (Finish to accept)" };
    return reason < 9 ? t[reason] : "";
}

static inline void magCalAppend(char *buf, size_t cap, size_t *len, const char *sep, const char *str)
{   // clamped append: snprintf returns the length it WOULD have written, not what fit
    if (*len >= cap - 1) return;
    if (*len) { size_t n = strlen(sep); if (n > cap - 1 - *len) n = cap - 1 - *len; memcpy(buf + *len, sep, n); *len += n; }
    size_t n = strlen(str); if (n > cap - 1 - *len) n = cap - 1 - *len; memcpy(buf + *len, str, n); *len += n; buf[*len] = 0;
}

/** True when the status carries pose guidance (aim at / turn about the floor). */
static inline int magCalHaveGuide(const mag_cal_coverage_t *c)
{
    return (c->guidePhase == MAG_CAL_PHASE_AIM || c->guidePhase == MAG_CAL_PHASE_TURN) && (c->poseDir[0] || c->poseDir[1] || c->poseDir[2]);
}

/** The one-line instruction shown to the user by every front end (cltool, EvalTool): same words everywhere. */
static inline void magCalInstruction(const mag_cal_coverage_t *cov, char *out, size_t cap)
{
    size_t il = 0; char tmp[120]; out[0] = 0;
    const int fitProblem = (cov->reason == MAG_CAL_REASON_FIT_REJECTED || cov->reason == MAG_CAL_REASON_NORM_CHECK);
    const int haveGuide = magCalHaveGuide(cov);
    if (cov->reason != MAG_CAL_REASON_NONE && cov->reason < 9 && cov->reason != MAG_CAL_REASON_REGION_FULL)
    {
        if (fitProblem && !haveGuide) { snprintf(tmp, sizeof(tmp), "%s - the map is complete, more rotation will not help", magCalReasonText(cov->reason)); magCalAppend(out, cap, &il, "; ", tmp); }   // only when nothing is left to cover
        else magCalAppend(out, cap, &il, "; ", magCalReasonText(cov->reason));
    }
    if (haveGuide)
    {   // Pose guidance relative to the FLOOR: easy to judge by eye. Shown even while a fit is being retried: gaps still help.
        char pose[24];
        magCalPoseName(cov->poseDir, pose, sizeof(pose));
        if (cov->guidePhase == MAG_CAL_PHASE_AIM) snprintf(tmp, sizeof(tmp), "point the %s at the floor: %u deg to go", pose, (unsigned)cov->poseErrDeg);
        else
        {   // second step: shorter name ("+Y face"), and a picture people already know
            char shortName[16]; int n = 0; for (; pose[n] && pose[n] != '(' && n < 15; n++) shortName[n] = pose[n]; while (n > 0 && shortName[n - 1] == ' ') n--; shortName[n] = 0;
            snprintf(tmp, sizeof(tmp), "keep the %s down; turn slowly in place, like a turntable (ring %u%%)", shortName, (unsigned)cov->ringPct);
        }
        magCalAppend(out, cap, &il, "; ", tmp);
    }
    else if (!fitProblem && cov->lifecycle == MAG_CAL_LC_COLLECTING && cov->mode == MAG_CAL_STATE_SINGLE_AXIS && cov->reason == MAG_CAL_REASON_NONE)
        magCalAppend(out, cap, &il, "; ", "keep turning about the vertical axis");
    if (!(cov->flags & MAG_CAL_FLAG_COVERAGE_VALID)) magCalAppend(out, cap, &il, "; ", "coverage map unknown");
}

#ifdef __cplusplus
}
#endif

#endif // IS_MAG_CAL_COVERAGE_H
