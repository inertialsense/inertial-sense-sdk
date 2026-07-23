/**
 * @file rtk_defines.h
 * @brief RTKLIB-derived build-time configuration: enabled GNSS constellations, observation/frequency
 * limits, and per-constellation satellite numbering ranges used by the embedded RTK positioning engine.
 *
 * Constellation support is selected at compile time via the ENA* macros below; enabling a constellation
 * pulls in its satellite-numbering range (the MINPRN, MAXPRN, and NSAT macros for that constellation)
 * and system count (the NSYS macro for that constellation), otherwise those are defined as 0 so
 * downstream array sizing (e.g. GPS_EPHEMERIS_ARRAY_SIZE) stays consistent.
 * DO NOT reorder or resize these without checking every consumer -- several are used as fixed array
 * dimensions in wire-format structures.
 *
 * @author Inertial Sense, Inc. (RTKLIB-derived)
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef __RTK_EMBEDDED_DEFINES_H_
#define __RTK_EMBEDDED_DEFINES_H_

/** Enabled GNSS constellations (comment out to disable; see the corresponding NSYS, MINPRN, and MAXPRN macros below). */
#define ENAGAL      //!< Galileo enabled
#define ENACMP      //!< BeiDou (Compass) enabled
// #define ENAGLO   //!< GLONASS -- disabled
// #define ENAQZS   //!< QZSS -- disabled
// #define ENASBS   //!< SBAS -- disabled

/** Observation buffer and frequency limits. */
#define NUMSATSOL   30      //!< Max number of observations used in the solution
#define MAXOBS      55      //!< Max number of observations used in pre-buffer
#define NFREQ       2       //!< Number of carrier frequencies
#define NEXOBS      0       //!< Number of extended obs codes

/** Selects which second-frequency slot RTK uses: L2 for u-blox ZED-F9P (GPX_GNSS_F9P), L5 otherwise. */
#if defined(GPX_GNSS_F9P)
    #error "F9P defined. Are you sure want to do this?  If so comment this line out in rtk_defines.h"
    #define L1_L5_RTK   0        //!< Use second slot for L2
#else
    #define L1_L5_RTK   1        //!< Use second slot for L5
#endif

// #if defined(RTK_EMBEDDED)
// #include "data_sets.h"
// #else
// #include "ISConstants.h"
// #endif

// #define RTK_MALLOC MALLOC
// // #define RTK_CALLOC calloc    // not supported
// #define RTK_FREE FREE

#define NFREQ2 (NFREQ == 1 ? 2 : NFREQ) //!< Effective frequency count for array sizing -- DO NOT CHANGE
#define RTK_INPUT_COUNT 2 //!< Number of RTK input streams: rover, base station -- DO NOT CHANGE

#define HALF_MAXOBS (MAXOBS/2) //!< Half of @ref MAXOBS, used for splitting rover/base observation buffers

#define MAXSUBFRMLEN 152   //!< Max GNSS navigation subframe length (bytes)
#define MAXRAWLEN 2048     //!< Max raw GNSS receiver message length (bytes)

/** Per-constellation carrier-frequency counts, used for array sizing when the constellation is enabled. */
#ifdef ENAGLO
#define NFREQGLO 2  //!< GLONASS carrier-frequency count
#else
#define NFREQGLO 0  //!< GLONASS disabled
#endif

#ifdef ENAGAL
#define NFREQGAL 2  //!< Galileo carrier-frequency count
#else
#define NFREQGAL 0  //!< Galileo disabled
#endif

/** GPS satellite PRN numbering range and system count (always enabled). */
#define MINPRNGPS   1                   //!< min satellite PRN number of GPS
#define MAXPRNGPS   32                  //!< max satellite PRN number of GPS
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) //!< number of GPS satellites
#define NSYSGPS     1                   //!< GPS system count (always 1; GPS cannot be disabled)

/** GLONASS satellite slot numbering range and system count; zeroed out when ENAGLO is not defined. */
#ifdef ENAGLO
#define MINPRNGLO   1                   //!< min satellite slot number of GLONASS
#define MAXPRNGLO   27                  //!< max satellite slot number of GLONASS
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) //!< number of GLONASS satellites
#define NSYSGLO     1                   //!< GLONASS enabled: system count 1
#else
#define MINPRNGLO   0                   //!< GLONASS disabled
#define MAXPRNGLO   0                   //!< GLONASS disabled
#define NSATGLO     0                   //!< GLONASS disabled: no satellites
#define NSYSGLO     0                   //!< GLONASS disabled: system count 0
#endif

/** Galileo satellite PRN numbering range and system count; zeroed out when ENAGAL is not defined. */
#ifdef ENAGAL
#define MINPRNGAL   1                   //!< min satellite PRN number of Galileo
#define MAXPRNGAL   36                  //!< max satellite PRN number of Galileo
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) //!< number of Galileo satellites
#define NSYSGAL     1                   //!< Galileo enabled: system count 1
#else
#define MINPRNGAL   0                   //!< Galileo disabled
#define MAXPRNGAL   0                   //!< Galileo disabled
#define NSATGAL     0                   //!< Galileo disabled: no satellites
#define NSYSGAL     0                   //!< Galileo disabled: system count 0
#endif

/** QZSS satellite PRN numbering range (including SAIF sub-range) and system count; zeroed out when ENAQZS is not defined. */
#ifdef ENAQZS
#define MINPRNQZS   193                 //!< min satellite PRN number of QZSS
#define MAXPRNQZS   202                 //!< max satellite PRN number of QZSS
#define MINPRNQZS_S 183                 //!< min satellite PRN number of QZSS SAIF
#define MAXPRNQZS_S 191                 //!< max satellite PRN number of QZSS SAIF
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) //!< number of QZSS satellites
#define NSYSQZS     1                   //!< QZSS enabled: system count 1
#else
#define MINPRNQZS   0                   //!< QZSS disabled
#define MAXPRNQZS   0                   //!< QZSS disabled
#define MINPRNQZS_S 0                   //!< QZSS disabled
#define MAXPRNQZS_S 0                   //!< QZSS disabled
#define NSATQZS     0                   //!< QZSS disabled: no satellites
#define NSYSQZS     0                   //!< QZSS disabled: system count 0
#endif

/** BeiDou satellite numbering range and system count; zeroed out when ENACMP is not defined. */
#ifdef ENACMP
#define MINPRNCMP   1                   //!< min satellite sat number of BeiDou
#define MAXPRNCMP   63                  //!< max satellite sat number of BeiDou
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) //!< number of BeiDou satellites
#define NSYSCMP     1                   //!< BeiDou enabled: system count 1
#else
#define MINPRNCMP   0                   //!< BeiDou disabled
#define MAXPRNCMP   0                   //!< BeiDou disabled
#define NSATCMP     0                   //!< BeiDou disabled: no satellites
#define NSYSCMP     0                   //!< BeiDou disabled: system count 0
#endif

/** IRNSS satellite numbering range and system count; zeroed out when ENAIRN is not defined. */
#ifdef ENAIRN
#define MINPRNIRN   1                   //!< min satellite sat number of IRNSS
#define MAXPRNIRN   14                  //!< max satellite sat number of IRNSS
#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) //!< number of IRNSS satellites
#define NSYSIRN     1                   //!< IRNSS enabled: system count 1
#else
#define MINPRNIRN   0                   //!< IRNSS disabled
#define MAXPRNIRN   0                   //!< IRNSS disabled
#define NSATIRN     0                   //!< IRNSS disabled: no satellites
#define NSYSIRN     0                   //!< IRNSS disabled: system count 0
#endif

/** LEO (low earth orbit augmentation) satellite numbering range and system count; zeroed out when ENALEO is not defined. */
#ifdef ENALEO
#define MINPRNLEO   1                   //!< min satellite sat number of LEO
#define MAXPRNLEO   10                  //!< max satellite sat number of LEO
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) //!< number of LEO satellites
#define NSYSLEO     1                   //!< LEO enabled: system count 1
#else
#define MINPRNLEO   0                   //!< LEO disabled
#define MAXPRNLEO   0                   //!< LEO disabled
#define NSATLEO     0                   //!< LEO disabled: no satellites
#define NSYSLEO     0                   //!< LEO disabled: system count 0
#endif

/** SBAS satellite PRN numbering range, system count, and ephemeris array size; zeroed out when ENASBS is not defined. */
#ifdef ENASBS
#define MINPRNSBS   120                 //!< min satellite PRN number of SBAS
#define MAXPRNSBS   158                 //!< max satellite PRN number of SBAS
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) //!< number of SBAS satellites
#define SBAS_EPHEMERIS_ARRAY_SIZE NSATSBS //!< SBAS ephemeris array size, equal to @ref NSATSBS
#else
#define MINPRNSBS   0                   //!< SBAS disabled
#define MAXPRNSBS   0                   //!< SBAS disabled
#define NSATSBS     0                   //!< SBAS disabled: no satellites
#define SBAS_EPHEMERIS_ARRAY_SIZE 0     //!< SBAS disabled: no ephemeris storage needed
#endif

#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSIRN+NSYSLEO) //!< number of enabled navigation systems

/**
 * Number of estimated parameters for point-positioning:
 *  - (3) ECEF receiver position (m)
 *  - (1) GPS receiver clock bias (expressed in meters, i.e. multiplied by speed of light)
 *  - (1) GLONASS-GPS time offset (expressed in meters)
 *  - (1) Galileo-GPS time offset (expressed in meters)
 *  - (1) BeiDou-GPS time offset (expressed in meters)
 */
#define NX 7

#define MAXERRMSG 0     //!< Max error message length (currently unused/reserved)

#define GPS_EPHEMERIS_ARRAY_SIZE (NSATGPS + NSATGAL + NSATQZS + NSATCMP)  //!< Ephemeris storage sized for GPS + the constellations sharing its broadcast ephemeris format (Galileo, QZSS, BeiDou)
#define GLONASS_EPHEMERIS_ARRAY_SIZE (NSATGLO)  //!< GLONASS ephemeris array size (GLONASS uses a distinct ephemeris format, stored separately)

/** Raw-observation data-type discriminator, identifying which member of a raw GNSS message union is populated. */
#define DATA_TYPE_NONE 0                 //!< No data / uninitialized
#define DATA_TYPE_OBSERVATION 1          //!< Pseudorange/carrier-phase observation data (obsd_t[])
#define DATA_TYPE_EPHEMERIS 2            //!< Broadcast ephemeris data
#define DATA_TYPE_SBS 3                  //!< SBAS message data
#define DATA_TYPE_ANTENNA_POSITION 5     //!< Base station position / antenna information
#define DATA_TYPE_DGPS 7                 //!< DGPS correction data
#define DATA_TYPE_ION_UTC_ALMANAC 9      //!< Ionosphere model, UTC, and almanac data
#define DATA_TYPE_SSR 10                 //!< State-space representation (SSR) correction data
#define DATA_TYPE_LEX 31                 //!< QZSS LEX message data
#define DATA_TYPE_CONTINUE 999           //!< Sentinel: more data of the same type follows (multi-part message)
#define DATA_TYPE_ERROR -1               //!< Sentinel: data parsing/decoding error

/** Navigation-system bitmask values, usable individually or OR'd together to select a set of systems. */
#define SYS_NONE    0x00                //!< navigation system: none
#define SYS_GPS     0x01                //!< navigation system: GPS
#define SYS_SBS     0x02                //!< navigation system: SBAS
#define SYS_GLO     0x04                //!< navigation system: GLONASS
#define SYS_GAL     0x08                //!< navigation system: Galileo
#define SYS_QZS     0x10                //!< navigation system: QZSS
#define SYS_CMP     0x20                //!< navigation system: BeiDou
#define SYS_IRN     0x40                //!< navigation system: IRNSS
#define SYS_LEO     0x80                //!< navigation system: LEO
#define SYS_ALL     0xFF                //!< navigation system: all


/**
 * On bare-metal ARM targets (non-Zephyr), stub out the Linux pthread/dirent types and macros that
 * RTKLIB code references, so that code doesn't need to be #if-guarded everywhere it appears. The
 * mutex macros expand to nothing since there is no threading to guard against on these targets.
 */
#if PLATFORM_IS_ARM
#ifndef __ZEPHYR__

typedef uint32_t pthread_t;         //!< Stub: unused on bare-metal ARM
typedef uint32_t pthread_mutex_t;   //!< Stub: unused on bare-metal ARM
typedef int lock_t;                 //!< Stub: unused on bare-metal ARM
typedef int DIR;                    //!< Stub: unused on bare-metal ARM
struct dirent { char* d_name; };    //!< Stub: unused on bare-metal ARM

#define pthread_mutex_init(f, f2)   //!< Stub: no-op, no threading on bare-metal ARM
#define pthread_mutex_lock(f)       //!< Stub: no-op, no threading on bare-metal ARM
#define pthread_mutex_unlock(f)     //!< Stub: no-op, no threading on bare-metal ARM

#endif
#endif // ARM

#endif // __RTK_EMBEDDED_DEFINES_H_
