/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef GPX_DATA_SETS_H
#define GPX_DATA_SETS_H

#include <stdint.h>
#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

// *****************************************************************************
// ****** InertialSense binary message Data Identification Numbers (DIDs) ****** 
// ******                                                                 ******
// ****** NEVER REORDER THESE VALUES!                                     ******
// *****************************************************************************
/** Data identifiers - these are unsigned int and #define because enum are signed according to C standard */
typedef uint32_t eDataIDs;

#define GPX_DID_GNSS1_POS			(eDataIDs)120	/** (gps_pos_t) Raw position from receiver, unless RTK is active - in which case the corrections are applied */
#define GPX_DID_GNSS1_VEL			(eDataIDs)121	/** (gps_vel_t) Raw velocity from receiver, unless RTK is active - in which case the corrections are applied */
#define GPX_DID_GNSS1_POS_RAW		(eDataIDs)122	/** (gps_pos_t) Raw position from reciever, non-RTK output */
#define GPX_DID_GNSS1_VEL_RAW		(eDataIDs)123	/** (gps_vel_t) Raw velocity from reciever, non-RTK output */
#define GPX_DID_GNSS1_SAT			(eDataIDs)124	/** (gps_sat_t) Satellite info */
#define GPX_DID_GNSS1_RAW			(eDataIDs)125	/** (gps_raw_t) RTCM3 or other raw messages, see data set for supported types */
#define GPX_DID_GNSS1_VERSION		(eDataIDs)126	/** (gps_version_t) Receiver version number */

#define GPX_DID_GNSS2_POS			(eDataIDs)127	/** (gps_pos_t) Raw position from receiver, unless RTK is active - in which case the corrections are applied */
#define GPX_DID_GNSS2_VEL			(eDataIDs)128	/** (gps_vel_t) Raw velocity from receiver, unless RTK is active - in which case the corrections are applied */
#define GPX_DID_GNSS2_POS_RAW		(eDataIDs)129	/** (gps_pos_t) Raw position from reciever, non-RTK output */
#define GPX_DID_GNSS2_VEL_RAW		(eDataIDs)130	/** (gps_vel_t) Raw velocity from reciever, non-RTK output */
#define GPX_DID_GNSS2_SAT			(eDataIDs)131	/** (gps_sat_t) Satellite info */
#define GPX_DID_GNSS2_RAW			(eDataIDs)132	/** (gps_raw_t) RTCM3 or other raw messages, see data set for supported types */
#define GPX_DID_GNSS2_VERSION		(eDataIDs)133	/** (gps_version_t) Receiver version number */

#define GPX_DID_BASE_RAW			(eDataIDs)134	/** (gps_raw_t) RTCM3 or other raw messages, see data set for supported types */

#define GPX_DID_RTK_REL				(eDataIDs)134	/** (gps_rtk_rel_t) Base to rover relative info (base to GNSS1) */
#define GPX_DID_CMP_REL				(eDataIDs)135	/** (gps_rtk_rel_t) Compassing mode relative info (GNSS1 to GNSS2) */


/*
 *	We need the following data sets: 
 * 		- raw GPS (RTCM3, leave room for others) messages wrapper
 * 		- GPS position (from IMX)
 * 		- GPS velocity (from IMX)
 *		- RTK solution info
 *		- Compassing solution info
 *		- GPS satellite info (upgraded from IMX to support multifreq)
 * 		- GPS timepulse info
 * 		- RTOS info (from IMX)
 * 		- Device info (DID_DEV_INFO from IMX)
 */

/** RTOS tasks */
typedef enum
{
	/** Task 0: Communications	*/
	GPX_TASK_COMM = 0,

	/** Task 1: Nav */
//	GPX_TASK_RTK,
//
//	/** Task 2: Maintenance */
//	GPX_TASK_MAINT,
//
//	/** Task 3: Idle */
//	GPX_TASK_IDLE,
//
//	/** Task 4: Timer */
//	GPX_TASK_TIMER,

	/** Number of RTOS tasks */
	GPX_RTOS_NUM_TASKS                 // Keep last
} eGpxRtosTask;

#ifdef __cplusplus
}
#endif

#endif  // GPX_DATA_SETS_H

