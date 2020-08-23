/*
Copyright 2014-2020 Inertial Sense, Inc. - http://inertialsense.com
*/

#ifndef LUNA_DATA_SETS_H
#define LUNA_DATA_SETS_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "ISConstants.h"

#ifdef __cplusplus
extern "C" {
#endif


#define DID_EVB_LUNA_FLASH_CFG          (eDataIDs)110 /** (evb_luna_flash_cfg_t) EVB Luna configuration. */
#define DID_EVB_LUNA_STATUS             (eDataIDs)111 /** (evb_luna_status_t) EVB Luna status. */
#define DID_EVB_LUNA_SENSORS            (eDataIDs)112 /** (evb_luna_sensors_t) EVB Luna sensors (proximity, etc.). */


PUSH_PACK_1

/**
* (DID_EVB_LUNA_FLASH_CFG) EVB-2 Luna specific flash config.
* This can be up to 4096 bytes in size.  If more is needed, we can adjust the IS SDK EVB-2 project to allocate more.
*/
typedef struct
{  
    /** Size of this struct */
    uint32_t				size;

    /** Checksum, excluding size and checksum */
    uint32_t                checksum;

    /** Manufacturer method for restoring flash defaults */
    uint32_t                key;

    /**Geofence Max Latitude**/
    double                  maxLatGeofence;

    /**Geofence Min Latitude**/
    double                  minLatGeofence;
    
    /**Geofence Max Longitude**/
    double                  maxLonGeofence;
    
    /**Geofence Min Latitude**/
    double                  minLonGeofence;

} evb_luna_flash_cfg_t;


/**
* (DID_EVB_LUNA_STATUS) EVB-2 Luna status.
*/
typedef struct
{
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

} evb_luna_status_t;


/**
* (DID_EVB_LUNA_SENSORS) EVB-2 Luna sensors (proximity, etc.).
*/
typedef struct
{
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

} evb_luna_sensors_t;

POP_PACK



#ifdef __cplusplus
}
#endif

#endif // LUNA_DATA_SETS_H
