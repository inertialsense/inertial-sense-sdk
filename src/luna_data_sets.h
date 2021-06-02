/*
Copyright 2014-2021 Inertial Sense, Inc. - http://inertialsense.com
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
#define DID_EVB_LUNA_REMOTE_KILL        (eDataIDs)113 /** (evb_luna_remote_kill_t) EVB remoteKill system */
#define DID_EVB_LUNA_WHEEL_CONTROLLER   (eDataIDs)114 /** (evb_luna_wheel_controller_t) EVB wheel control information */
#define DID_EVB_LUNA_WHEEL_COMMAND      (eDataIDs)115 /** (evb_luna_wheel_command_t) EVB velocity command */
#define DID_LUNA_COUNT					116				/** Make larger than all Luna DIDs */


PUSH_PACK_1



typedef enum
{
    EVB_LUNA_CFG_BITS_ENABLE_SS_GEOFENCE   			    = 0x00000001,
    EVB_LUNA_CFG_BITS_ENABLE_SS_BUMP            	    = 0x00000002,
    EVB_LUNA_CFG_BITS_ENABLE_SS_PROXIMITY       	    = 0x00000004,
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL              = 0x00000100,	// On vehicle
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_1     = 0x00000200,	// External buttons
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_2	    = 0x00000400,	// No external buttons
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_MASK  = 0x00000600,
} eEvbLunaFlashCfgBits;

#define NUM_FF_COEFS	2
#define NUM_AL_COEFS	5

typedef struct
{
	/** Timeout period before motors disable is triggered.*/
	uint32_t				cmdTimeoutMs;
  
	/** Commanded velocity slew rate (rad/s/s) */
	float					slewRate;

	/** Commanded velocity max (rad/s) */
	float					velMax;

	/** Feadforward deadband (m/s) */
	float					FF_vel_deadband;

	/** Feadforward C0 estimate integral gain */
	float					FF_c0_est_Ki;

    /** Feadforward C0 estimate maximum value (rad/s) */
    float                 	FF_c0_est_max;

    /** Feadforward C0 and C1 coefficients */
    float                 	FF_c_l[NUM_FF_COEFS];
    float                 	FF_c_r[NUM_FF_COEFS];

	/** Feadback proportional gain */
	float					FB_Kp;

	/** Feadback derivative gain */
	float					FB_Kd;

    /** EVB2 velocity Linearization Coefficients */
    float                   LinearCoEff[NUM_AL_COEFS];

	/** Actuator counts per radian velocity controller */
	float					actuatorEncoderCountsPerRad;

    /** Actuator count [min, max] Duty cycle will drive between these numbers. Duty of 0 - min, duty of 100 - Max */
    int32_t                 actuatorEncoderRange[2];

    /** (rad) Angle that sets actuator zero velocity (center) position relative to home point. */
    float                	actuatorTrim_l;
    float                	actuatorTrim_r;

    /** (rad) Control effort angle (transmission angle) from zero required before wheels actually start spinning. */
    float                   actuatorDeadbandAngle;

    /** (rpm) Engine RPM corresponding with control gains. */
    float                   FF_FB_engine_rpm;

    /** (rpm) Current engine RPM.  Used for wheel control gain scheduling. */
    float                   engine_rpm;

	/** Test sweep rate (rad/s/s) */
	float					testSweepRate;

} evb_luna_wheel_control_cfg_t;

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
	
    /** Config bits (see eEvbLunaFlashCfgBits) */
    uint32_t                bits;

	/** Geofence Min Latitude **/
	double                  minLatGeofence;
	
	/** Geofence Max Latitude **/
    double                  maxLatGeofence;

	/** Geofence Min Latitude **/
	double                  minLonGeofence;

    /** Geofence Max Longitude **/
    double                  maxLonGeofence;
	
	/** Timeout period before motors disable is triggered.*/
	uint32_t				remoteKillTimeoutMs;

    /** Bump detection threshhold */
    float                   bumpSensitivity;

    /** Proximity threshhold for prox error.*/
    float                   minProxDistance;

	/** Wheel velocity control */
	evb_luna_wheel_control_cfg_t wheelControl;

} evb_luna_flash_cfg_t;

/**
* (DID_EVB_LUNA_STATUS) EVB-2 Luna status.
*/
typedef struct
{
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t               	timeOfWeekMs;
	
	/** Status (eEvbLunaStatus) Reset faults by setting these to zero */
	uint32_t                evbLunaStatus;

	/** Motor state (eLunaMotorState) */
	uint32_t                motorState;

	/** Remotekill mode (eLunaRemoteKillMode) */
	uint32_t                remoteKillMode;

	/** Supply voltage (V) */
	float                   supplyVoltage;

} evb_luna_status_t;

typedef enum
{
    /** Motor command timeout */
    EVB_LUNA_STATUS_WHEEL_CMD_TIMEOUT      		        = 0x00000001,

	/** Geofence boundary exceeded */
	EVB_LUNA_STATUS_ERR_GEOFENCE_EXCEEDED  		        = 0x00000002,
		
	/** Remote kill - motors disabled by remote kill */
	EVB_LUNA_STATUS_ERR_REMOTE_KILL				        = 0x00000004,
	
	/** Emergency stop button */
	EVB_LUNA_STATUS_ERR_ESTOP					        = 0x00000008,
		
	/** Bump sensor */
	EVB_LUNA_STATUS_ERR_BUMP					        = 0x00000010,
	
	/** Range Sensor */
	EVB_LUNA_STATUS_ERR_PROXIMITY				        = 0x00000020,

    /** EVB Error bit mask */
    EVB_LUNA_STATUS_ERR_MASK                            = 0x00000FFF,

    /**Controller not tracking heading*/
    COYOTE_STATUS_ERR_TRACKING_HEADING                  = 0x00001000,

    /**Controller not tracking ang vel*/
    COYOTE_STATUS_ERR_TRACKING_ANG_VEL                  = 0x00002000,

    /**Controller not approaching waypoint*/
    COYOTE_STATUS_ERR_TRACKING_WAYPOINT                 = 0x00004000,

    /**Controller tracking error bit mask*/
    COYOTE_STATUS_ERR_TRACKING_BITMASK                  = 0x0000F000,
	
    /** Error Mask */
    COYOTE_STATUS_ERR_MASK                              = 0x0000FFFF,

    /**OGM Blocked */
    COYOTE_STATUS_AUTONOMOUS_OGM_BLOCK                  = 0x00010000,

    /**Paused in autonomous mode */
    COYOTE_STATUS_AUTONOMOUS_PAUSE                      = 0x00020000,

    /**Full Mask */
    COYOTE_STATUS_MASK                                  = 0x000FFFFF,

} eCoyoteStatus;

typedef enum
{
	LMS_UNSPECIFIED					= 0,
	LMS_MOTOR_ENABLE				= 1,
	LMS_MOTOR_DISABLE				= 2,	
} eLunaMotorState;

typedef enum
{
	LRKM_UNSPECIFIED				= 0,
	LRKM_ENABLE						= 1,	// Keep alive motors enabled.
	LRKM_DISABLE					= 2,	// Disable motors.
	LRKM_PAUSE						= 3,	// Keep alive motors paused.
	LRKM_DISARM 					= 4,	// Turn off remote kill and then switch to LMS_MOTOR_ENABLE.
} eLunaRemoteKillMode;

/**
* (DID_EVB_LUNA_SENSORS) EVB-2 Luna sensors (proximity, etc.).
*/
typedef struct
{
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;
	
	/*Proximity Sensory distance measurement arrays*/
    float                   proxSensorOutput[9];
	
	int32_t					bumpEvent;

} evb_luna_sensors_t;


/**
* (DID_EVB_LUNA_REMOTE_KILL) EVB Luna Remote Kill system.
*/
typedef struct
{
	/** (eLunaRemoteKillMode) */
	int32_t					mode;

} evb_luna_remote_kill_t;

/**
* (DID_EVB_LUNA_WHEEL_CMD) EVB Luna wheel command.
*/
typedef struct
{
	/** Local system time in milliseconds */
	uint32_t                timeMs;

	/** Control mode (see eLunaWheelControllerMode) */
	uint32_t                mode;

	/** Left wheel velocity */
	float					wheel_l;

	/** Right wheel velocity */
	float					wheel_r;

} evb_luna_wheel_command_t;


typedef enum
{
	LCM_DISABLED					= 0,
	LCM_STOP						= 1,
	LCM_ENABLE						= 2,	// With watchdog
	// Velocity TESTS
	LCM_TEST_VEL_DUAL_CMD			= 3,	// Use left vel cmd to drive left and right together
	LCM_TEST_VEL_CMD				= 4,
	LCM_TEST_VEL_SWEEP				= 5,
	// Effort TESTS
	LCM_TEST_EFFORT					= 6,	// (Keep as first effort test)
	// Duty TESTS	
	LCM_TEST_DUTY					= 7,	// (Keep as first duty cycle test)
	LCM_TEST_DUTY_SWEEP				= 8,	// Watchdog disabled in testing
} eLunaWheelControllerMode;

typedef enum
{
	LCS_FAULT_L						= 0x00000001,
	LCS_FAULT_R						= 0x00000002,
	LCS_VEL_CMD_LIMITED_L			= 0x00000010,
	LCS_VEL_CMD_LIMITED_R			= 0x00000020,
	LCS_VEL_CMD_SLEW_LIMITED_L		= 0x00000040,
	LCS_VEL_CMD_SLEW_LIMITED_R		= 0x00000080,
	LCS_VEL_LIMITED_L_MASK			= (LCS_VEL_CMD_LIMITED_L | LCS_VEL_CMD_SLEW_LIMITED_L),
	LCS_VEL_LIMITED_R_MASK			= (LCS_VEL_CMD_LIMITED_R | LCS_VEL_CMD_SLEW_LIMITED_R),
} eLunaWheelControllerStatus;

/**
* (DID_EVB_LUNA_WHEEL_CONTROLLER) EVB Luna wheel controller info.
*/
typedef struct
{
	/** Local system time in milliseconds */
	uint32_t                timeMs;

	/** Delta time */
	float                	dt;

	/** Wheel control mode: (see eLunaWheelControllerMode) */
	uint32_t            	mode;

	/** Wheel control status (see eLunaWheelControllerStatus) */
	uint32_t            	status;

	/** Velocity commanded */
	float 					velCmd_l;
	float 					velCmd_r;

	/** Velocity commanded after slew rate */
	float 					velCmdSlew_l;
	float 					velCmdSlew_r;

	/** Velocity */
	float 					vel_l;
	float 					vel_r;

	/** Velocity error */
	float 					velErr_l;
	float 					velErr_r;

	/** Feedforward control effort (rad) */
	float 					ff_eff_l;
	float 					ff_eff_r;

	/** Feedback control effort (rad) */
	float 					fb_eff_l;
	float 					fb_eff_r;

	/** Control effort at transmission (rad) */
	float 					eff_l;
	float 					eff_r;

	/** Control effort at actuator (rad) */
	float 					effAct_l;
	float 					effAct_r;

	/** Feedback control effort */
	uint32_t 				effDuty_l;
	uint32_t 				effDuty_r;

} evb_luna_wheel_controller_t;


POP_PACK

#ifdef __cplusplus
}
#endif

#endif // LUNA_DATA_SETS_H
