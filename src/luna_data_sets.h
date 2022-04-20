/*
Copyright 2014-2022 Inertial Sense, Inc. - http://inertialsense.com
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
#define DID_EVB_LUNA_WHEEL_CONTROLLER   (eDataIDs)114 /** (evb_luna_velocity_control_t) EVB wheel control information */
#define DID_EVB_LUNA_VELOCITY_COMMAND   (eDataIDs)115 /** (evb_luna_velocity_command_t) EVB velocity command */
#define DID_EVB_LUNA_AUX_COMMAND        (eDataIDs)116 /** (evb_luna_aux_command_t) EVB auxillary commands */
#define DID_LUNA_COUNT					117				/** Make larger than all Luna DIDs */


PUSH_PACK_1



typedef enum
{
    EVB_LUNA_CFG_BITS_ENABLE_SS_GEOFENCE                = 0x00000001,
    EVB_LUNA_CFG_BITS_ENABLE_SS_BUMP_ADC                = 0x00000002,
    EVB_LUNA_CFG_BITS_ENABLE_SS_PROXIMITY               = 0x00000004,
	EVB_LUNA_CFG_BITS_ENABLE_SS_BUMP_I2C                = 0x00000008,
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL              = 0x00000100,	// On vehicle
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_1     = 0x00000200,	// External buttons
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_2	    = 0x00000400,	// No external buttons
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_MASK  = 0x00000600,
} eEvbLunaFlashCfgBits;


typedef enum
{
	EVB_WHEEL_CONTROL_CONFIG_TYPE_UNDEFINED             = 0,
	EVB_WHEEL_CONTROL_CONFIG_TYPE_HOVERBOT              = 1,
	EVB_WHEEL_CONTROL_CONFIG_TYPE_ZERO_TURN             = 2,
	EVB_WHEEL_CONTROL_CONFIG_TYPE_PWM                   = 3,
	EVB_WHEEL_CONTROL_CONFIG_TYPE_Z1R					= 4,
	EVB_WHEEL_CONTROL_CONFIG_TYPE_MASK                  = 0x00000007,
} eEvbLunaWheelControlConfig_t;


typedef struct
{
	/** Forward Velocity Feedback proportional gain */
	float					u_FB_Kp;

	/** Turn Rate Feedback proportional gain */
	float					w_FB_Kp;

    /** Turn Rate Feedforward */
    float                 	w_FF;

} evb_luna_velocity_control_vehicle_cfg_t;

#define NUM_FF_COEFS	2
#define NUM_AL_COEFS	5

typedef struct
{
	/** Commanded velocity slew rate (rad/s/s) */
	float					slewRate;

	/** Commanded velocity max (rad/s) */
	float					velMax;

	/** Feedforward deadband (m/s) */
	float					FF_vel_deadband;

	/** Feedforward coefficient estimation gain.  Zero to disable estimation..  Zero to disable estimation..  Zero to disable estimation. */
	float					FF_c_est_Ki[NUM_FF_COEFS];

    /** Feedforward coefficient estimation maximum value */
    float                 	FF_c_est_max[NUM_FF_COEFS];

    /** Feedforward coefficients */
    float                 	FF_c[NUM_FF_COEFS];

    /** (rpm) Engine RPM corresponding with control gains. */
    float                   FF_FB_engine_rpm;

	/** Feedback proportional gain */
	float					FB_Kp;

    /** Feedback integral gain */
    float                   FB_Ki;

	/** Feedback derivative gain */
	float					FB_Kd;

    /** EVB2 velocity Linearization Coefficients */
    float                   InversePlant_l[NUM_AL_COEFS];
    float                   InversePlant_r[NUM_AL_COEFS];

    /** Sets actuator zero velocity (center) position relative to home point. */
    float                	actuatorTrim_l;
    float                	actuatorTrim_r;

    /** Limits for actuator angle. */
    float                	actuatorLimits_l[2];
    float                	actuatorLimits_r[2];

    /** Control effort from zero (trim) before wheels start spinning. */
    float                   actuatorDeadbandDuty_l;
    float                   actuatorDeadbandDuty_r;
    float                   actuatorDeadbandVel;

	/** Test sweep rate (rad/s) */
	float					testSweepRate;

} evb_luna_velocity_control_wheel_cfg_t;

typedef struct
{
	/** Various config like motor control types, etc. (eEvbLunaWheelControlConfig_t) */
	uint32_t				config;

	/** Timeout period before motors disable is triggered.*/
	uint32_t				cmdTimeoutMs;

	/** (m) Wheel radius */
	float 					wheelRadius;

	/** (m) Wheel baseline, distance between wheels */
	float					wheelBaseline;

    /** (rpm) Current engine RPM.  Used for wheel control gain scheduling. */
    float                   engine_rpm;

	/** Wheel control */
	evb_luna_velocity_control_wheel_cfg_t       wheel;

	/** Vehicle control */
	evb_luna_velocity_control_vehicle_cfg_t     vehicle;

} evb_luna_velocity_control_cfg_t;

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

	/** Velocity control */
	evb_luna_velocity_control_cfg_t velControl;

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
    EVB_LUNA_STATUS_WHEEL_CMD_TIMEOUT                   = 0x00000001,

	/** Geofence boundary exceeded */
	EVB_LUNA_STATUS_ERR_GEOFENCE_EXCEEDED               = 0x00000002,
		
	/** Remote kill - motors disabled by remote kill */
	EVB_LUNA_STATUS_ERR_REMOTE_KILL                     = 0x00000004,
	
	/** Emergency stop button */
	EVB_LUNA_STATUS_ERR_ESTOP                           = 0x00000008,
		
	/** Bump sensor mask */
	EVB_LUNA_STATUS_ERR_BUMP_MASK                       = 0x000000F0,

	/** Bump sensor front */
	EVB_LUNA_STATUS_ERR_BUMP_FRONT                      = 0x00000010,
	
	/** Bump sensor back */
	EVB_LUNA_STATUS_ERR_BUMP_BACK                       = 0x00000020,
	
	/** Bump sensor left */
	EVB_LUNA_STATUS_ERR_BUMP_LEFT                       = 0x00000040,

	/** Bump sensor right */
	EVB_LUNA_STATUS_ERR_BUMP_RIGHT                      = 0x00000080,
	
	/** Range Sensor */
	EVB_LUNA_STATUS_ERR_PROXIMITY                       = 0x00000100,

    /** EVB Error bit mask.  Errors in this mask will stop control. */
    EVB_LUNA_STATUS_ERR_MASK                            = 0x00000FFF,

	/** Wheel encoder fault */
	EVB_LUNA_STATUS_FAULT_WHEEL_ENCODER                 = 0x00001000,

	/** Bump sensor not communicating */
	EVB_LUNA_STATUS_FAULT_BUMP_SENSOR_COM               = 0x00002000,

	/** Mower blade on */
	EVB_LUNA_STATUS_MOWER_BLADE_ON						= 0x00010000,

	/** Axis is in an invalid state */
	EVB_LUNA_STATUS_AXIS_ERR_INVALID_STATE				= 0x01000000,
	
	/** Watchdog has expired */
	EVB_LUNA_STATUS_AXIS_ERR_WATCHDOG					= 0x02000000,
	
	/** Motor or driver temperature is above limits */
	EVB_LUNA_STATUS_AXIS_ERR_TEMP						= 0x04000000,
	
	/** Axis error bit mask */ 
	EVB_LUNA_STATUS_AXIS_ERR_MASK						= 0xFF000000,

} eEvbLunaStatus;

typedef enum
{
	LMS_UNSPECIFIED					= 0,
	LMS_MOTOR_CONTROL_ENABLE		= 1,	// Motor control enabled.
	LMS_MOTOR_CONTROL_DISABLE		= 2,	// Motor control disabled.  Engine shutoff is controlled only by remote kill.
} eLunaMotorState;

typedef enum
{
	LRKM_UNSPECIFIED				= 0,
	LRKM_ENABLE						= 1,	// Keep alive motors enabled.
	LRKM_DISABLE					= 2,	// Disable motors.
	LRKM_PAUSE						= 3,	// Keep alive motors paused.
	LRKM_DISARM 					= 4,	// Turn off remote kill and then switch to LMS_MOTOR_CONTROL_ENABLE.
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
* (DID_EVB_LUNA_VELOCITY_COMMAND) EVB Luna wheel command.
*/
typedef struct
{
	/** Local system time in milliseconds */
	uint32_t                timeMs;

	/** Control mode (see eLunaWheelControllerMode) */
	uint32_t                modeCmd;

	/** Forward velocity (m/s) */
	float					fwd_vel;

	/** Turn rate (rad/s) */
	float					turn_rate;

} evb_luna_velocity_command_t;

typedef struct evb_luna_aux_command_t
{
    uint32_t                    command;	// (see eLunaAuxCommands)

}evb_luna_aux_command_t;

typedef enum
{
    //Possible Commands
    AUX_CMD_BLADE_OFF           = 0,
    AUX_CMD_BLADE_ON            = 1,
    AUX_CMD_EBRAKE_ENGAGE       = 2,
    AUX_CMD_EBRAKE_DISENGAGE    = 3,
    AUX_CMD_BEEP                = 4,
    // AUX_CMD_DIDS_LOG_START      = 5,		// Used in inertial_sense_ros node
    // AUX_CMD_DIDS_LOG_STOP       = 6,
} eLunaAuxCommands;

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
	LCM_TEST_WHL_ANG_VEL_SWEEP      = 9,
} eLunaWheelControllerMode;

typedef enum
{
	LCS_FAULT_L						= 0x00000001,
	LCS_FAULT_R						= 0x00000002,
	LCS_VEL_CMD_LIMITED_L			= 0x00000010,
	LCS_VEL_CMD_LIMITED_R			= 0x00000020,
	LCS_VEL_CMD_LIMITED_MASK		= (LCS_VEL_CMD_LIMITED_L | LCS_VEL_CMD_LIMITED_R),
	LCS_VEL_CMD_SLEW_LIMITED_L		= 0x00000040,
	LCS_VEL_CMD_SLEW_LIMITED_R		= 0x00000080,
	LCS_VEL_LIMITED_L_MASK			= (LCS_VEL_CMD_LIMITED_L | LCS_VEL_CMD_SLEW_LIMITED_L),
	LCS_VEL_LIMITED_R_MASK			= (LCS_VEL_CMD_LIMITED_R | LCS_VEL_CMD_SLEW_LIMITED_R),
} eLunaWheelControllerStatus;

typedef struct
{
	/** Vehicle forward and angular velocity, Commanded (m/s, rad/s) */
	float 					velCmd_f;
	float 					velCmd_w;

	/** Vehicle forward and angular velocity (m/s, rad/s) */
	float 					vel_f;
	float 					vel_w;

} evb_luna_velocity_control_vehicle_t;

typedef struct
{
	/** Wheel velocity, Commanded (rad/s) */
	float 					velCmd_l;
	float 					velCmd_r;

	/** Wheel velocity commanded after slew rate (rad/s) */
	float 					velCmdSlew_l;
	float 					velCmdSlew_r;

	/** Wheel velocity (rad/s) */
	float 					vel_l;
	float 					vel_r;

	/** Wheel velocity error (rad/s) */
	float 					velErr_l;
	float 					velErr_r;

	/** Feedforward control effort */
	float 					ff_eff_l;
	float 					ff_eff_r;

	/** Feedback control effort */
	float 					fb_eff_l;
	float 					fb_eff_r;

	/** Control effort = ff_eff_x + fb_eff_x */
	float 					eff_l;
	float 					eff_r;

	/** Control effort intermediate */
	float 					effInt_l;
	float 					effInt_r;

	/** Duty cycle control effort at actuator (-1.0 to 1.0) */
	float 					effDuty_l;
	float 					effDuty_r;

} evb_luna_velocity_control_wheel_t;

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
	uint32_t            	current_mode;

	/** Wheel control status (see eLunaWheelControllerStatus) */
	uint32_t            	status;

	/** Vehicle velocity control */
	evb_luna_velocity_control_vehicle_t     vehicle;

	/** Wheel velocity control */
	evb_luna_velocity_control_wheel_t       wheel;

} evb_luna_velocity_control_t;


POP_PACK

#ifdef __cplusplus
}
#endif

#endif // LUNA_DATA_SETS_H
