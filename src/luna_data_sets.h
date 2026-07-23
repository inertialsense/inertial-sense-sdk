/**
 * @file luna_data_sets.h
 * @brief Data Identification Number (DID) payloads for the EVB-2 "Luna" autonomous-vehicle add-on board.
 *
 * Luna is an EVB-2 daughterboard/firmware profile for differential-drive autonomous vehicles (e.g.
 * robotic mowers): it adds wheel velocity control, remote-kill safety interlocks, bump/proximity
 * sensing, and geofencing on top of the standard EVB-2 DID set. As with the main data_sets.h, each
 * DID_EVB_LUNA_* macro's trailing comment names the payload type it carries, and payloads are never
 * reordered or reused since they are part of the wire protocol.
 *
 * Unless otherwise noted: linear velocities are in meters/second, angular velocities are in
 * radians/second, and positions given as latitude/longitude are in degrees.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
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


#define DID_EVB_LUNA_FLASH_CFG          (eDataIDs)110 //!< (evb_luna_flash_cfg_t) EVB Luna configuration.
#define DID_EVB_LUNA_STATUS             (eDataIDs)111 //!< (evb_luna_status_t) EVB Luna status.
#define DID_EVB_LUNA_SENSORS            (eDataIDs)112 //!< (evb_luna_sensors_t) EVB Luna sensors (proximity, etc.).
#define DID_EVB_LUNA_REMOTE_KILL        (eDataIDs)113 //!< (evb_luna_remote_kill_t) EVB remoteKill system
#define DID_EVB_LUNA_VELOCITY_CONTROL   (eDataIDs)114 //!< (evb_luna_velocity_control_t) EVB wheel control information
#define DID_EVB_LUNA_VELOCITY_COMMAND   (eDataIDs)115 //!< (evb_luna_velocity_command_t) EVB velocity command
#define DID_EVB_LUNA_AUX_COMMAND        (eDataIDs)116 //!< (evb_luna_aux_command_t) EVB auxillary commands
#define DID_LUNA_COUNT                  (eDataIDs)117                //!< Sentinel: made larger than all Luna DIDs


PUSH_PACK_1



/** Config bits for evb_luna_flash_cfg_t::bits: enables individual Luna subsystems. */
typedef enum
{
    EVB_LUNA_CFG_BITS_ENABLE_SS_GEOFENCE                = 0x00000001,   //!< Enable geofence boundary enforcement
    EVB_LUNA_CFG_BITS_ENABLE_SS_BUMP_ADC                = 0x00000002,   //!< Enable analog (ADC) bump sensors
    EVB_LUNA_CFG_BITS_ENABLE_SS_PROXIMITY               = 0x00000004,   //!< Enable proximity (range) sensors
    EVB_LUNA_CFG_BITS_ENABLE_SS_BUMP_I2C                = 0x00000008,   //!< Enable I2C bump sensors
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL              = 0x00000100,   //!< Enable remote-kill subsystem (on vehicle)
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_1     = 0x00000200,   //!< Remote-kill client 1: external buttons
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_2     = 0x00000400,   //!< Remote-kill client 2: no external buttons
    EVB_LUNA_CFG_BITS_ENABLE_SS_REMOTEKILL_CLIENT_MASK  = 0x00000600,   //!< Mask of both remote-kill client bits
} eEvbLunaFlashCfgBits;


/** Wheel-motor control scheme, selected by evb_luna_velocity_control_cfg_t::config (masked by TYPE_MASK). */
typedef enum
{
    EVB_WHEEL_CONTROL_CONFIG_TYPE_UNDEFINED             = 0,             //!< Not configured
    EVB_WHEEL_CONTROL_CONFIG_TYPE_HOVERBOT              = 1,             //!< Hoverboard-motor-derived control
    EVB_WHEEL_CONTROL_CONFIG_TYPE_ZERO_TURN             = 2,             //!< Zero-turn-radius (skid-steer) control
    EVB_WHEEL_CONTROL_CONFIG_TYPE_PWM                   = 3,             //!< Direct PWM duty-cycle control
    EVB_WHEEL_CONTROL_CONFIG_TYPE_Z1R                   = 4,             //!< Z1R actuator control
    EVB_WHEEL_CONTROL_CONFIG_TYPE_MASK                  = 0x00000007,    //!< Mask isolating the control-type field
} eEvbLunaWheelControlConfig_t;


/** Vehicle-level (as opposed to per-wheel) velocity-control tuning parameters. */
typedef struct
{
    float   u_min;              //!< Forward velocity, minimum (m/s)
    float   u_cruise;           //!< Forward velocity, cruise/nominal (m/s)
    float   u_max;               //!< Forward velocity, maximum (m/s)
    float   u_slewLimit;         //!< Forward velocity, commanded slew-rate limit (m/s)

    float   w_max_autonomous;    //!< Turn-rate velocity, maximum while under autonomous control (rad/s)
    float   w_max;               //!< Turn-rate velocity, maximum (rad/s)
    float   w_slewLimit;         //!< Turn-rate velocity, commanded slew-rate limit (rad/s)

    float   testSweepRate;       //!< Test sweep rate (m/s/s)

    float   u_FB_Kp;             //!< Forward velocity feedback proportional gain

    float   w_FB_Kp;             //!< Turn rate feedback proportional gain

    float   w_FB_Ki;             //!< Turn rate feedback integral gain

    float   w_FF_c0;             //!< Turn rate feedforward coefficient 0 (rad/s)
    float   w_FF_c1;             //!< Turn rate feedforward coefficient 1 (rad/s)

    float   w_FF_deadband;       //!< Turn rate feedforward deadband (rad/s)
} evb_luna_velocity_control_vehicle_cfg_t;

#define NUM_FF_COEFS    2   //!< Number of feedforward coefficients per wheel (see evb_luna_velocity_control_wheel_cfg_t)
#define NUM_AL_COEFS    5   //!< Number of actuator-linearization coefficients per wheel (see evb_luna_velocity_control_wheel_cfg_t)

/** Per-wheel velocity-control tuning parameters: feedforward/feedback gains, actuator linearization, and trim. */
typedef struct
{
    float   slewRate;                    //!< Commanded velocity slew rate (rad/s/s)

    float   velMax;                      //!< Commanded velocity max (rad/s)

    float   FF_vel_deadband;             //!< Feedforward deadband (m/s)

    float   FF_c_est_Ki[NUM_FF_COEFS];   //!< Feedforward coefficient estimation gain. Zero to disable estimation.

    float   FF_c_est_max[NUM_FF_COEFS];  //!< Feedforward coefficient estimation maximum value

    float   FF_c_l[NUM_FF_COEFS];        //!< Feedforward coefficients, left wheel
    float   FF_c_r[NUM_FF_COEFS];        //!< Feedforward coefficients, right wheel

    float   FF_FB_engine_rpm;            //!< Engine RPM corresponding with control gains (rpm)

    float   FB_Kp;                       //!< Feedback proportional gain

    float   FB_Ki;                       //!< Feedback integral gain

    float   FB_Kd;                       //!< Feedback derivative gain

    float   FB_gain_deadband;             //!< Feedback deadband (rad/s): feedback gains are linearly reduced down to FB_gain_deadband_reduction at zero
    float   FB_gain_deadband_reduction;   //!< Feedback deadband: reduce gains by this amount near zero, transitioning to full gain at and above the deadband

    float   InversePlant_l[NUM_AL_COEFS]; //!< EVB2 velocity linearization coefficients, left wheel
    float   InversePlant_r[NUM_AL_COEFS]; //!< EVB2 velocity linearization coefficients, right wheel

    float   actuatorTrim_l;      //!< Sets actuator zero-velocity (center) position relative to home point, left wheel
    float   actuatorTrim_r;      //!< Sets actuator zero-velocity (center) position relative to home point, right wheel

    float   actuatorLimits_l[2]; //!< Wheel controller effort limits for 1. actuator angle or 2. velocity (based on actuator type), left wheel
    float   actuatorLimits_r[2]; //!< Wheel controller effort limits for 1. actuator angle or 2. velocity (based on actuator type), right wheel

    float   actuatorDeadbandDuty_l; //!< Control effort from zero (trim) before the left wheel starts spinning
    float   actuatorDeadbandDuty_r; //!< Control effort from zero (trim) before the right wheel starts spinning
    float   actuatorDeadbandVel;    //!< Wheel velocity below which the actuator deadband duty is applied
} evb_luna_velocity_control_wheel_cfg_t;

/** Top-level wheel-velocity-controller configuration: motor/actuator type, wheel geometry, and vehicle/wheel tuning. */
typedef struct
{
    uint32_t                                config;         //!< Various config like motor control types, etc. (eEvbLunaWheelControlConfig_t)

    uint32_t                                cmdTimeoutMs;   //!< Timeout period before motors disable is triggered

    float                                   wheelRadius;    //!< Wheel radius (m)

    float                                   wheelBaseline;  //!< Wheel baseline, distance between wheels (m)

    float                                   engine_rpm;     //!< Current engine RPM (rpm). Used for wheel control gain scheduling.

    evb_luna_velocity_control_vehicle_cfg_t vehicle;        //!< Vehicle-level velocity control tuning

    evb_luna_velocity_control_wheel_cfg_t   wheel;          //!< Per-wheel velocity control tuning (shared by both wheels)
} evb_luna_velocity_control_cfg_t;

/**
 * @brief (DID_EVB_LUNA_FLASH_CFG) EVB-2 Luna specific flash config.
 * This can be up to 4096 bytes in size. If more is needed, we can adjust the IS SDK EVB-2 project to allocate more.
 */
typedef struct
{
    uint32_t    size;                              //!< Size of this struct

    uint32_t                        checksum;       //!< Checksum, excluding size and checksum

    uint32_t                        key;            //!< Manufacturer method for restoring flash defaults

    uint32_t                        bits;           //!< Config bits (see eEvbLunaFlashCfgBits)

    double                          minLatGeofence;  //!< Geofence minimum latitude (degrees)

    double                          maxLatGeofence;  //!< Geofence maximum latitude (degrees)

    double                          minLonGeofence;  //!< Geofence minimum longitude (degrees)

    double                          maxLonGeofence;  //!< Geofence maximum longitude (degrees)

    uint32_t                        remoteKillTimeoutMs; //!< Timeout period before motors disable is triggered

    float                           bumpSensitivity;  //!< Bump detection threshold

    float                           minProxDistance;  //!< Proximity threshold for prox error

    evb_luna_velocity_control_cfg_t velControl;       //!< Velocity control configuration
} evb_luna_flash_cfg_t;

/** @brief (DID_EVB_LUNA_STATUS) EVB-2 Luna status. */
typedef struct
{
    uint32_t                   timeOfWeekMs;    //!< GPS time of week (since Sunday morning) in milliseconds

    uint32_t                evbLunaStatus;      //!< Status (eEvbLunaStatus). Reset faults by setting these to zero.

    uint32_t                motorState;         //!< Motor state (eLunaMotorState)

    uint32_t                remoteKillMode;     //!< Remotekill mode (eLunaRemoteKillMode)

    float                   supplyVoltage;      //!< Supply voltage (V)

} evb_luna_status_t;

/** Status/error/fault bits for evb_luna_status_t::evbLunaStatus. */
typedef enum
{
    EVB_LUNA_STATUS_WHEEL_CMD_TIMEOUT       = 0x00000001,   //!< Motor command timeout

    EVB_LUNA_STATUS_ERR_GEOFENCE_EXCEEDED   = 0x00000002,   //!< Geofence boundary exceeded

    EVB_LUNA_STATUS_ERR_REMOTE_KILL         = 0x00000004,   //!< Remote kill: motors disabled by remote kill

    EVB_LUNA_STATUS_ERR_ESTOP               = 0x00000008,   //!< Emergency stop button pressed

    EVB_LUNA_STATUS_ERR_BUMP_MASK           = 0x000000F0,   //!< Mask of all bump-sensor error bits

    EVB_LUNA_STATUS_ERR_BUMP_FRONT          = 0x00000010,   //!< Bump sensor front triggered

    EVB_LUNA_STATUS_ERR_BUMP_BACK           = 0x00000020,   //!< Bump sensor back triggered

    EVB_LUNA_STATUS_ERR_BUMP_LEFT           = 0x00000040,   //!< Bump sensor left triggered

    EVB_LUNA_STATUS_ERR_BUMP_RIGHT          = 0x00000080,   //!< Bump sensor right triggered

    EVB_LUNA_STATUS_ERR_PROXIMITY           = 0x00000100,   //!< Range/proximity sensor triggered

    EVB_LUNA_STATUS_ERR_MASK                = 0x00000FFF,   //!< EVB error bit mask. Errors in this mask will stop control.

    EVB_LUNA_STATUS_FAULT_WHEEL_ENCODER     = 0x00001000,   //!< Wheel encoder fault

    EVB_LUNA_STATUS_FAULT_BUMP_SENSOR_COM   = 0x00002000,   //!< Bump sensor not communicating

    EVB_LUNA_STATUS_FAULT_ESTOP_RECENT      = 0x00004000,   //!< Estop button (or interlock) was pressed in the past 10 seconds

    EVB_LUNA_STATUS_MOWER_BLADE_ON          = 0x00010000,   //!< Mower blade on

    EVB_LUNA_STATUS_AXIS_ERR_INVALID_STATE  = 0x01000000,   //!< Axis is in an invalid state

    EVB_LUNA_STATUS_AXIS_ERR_WATCHDOG       = 0x02000000,   //!< Watchdog has expired

    EVB_LUNA_STATUS_AXIS_ERR_TEMP           = 0x04000000,   //!< Motor or driver temperature is above limits

    EVB_LUNA_STATUS_AXIS_ERR_MASK           = 0xFF000000,   //!< Mask of all axis-error bits

} eEvbLunaStatus;

/** Motor-control enable state for evb_luna_status_t::motorState. */
typedef enum
{
    LMS_UNSPECIFIED             = 0,    //!< Not specified
    LMS_MOTOR_CONTROL_ENABLE    = 1,    //!< Motor control enabled.
    LMS_MOTOR_CONTROL_DISABLE   = 2,    //!< Motor control disabled. Engine shutoff is controlled only by remote kill.
} eLunaMotorState;

/** Remote-kill safety-interlock mode for evb_luna_status_t::remoteKillMode and evb_luna_remote_kill_t::mode. */
typedef enum
{
    LRKM_UNSPECIFIED            = 0,    //!< Not specified
    LRKM_ENABLE                 = 1,    //!< Keep alive motors enabled.
    LRKM_DISABLE                = 2,    //!< Disable motors.
    LRKM_PAUSE                  = 3,    //!< Keep alive motors paused.
    LRKM_DISARM                 = 4,    //!< Turn off remote kill and then switch to LMS_MOTOR_CONTROL_ENABLE.
} eLunaRemoteKillMode;

/** @brief (DID_EVB_LUNA_SENSORS) EVB-2 Luna sensors (proximity, etc.). */
typedef struct
{
    uint32_t    timeOfWeekMs;         //!< GPS time of week (since Sunday morning) in milliseconds

    float       proxSensorOutput[9];  //!< Proximity sensor distance measurement array, one entry per sensor

    int32_t     bumpEvent;            //!< Bump event indicator (see eEvbLunaStatus bump bits); non-zero when a bump was detected
} evb_luna_sensors_t;


/** @brief (DID_EVB_LUNA_REMOTE_KILL) EVB Luna Remote Kill system. */
typedef struct
{
    int32_t     mode;    //!< Remote-kill mode (eLunaRemoteKillMode)
} evb_luna_remote_kill_t;

/** @brief (DID_EVB_LUNA_VELOCITY_COMMAND) EVB Luna wheel command. */
typedef struct
{
    uint32_t    timeMs;      //!< Local system time in milliseconds

    uint32_t    modeCmd;     //!< Control mode (see eLunaVelocityControlMode)

    float       fwd_vel;     //!< Forward velocity (m/s)

    float       turn_rate;   //!< Turn rate (rad/s)
} evb_luna_velocity_command_t;

/** @brief (DID_EVB_LUNA_AUX_COMMAND) EVB Luna auxiliary command (e.g. blade, e-brake, beep). */
typedef struct evb_luna_aux_command_t
{
    uint32_t    command;    //!< Auxiliary command to execute (see eLunaAuxCommands)
}evb_luna_aux_command_t;

/** Auxiliary commands for evb_luna_aux_command_t::command. */
typedef enum
{
    AUX_CMD_BLADE_OFF           = 0,    //!< Turn mower blade off
    AUX_CMD_BLADE_ON            = 1,    //!< Turn mower blade on
    AUX_CMD_EBRAKE_ENGAGE       = 2,    //!< Engage the electronic brake
    AUX_CMD_EBRAKE_DISENGAGE    = 3,    //!< Disengage the electronic brake
    AUX_CMD_BEEP                = 4,    //!< Sound an audible beep
    // AUX_CMD_DIDS_LOG_START      = 5,        // Used in inertial_sense_ros node
    // AUX_CMD_DIDS_LOG_STOP       = 6,
} eLunaAuxCommands;

/** Operating/test mode for evb_luna_velocity_control_t::current_mode. */
typedef enum
{
    LVC_MODE_DISABLED               = 0,     //!< Velocity control disabled
    LVC_MODE_STOP                   = 1,     //!< Commanded to stop
    LVC_MODE_ENABLE                 = 2,     //!< Normal operation, with watchdog
    LVC_MODE_MANUAL                 = 3,     //!< Manual (joystick/pot) control
    // Velocity TESTS
    LVC_MODE_TEST_VEL_VEHICLE_CMD   = 4,     //!< Test: use left vel cmd to drive left and right together
    LVC_MODE_TEST_VEL_WHEEL_CMD     = 5,     //!< Test: independent per-wheel velocity command
    LVC_MODE_TEST_VEL_SWEEP         = 6,     //!< Test: velocity sweep
    // Effort TESTS
    LVC_MODE_TEST_EFFORT            = 7,     //!< Test: open-loop control effort (keep as first effort test)
    // Duty TESTS
    LVC_MODE_TEST_DUTY              = 8,     //!< Test: duty cycle (keep as first duty cycle test)
    LVC_MODE_TEST_DUTY_SWEEP        = 9,     //!< Test: duty cycle sweep. Watchdog disabled in testing.
    LVC_MODE_TEST_WHL_ANG_VEL_SWEEP = 10,    //!< Test: wheel angular velocity sweep
    // Solve for Feedforward
    LVC_MODE_CALIBRATE_FEEDFORWARD  = 11,    //!< Solve for feedforward coefficients
} eLunaVelocityControlMode;

/** Status/limiting bits for evb_luna_velocity_control_t::status. */
typedef enum
{
    LVC_STATUS_FAULT_L                  = 0x00000001,   //!< Left wheel fault
    LVC_STATUS_FAULT_R                  = 0x00000002,   //!< Right wheel fault
    LVC_STATUS_VEL_CMD_LIMITED_L        = 0x00000010,   //!< Left wheel velocity command was limited
    LVC_STATUS_VEL_CMD_LIMITED_R        = 0x00000020,   //!< Right wheel velocity command was limited
    LVC_STATUS_VEL_CMD_LIMITED_MASK     = (LVC_STATUS_VEL_CMD_LIMITED_L | LVC_STATUS_VEL_CMD_LIMITED_R),   //!< Mask of both wheels' velocity-limited bits
    LVC_STATUS_VEL_CMD_SLEW_LIMITED_L   = 0x00000040,   //!< Left wheel velocity command was slew-rate limited
    LVC_STATUS_VEL_CMD_SLEW_LIMITED_R   = 0x00000080,   //!< Right wheel velocity command was slew-rate limited
    LVC_STATUS_VEL_LIMITED_L_MASK       = (LVC_STATUS_VEL_CMD_LIMITED_L | LVC_STATUS_VEL_CMD_SLEW_LIMITED_L),   //!< Mask of left wheel's velocity- and slew-limited bits
    LVC_STATUS_VEL_LIMITED_R_MASK       = (LVC_STATUS_VEL_CMD_LIMITED_R | LVC_STATUS_VEL_CMD_SLEW_LIMITED_R),   //!< Mask of right wheel's velocity- and slew-limited bits
    LVC_STATUS_VEL_CMD_SLEW_LIMITED_F   = 0x00010000,   //!< Vehicle forward velocity command was slew-rate limited
    LVC_STATUS_VEL_CMD_SLEW_LIMITED_W   = 0x00020000,   //!< Vehicle turn-rate command was slew-rate limited
    LVC_STATUS_VEL_CMD_MANUAL_INPUT     = 0x00040000,   //!< Velocity command is coming from manual input
} eLunaVelocityControlStatus;

/** Vehicle-level (forward + turn-rate) velocity control state: command, slew-limited command, actual, error, and effort. */
typedef struct
{
    float       velCmd_f;        //!< Vehicle forward velocity, commanded (m/s)
    float       velCmd_w;        //!< Vehicle angular (turn-rate) velocity, commanded (rad/s)

    float       velCmdMnl_f;     //!< Vehicle forward velocity, manually commanded (m/s)
    float       velCmdMnl_w;     //!< Vehicle angular (turn-rate) velocity, manually commanded (rad/s)

    float       velCmdSlew_f;    //!< Vehicle forward velocity, slew-rate-limited commanded (m/s)
    float       velCmdSlew_w;    //!< Vehicle angular (turn-rate) velocity, slew-rate-limited commanded (rad/s)

    float       vel_f;           //!< Vehicle forward velocity, actual (m/s)
    float       vel_w;           //!< Vehicle angular (turn-rate) velocity, actual (rad/s)

    float       err_f;           //!< Vehicle forward velocity, error (m/s)
    float       err_w;           //!< Vehicle angular (turn-rate) velocity, error (rad/s)

    float       eff_f;           //!< Vehicle forward velocity, control effort (m/s)
    float       eff_w;           //!< Vehicle angular (turn-rate) velocity, control effort (rad/s)
} evb_luna_velocity_control_vehicle_t;

/** Per-wheel velocity control state: command, actual, error, and feedforward/feedback/total control effort. */
typedef struct
{
    float       velCmd;          //!< Wheel velocity, commanded (rad/s)

    float       velCmdSlew;      //!< Wheel velocity commanded after slew rate limiting (rad/s)

    float       vel;             //!< Wheel velocity, actual (rad/s)

    float       err;             //!< Wheel velocity error (rad/s)

    float       ff_eff;          //!< Feedforward control effort

    float       fb_eff;          //!< Feedback control effort

    float       fb_eff_integral; //!< Feedback integral control effort

    float       eff;             //!< Total control effort = ff_eff + fb_eff

    float       effInt;          //!< Control effort, intermediate (pre-limiting) value

    float       effDuty;         //!< Duty cycle control effort at actuator (-1.0 to 1.0)
} evb_luna_velocity_control_wheel_t;

/** @brief (DID_EVB_LUNA_VELOCITY_CONTROL) EVB Luna wheel controller info. */
typedef struct
{
    uint32_t                                timeMs;     //!< Local system time in milliseconds

    float                                   dt;          //!< Delta time since the previous update (seconds)

    uint32_t                                status;      //!< Wheel control status (see eLunaVelocityControlStatus)

    uint32_t                                current_mode; //!< Wheel control mode (see eLunaVelocityControlMode)

    evb_luna_velocity_control_vehicle_t     vehicle;     //!< Vehicle-level velocity control state

    evb_luna_velocity_control_wheel_t       wheel_l;     //!< Left wheel velocity control state
    evb_luna_velocity_control_wheel_t       wheel_r;     //!< Right wheel velocity control state

    float                                   potV_l;      //!< Manual control input potentiometer voltage, left wheel (V)
    float                                   potV_r;      //!< Manual control input potentiometer voltage, right wheel (V)
} evb_luna_velocity_control_t;


POP_PACK

#ifdef __cplusplus
}
#endif

#endif // LUNA_DATA_SETS_H
