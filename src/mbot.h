#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/motor/motor.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>
#include <mbot/fram/fram.h>
#include <mbot/imu/imu.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_channels.h>
#include <mbot_lcm_msgs_serial.h>
#include "pid_filter.h"

#include <math.h>
#include <inttypes.h>

#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 1

//Define drive type of this robot. See mbot_params.h.
// #define MBOT_DRIVE_TYPE OMNI_120_DRIVE
#define MBOT_DRIVE_TYPE DIFFERENTIAL_DRIVE

extern mbot_bhy_data_t mbot_imu_data;

// Global pointer to the i2c bus
static i2c_inst_t *i2c;

// data to hold calibration coefficients
float coeffs[12];  // 4 calibration parameters per motor 

enum drive_modes{
    MODE_MOTOR_PWM = 0,
    MODE_MOTOR_VEL_OL = 1,
    MODE_MOTOR_VEL_PID = 2,
    MODE_MBOT_VEL = 3
};

/*
* Messages used by the MBot code, 
* we also use these to store state
*/
// origin: mbot
serial_mbot_imu_t mbot_imu = {0};
serial_pose2D_t mbot_odometry = {0};
serial_mbot_encoders_t mbot_encoders = {0};
serial_twist2D_t mbot_vel = {0};
serial_mbot_motor_pwm_t mbot_motor_pwm = {0};
serial_mbot_motor_vel_t mbot_motor_vel = {0};

// origin: comms
serial_joy_t joy_cmd = {0};
serial_twist2D_t mbot_vel_cmd = {0};
serial_mbot_motor_pwm_t mbot_motor_pwm_cmd = {0};
serial_mbot_motor_vel_t mbot_motor_vel_cmd = {0};
serial_timestamp_t mbot_received_time = {0};

//callback functions
void joy_cmd_cb(serial_joy_t *msg);
void timestamp_cb(serial_timestamp_t *msg);
void reset_encoders_cb(serial_mbot_encoders_t *msg);
void reset_odometry_cb(serial_pose2D_t *msg);
void mbot_vel_cmd_cb(serial_twist2D_t *msg);
void mbot_motor_vel_cmd_cb(serial_mbot_motor_vel_t *msg);
void mbot_motor_pwm_cmd_cb(serial_mbot_motor_pwm_t *msg);
bool mbot_loop(repeating_timer_t *rt);
void mbot_read_encoders(serial_mbot_encoders_t* encoders);
void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel);

//helper functions
float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx);


// Balance Bot
// static rc_filter_t filter_theta = RC_FILTER_INITIALIZER;
// static rc_filter_t filter_psi = RC_FILTER_INITIALIZER;
// static rc_filter_t filter_phi = RC_FILTER_INITIALIZER;

static PID_t filter_theta; // Body Angle
static PID_t filter_psi; // Wheel Position
static PID_t filter_phi; // Yaw
static PID_t filter_vel; // Velocity

// static bool running = false;
// static float P = 4;
// static float I = .2;
// static float D = .1;

// static double eP = .1;
// static double eI = 0;
// static double eD = .04;
static float P = 4.0;
static float I = .1;
static float D = .1;

static double eP = .1;
static double eI = 0;
static double eD = 2.5;

static float uP = .2;
static float uI = 0;
static float uD = .001;

static float vP = .2;
static float vI = 0;
static float vD = .001;

mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;
serial_joy_t joy_cmd;
serial_twist2D_t gains;

static int motor_0_polarity = -1;
static int motor_2_polarity = 1;
static float encoder_clicks_to_2rad = .001;
static float target_theta = .13;
static float last_d3 = 0;
static float target_psi = 0.0;
static float target_vel = 0.0;
static float target_phi = 0;
static float del_psi = 0;
static float del_theta = 0;
static float vel = 0;
static float instruction = 0;
static float sum_theta = 0;
static int cycles = 0;
static double d1, d2, d3, t1, t2, t3 = 0;
static double balanced_theta = .08;
static double limits = .30;
static int loop_time_ms = 10;

#endif
