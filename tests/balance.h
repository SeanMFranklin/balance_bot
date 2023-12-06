#ifndef BALANCE_H
#define BALANCE_H

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

#include <math.h>
#include <inttypes.h>

static rc_filter_t filter_theta = RC_FILTER_INITIALIZER;
static rc_filter_t filter_psi = RC_FILTER_INITIALIZER;
static rc_filter_t filter_phi = RC_FILTER_INITIALIZER;

static bool running = false;
static float P = 5.0;
static float I = .001;
static float D = .03;

static double eP = -0.0001;
static double eI = 0;
static double eD = -0.00005;

mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;
serial_joy_t joy_cmd;
serial_twist2D_t gains;

static float target_theta = .115;
static float del_theta = 0;
static float vel = 0;
static float instruction = 0;
static float sum_theta = 0;
static int cycles = 0;
static double d1, d2, d3, t1, t2, t3 = 0;
static double balanced_theta = .115;
static double limits = .035;
static int loop_time_ms = 10;



#endif
