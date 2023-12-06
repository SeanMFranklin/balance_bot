/**
 * This file is the main executable MBot Balance-Bot
 */

#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <mbot/motor/motor.h>
#include <mbot/defs/mbot_params.h>
#include <pico/binary_info.h>
#include <mbot/imu/imu.h>
#include <mbot/encoder/encoder.h>
#include <rc/math/filter.h>
#include <mbot/utils/utils.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_channels.h>
#include <mbot_lcm_msgs_serial.h>
#include <pico/multicore.h>
#include <mbot_lcm_msgs_serial.h>
#include "balance.h"


int mbot_init_pico(void)
{
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
    if (!set_sys_clock_khz(SYS_CLOCK, true))
    {
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return MBOT_ERROR;
    };

    stdio_init_all(); // enable USB serial terminal
    sleep_ms(500);
    printf("\nMBot Booting Up!\n");
    return MBOT_OK;
}

int mbot_init_hardware(void)
{
    sleep_ms(500);
    // Initialize Motors
    printf("initializinging motors...\n");
    printf("Motor 0: %s\n", mbot_motor_init(0) ? "Working" : "ERROR");
    printf("Motor 2: %s\n\n", mbot_motor_init(2) ? "Working" : "ERROR");

    // Initialize Encoders
    printf("initializinging encoders...\n");
    printf("Encoder: %s\n\n", mbot_encoder_init() ? "Working" : "ERROR");

    // Initialize LED
    printf("Starting heartbeat LED...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the IMU using the Digital Motion Processor
    printf("Initializing IMU...\n");
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.accel_range = 4;
    mbot_imu_config.gyro_range = 250;
    mbot_imu_config.enable_rpy = 1;
    mbot_imu_config.enable_quat = 1;
    mbot_imu_config.enable_mag = 1;
    mbot_imu_config.sample_rate = 200;
    printf("IMU: %s\n\n", mbot_imu_init(&mbot_imu_data, mbot_imu_config) ? "Working" : "ERROR");
    sleep_ms(1000);
    mbot_motor_set_duty(0, 0);
    mbot_motor_set_duty(2, 0);
    return MBOT_OK;
}

void joy_cmd_cb(serial_joy_t *msg)
{
    memcpy(&joy_cmd, msg, sizeof(serial_joy_t));
    printf("Joystick: %4.4f\n", joy_cmd.left_analog_X);
}

void pid_cb(serial_twist2D_t *msg)
{
    memcpy(&gains, msg, sizeof(serial_twist2D_t));
    printf("Inner P: %4.4f\n", gains.vx);
}

int mbot_init_comms(void)
{
    printf("Initializing LCM serial communication...\r\n");
    printf("Comms Initialized: %s\n", comms_init_protocol() ? "Working" : "ERROR");
    printf("Topic Data Initialized: %s\n", comms_init_topic_data() ? "Working" : "ERROR");
    printf("Mbot_Joy Registered: %s\n", comms_register_topic(MBOT_JOY, sizeof(serial_joy_t), (Deserialize)&joy_t_deserialize, (Serialize)&joy_t_serialize, (MsgCb)&joy_cmd_cb) ? "Working" : "ERROR");
    printf("Mbot_Vel_Cmd Registered: %s\n", comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)&pid_cb) ? "Working" : "ERROR");
    printf("Mbot_Vel Registered: %s\n", comms_register_topic(MBOT_VEL, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, NULL) ? "Working" : "ERROR");
    

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    return MBOT_OK;
}

void read_encoders(double *d1,double *d2,double *d3,double *t1,double *t2,double *t3)
{
    *d1 = 1.0 * mbot_encoder_read_delta(0);
    *d2 = 1.0 * mbot_encoder_read_delta(1);
    *d3 = 1.0 * mbot_encoder_read_delta(2);
    *t1 = 1.0 * mbot_encoder_read_count(0);
    *t2 = 1.0 * mbot_encoder_read_count(1);
    *t3 = 1.0 * mbot_encoder_read_count(2);
}

// int inner_loop(float* TARGET_THETA) {
//     float del_theta = mbot_imu_config.rpy[1] - TARGET_THETA;

//     mbot_motor_set_duty(0, d);

// }



bool control_loop()
{
    // serial_twist2D_t test_message;

    read_encoders(&d1, &d2, &d3, &t1, &t2, &t3);

    target_theta = rc_filter_march(&filter_psi, t1);
    del_theta = mbot_imu_data.rpy[1] - target_theta;
    instruction = rc_filter_march(&filter_theta, del_theta);
    // vel = mbot_imu_data.gyro[1];
    // instruction = P * del_theta + I * sum_theta + D * vel;
    mbot_motor_set_duty(0, instruction);
    mbot_motor_set_duty(2, instruction);
    if (cycles++ % 40 == 0)
    {
        // printf("\033[2J\r");
        // mbot_imu_print(mbot_imu_data);
        // printf("\nInstruction: %4.4f", instruction);
        // printf("\nTarget Theta: %4.4f\n", target_theta);
        // printf("\nEncoder 0: %4.1f Encoder 2: %4.1f\n", t1, t3);
        // printf("Joystick: %4.4f\n", joy_cmd.left_analog_X);
        printf("Inner P: %4.4f\n", gains.vx);
    }
    // if (joy_cmd.button_A == 1)
    // {
    //     running = 0;
    // }
    // sleep_ms(loop_time_ms);
    return true;
}

int main()
{
    sleep_ms(500);
    mbot_init_pico();
    mbot_init_hardware();
    mbot_init_comms();
    sleep_ms(3000);
    printf("\033[2J\r");

    rc_filter_pid(&filter_theta, P, I, D, loop_time_ms / 250.0, loop_time_ms / 1000.0);
    rc_filter_pid(&filter_psi, eP, eI, eD, loop_time_ms / 250.0, loop_time_ms / 1000.0);
    rc_filter_enable_saturation(&filter_psi, balanced_theta - limits, balanced_theta + limits);

    repeating_timer_t loop_timer;
    add_repeating_timer_ms(loop_time_ms, control_loop, NULL, &loop_timer);

    running = 1;

    while(running){
        // Heartbeat

        // Print State
        printf("\033[2J\r");
        // printf("Inner P: %4.4f\n", gains.vx);
        printf("\nInstruction: %4.4f", instruction);
        printf("\nTarget Theta: %4.4f\n\n", target_theta);
        mbot_imu_print(mbot_imu_data);
        sleep_ms(200); 
    }
    return 0;
}