/**
 * This file is the main executable for the MBot firmware.
 */
#include "mbot.h"
#include "print_tables.h"

// Global
uint64_t timestamp_offset = 0;
uint64_t current_utime = 0;
int drive_mode = 0;
bool running = false;

void register_topics()
{
    // Subscriptions
    comms_register_topic(MBOT_TIMESYNC, sizeof(serial_timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    comms_register_topic(MBOT_ODOMETRY_RESET,  sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, (MsgCb)&reset_odometry_cb);
    comms_register_topic(MBOT_ENCODERS_RESET, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, (MsgCb)&reset_encoders_cb);
    comms_register_topic(MBOT_MOTOR_PWM_CMD, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, (MsgCb)mbot_motor_vel_cmd_cb);
    comms_register_topic(MBOT_MOTOR_VEL_CMD, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, (MsgCb)mbot_motor_pwm_cmd_cb);
    comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)mbot_vel_cmd_cb);

    // Published Topics
    comms_register_topic(MBOT_ODOMETRY, sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, NULL);
    comms_register_topic(MBOT_IMU, sizeof(serial_mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    comms_register_topic(MBOT_ENCODERS, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, NULL);
    comms_register_topic(MBOT_VEL, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize);
    comms_register_topic(MBOT_MOTOR_VEL, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_PWM, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, NULL);

}

void timestamp_cb(serial_timestamp_t *msg)
{
    mbot_received_time.utime = msg->utime;
    uint64_t current_utime = to_us_since_boot(get_absolute_time());
    timestamp_offset = received_time.utime - current_utime;
}

void reset_encoders_cb(serial_mbot_encoders_t *msg)
{
    //memcpy(&encoders, msg, sizeof(serial_mbot_encoders_t));
    for(int i=0; i<3; i++){
        rc_encoder_write(i, msg->ticks[i]);
    }
}

void reset_odometry_cb(serial_pose2D_t *msg)
{
    mbot_odometry.x = msg->x;
    mbot_odometry.y = msg->y;
    mbot_odometry.theta = msg->theta;
}

void mbot_vel_cmd_cb(serial_twist2D_t *msg)
{
    memcpy(&mbot_vel_cmd, msg, sizeof(serial_twist2D_t));
    drive_mode = MODE_MBOT_VEL;
}

void mbot_motor_vel_cmd_cb(serial_mbot_motor_vel_t *msg)
{
    memcpy(&mbot_motor_vel_cmd, msg, sizeof(serial_mbot_motor_vel_t));
    drive_mode = MODE_MOTOR_VEL;
}

void mbot_motor_pwm_cmd_cb(serial_mbot_motor_pwm_t *msg)
{
    memcpy(&mbot_motor_pwm_cmd, msg, sizeof(serial_mbot_motor_pwm_t));
    drive_mode = MODE_MOTOR_PWM;
}

//TODO: this should be tied to the IMU interrupt
bool mbot_loop(repeating_timer_t *rt)
{
    // only run if we've received a timesync message...
    if (comms_get_topic_data(MBOT_TIMESYNC, &mbot_received_time))
    {
        uint64_t current_utime = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        mbot_read_imu(&mbot_imu);
        mbot_read_encoders(&mbot_encoders);
        mbot_calculate_motor_vel(mbot_encoders, &mbot_motor_vel);
        mbot_calculate_mbot_vel(mbot_imu, mbot_motor_vel, &mbot_vel);
        mbot_calculate_odometry(mbot_vel, &mbot_odometry);
        

        if(drive_mode = MODE_MOTOR_VEL){
            mbot_motor_vel_controller(mbot_motor_vel_cmd, &mbot_motor_pwm_cmd);
        }

        else if(drive_mode = MODE_MBOT_VEL){
            mbot_motor_vel_controller(mbot_vel_cmd, &mbot_motor_pwm_cmd);
        }

        // Set motors
        rc_motor_set(0, mbot_motor_pwm_cmd.pwm[0]);
        rc_motor_set(1, mbot_motor_pwm_cmd.pwm[1]);
        rc_motor_set(2, mbot_motor_pwm_cmd.pwm[2]);

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &mbot_encoders);
        // send odom on wire
        comms_write_topic(MBOT_ODOMETRY, &mbot_odometry);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &mbot_imu);
        //uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }

    return true;
}

mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel){
    float conversion = (1.0 / GEAR_RATIO) * (1.0 / ENCODER_RES) * 1E6f * 2.0 * M_PI;
    motor_vel->velocity[0] = (conversion / encoders.delta_time) * encoders.delta_ticks[0];
    motor_vel->velocity[1] = (conversion / encoders.delta_time) * encoders.delta_ticks[1];
    motor_vel->velocity[2] = (conversion / encoders.delta_time) * encoders.delta_ticks[2];
}

mbot_calculate_mbot_vel(serial_mbot_imu_t imu, serial_mbot_motor_vel_t motor_vel, serial_twist2D_t *mbot_vel){
    // nothing here yet
}

void mbot_calculate_odometry(serial_twist2D_t mbot_vel, serial_pose2D_t *odometry){
    // nothing here yet
}

void mbot_read_imu(serial_mbot_imu_t *imu){
    // nothing here yet
}

void mbot_read_encoders(serial_mbot_encoders_t* encoders){
    int64_t delta_time = current_utime - encoders->utime;
    encoders->utime = current_utime;
    encoders->delta_time = delta_time;
    encoders->ticks[0] = rc_encoder_read_count(0);
    encoders->ticks[1] = rc_encoder_read_count(1);
    encoders->ticks[2] = rc_encoder_read_count(2);
    encoders->delta_ticks[0] = rc_encoder_read_delta(0);
    encoders->delta_ticks[1] = rc_encoder_read_delta(1);
    encoders->delta_ticks[2] = rc_encoder_read_delta(2);
}

int mbot_init_pico(void){
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));
    
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
    if(!set_sys_clock_khz(SYS_CLOCK, true)){
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return -1;
    }; 
    
    stdio_init_all(); // enable USB serial terminal
    printf("\nMBot Booting Up!\n");
    return 1;
}

int mbot_init_hardware(void){

    // Initialize Motors
    printf("initializinging motors...\n");
    rc_motor_init();
    printf("initializinging encoders...\n");
    rc_encoder_init();

    // Initialize I2C
    printf("setting i2c bus...\n");
    // Initialize I2C port at 400 kHz
    i2c = i2c0;
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    // Initialize LED
    printf("Starting heartbeat LED...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the IMU using the Digital Motion Processor
    printf("Initializing DMP...\n");
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro = 1;
    mpu_config.enable_magnetometer = 1;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    
    // Calibrate the gyro to eliminate bias, Mbot must be still for this
    rc_mpu_calibrate_gyro_routine(mpu_config);
    sleep_ms(500);
    rc_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(rc_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &rc_dmp_callback);
}

int mbot_init_comms(void){
    printf("Initializing LCM serial communication...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
}

int mbot_start_timer_loop(void){
    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, mbot_loop, NULL, &loop_timer); // 1000x to convert to ms
}

void mbot_print_state(serial_mbot_imu_t imu, serial_mbot_encoders_t encoders, serial_pose2D_t odometry, serial_mbot_motor_vel_t motor_vel){
    printf("\033[2J\r");
    const char* imu_headings[] = {"ROLL", "PITCH", "YAW"};
    const char* enc_headings[] = {"ENC 0", "ENC 1", "ENC 2"};
    const char* odom_headings[] = {"X", "Y", "THETA"};
    const char* motor_vel_headings[] = {"MOT 0", "MOT 1", "MOT 2"};
    
    char buf[1024] = {0};
    generateTableInt(buf, 1, 3, "ENCODERS", enc_headings, encoders.ticks);
    printf("\r%s\n", buf);
    
    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "IMU", imu_headings, imu.angles_rpy);
    printf("\r%s\n", buf);
    
    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "MOTOR", motor_vel_headings, motor_vel.velocity);
    printf("\r%s\n", buf);
    
    buf[0] = '\0';
    float odom_array = {odometry.x, odometry.y, odometry.theta};
    generateTableFloat(buf, 1, 3, "ODOMETRY", imu_headings, odom_array);
    printf("\r%s\n", buf);

}

int main()
{
    running = false;
    mbot_init_pico();
    mbot_init_hardware();
    mbot_init_comms();
    mbot_start_timer_loop();
    printf("Done Booting Up!\n\n");
    running = true;
    while(running){
        mbot_print_state(mbot_imu, mbot_encoders, mbot_odometry, mbot_motor_vel);   
    }
}