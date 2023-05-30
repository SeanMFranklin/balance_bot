#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <mbot/imu/imu.h>

extern mbot_bhy_data_t mbot_imu_data;

int main() {
    stdio_init_all();
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");
    bi_decl(bi_program_description("This is a test for the IMU."));
    mbot_bhy_config_t mbot_imu_config;
    mbot_imu_config.enable_quat = 1;
    mbot_imu_config.enable_mag = 0;
    mbot_imu_config.enable_rpy = 0;
    mbot_imu_config.sample_rate = 100;
    mbot_imu_config.accel_range = 2;
    mbot_imu_config.gyro_range = 125;
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);

    while(1){
        printf("\033[2J\r");
        mbot_imu_print(mbot_imu_data);
        sleep_ms(100);
    }

}