#include <stdio.h>
#include <stdint.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <mbot/imu/imu.h>

mbot_bhy_data_t mbot_imu_data;

int main() {
    stdio_init_all();
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing!\n");
    bi_decl(bi_program_description("This is a test for the IMU."));
    mbot_imu_init(&mbot_imu_data);

    while(1){
        mbot_imu_print(mbot_imu_data);
        sleep_ms(50);
    }

}