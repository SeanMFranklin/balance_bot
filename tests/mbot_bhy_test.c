/**
 * This file is to test he mpu9250
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/binary_info.h>

#include <mbot/imu/bhy_uc_driver.h>
#include <mbot/imu/bhy_uc_driver_config.h>
#include <mbot/imu/bhy_uc_driver_types.h>
#include <mbot/imu/bhy_uc_driver_constants.h>
#include <mbot/imu/bhy.h>
#include <mbot/imu/bhy_support.h>
#include <mbot/imu/firmware/BHI160B_BMM150_fw.h>
#include <mbot/fram/fram.h>
#define DEBUG


static i2c_inst_t *i2c;

// implementation of extern function in bosch IMU code
int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{    
    // this is inefficient - shouldnt copy data into a buffer
    // am doing now for ease of debug
    uint8_t buf[size+1];
	buf[0] = addr;
	for(int i=0; i<size; i++) buf[i+1]=p_buf[i];

	if(i2c_write_blocking(i2c, addr, &buf[0], size+1, false) == PICO_ERROR_GENERIC){
		printf("ERROR: failed to write to bosch MPU!\n");
		return -1;
	}

    return 0;
}

// implementation of extern function in bosch IMU code
int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
    if(i2c_write_blocking(i2c, addr, &reg, 1, true) == PICO_ERROR_GENERIC){
		return -1;
	}

    int bytes_read = i2c_read_blocking(i2c, addr, p_buf, size, false);
    printf("read %d bytes, expected %d\r\n", bytes_read, size);
    if( bytes_read == PICO_ERROR_GENERIC){
		return -1;
	}

	return bytes_read;
}

// implementation of extern function in bosch IMU code
void mdelay(unsigned int msec)
{
    sleep_ms(msec);
}


int main() {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));

    stdio_init_all();

    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("Initializing...\n");
    // Pins
    // for the i2c to the IMU
    //const uint sda_pin = 4;
    //const uint scl_pin = 5;
    // Ports
    i2c = i2c0;
    // Initialize I2C pins
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Config Bosch chip
    printf("initializing the Bosch IMU...\r\n");
    int8_t init_result = bhy_driver_init(bhy1_fw);
    printf("got initialization result: %d\r\n", init_result);

    int ii=0;
    printf("   TB_X  |");
    printf("   TB_Y  |");
    printf("   TB_Z  |");
    printf("   A_X   |");
    printf("   A_Y   |");
    printf("   A_Z   |");
    printf("   G_X   |");
    printf("   G_Y   |");
    printf("   G_Z   |");
    printf("  COUNT  |");
    printf("\r\n");
    while (1) {
        printf("\r");
		// printf("%7.3f  |", mpu_data.dmp_TaitBryan[0]);
        // printf("%7.3f  |", mpu_data.dmp_TaitBryan[1]);
        // printf("%7.3f  |", mpu_data.dmp_TaitBryan[2]);
        // printf("%7.3f  |", mpu_data.accel[0]);
        // printf("%7.3f  |", mpu_data.accel[1]);
        // printf("%7.3f  |", mpu_data.accel[2]);
        // printf("%7.3f  |", mpu_data.gyro[0]);
        // printf("%7.3f  |", mpu_data.gyro[1]);
        // printf("%7.3f  |", mpu_data.gyro[2]);
        printf("%7d  |", ii);
        ii++;
        sleep_ms(100);
    }
}
