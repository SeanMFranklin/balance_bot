/**
 *
 * @brief Functions to use the BOSCH BHI160B IMU with BMM150 Magnetometer
 *
 * @author pgaskell
 * @date 2022
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>

#include <mbot/imu/bhy_uc_driver.h>
#include <mbot/imu/bhy_uc_driver_config.h>
#include <mbot/imu/bhy_uc_driver_types.h>
#include <mbot/imu/bhy_uc_driver_constants.h>
#include <mbot/imu/bhy.h>
#include <mbot/imu/bhy_support.h>
#include <mbot/imu/firmware/BHI160B_fw.h>

#define ACCEL_2_MS2     -0.001209716796875
#define GYRO_2_RADS     6.651407210344245e-05
#define MAG_2_UT        0.0625 // 16LSB/uT not certain about this value
#define QUAT_2_NORM     6.103515625e-05
#define RPY_2_RAD       0.00019174759848570515

#define FIFO_SIZE                      70
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60

uint8_t fifo[FIFO_SIZE];
static i2c_inst_t *i2c;
uint8_t *fifoptr = NULL;
uint8_t bytes_left_in_fifo = 0;
uint16_t bytes_remaining = 0;
uint16_t bytes_read = 0;

typedef struct mbot_bhy_data_t{
	/** @name base sensor readings in real units */
	///@{
	float accel[3];	    ///< accelerometer (XYZ) in units of m/s^2
	float gyro[3];		///< gyroscope (XYZ) in units of rad/s
	float mag[3];		///< magnetometer (XYZ) in units of uT
    float quat[4];	    ///< normalized quaternion from Fuser Core
	float rpy[3];       ///< Roll(x) pitch(Y) yaw(Z) in radians from Fuser Core
    int16_t quat_qlty;  ///< quality estimate from Fuser Core
	///@}

	/** @name 16 bit raw readings and conversion rates*/
	///@{
    int16_t raw_accel[3];	///< raw accelerometer (XYZ) from 16-bit ADC
	int16_t raw_gyro[3];	///< raw gyroscope (XYZ)from 16-bit ADC
    int16_t raw_mag[3]; 	///< raw magnetometer (XYZ)from 16-bit ADC
    int16_t raw_quat[4];    ///< raw quaternion (WXYZ) from 16-bit ADC
    int16_t raw_rpy[3];     ///< raw RPY vector (XYZ) Converted to -2^15 TO 2^15
	float accel_to_ms2;	    ///< conversion rate from raw accelerometer to m/s^2
	float gyro_to_rads; 	///< conversion rate from raw gyroscope to rad/s
    float mag_to_uT;	    ///< conversion rate from raw gyroscope to uT
    float quat_to_norm;     ///< conversion rate from raw quaternion
    float rpy_to_rad;       ///< conversion rate from raw RPY vector to radians
	///@}
} mbot_bhy_data_t;

int mbot_imu_init(mbot_bhy_data_t* data);
int mbot_imu_print(mbot_bhy_data_t data);


