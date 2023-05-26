#include <mbot/imu/imu.h>

extern mbot_bhy_data_t mbot_imu_data;

// Globals for IMU
uint8_t fifo[FIFO_SIZE];
static i2c_inst_t *i2c;
uint8_t *fifoptr = NULL;
uint8_t bytes_left_in_fifo = 0;
uint16_t bytes_remaining = 0;
uint16_t bytes_read = 0;

void _imu_callback(uint gpio, uint32_t events);
void _bhy_dump_status(void);
static void _sensors_callback_quaternion(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);
static void _sensors_callback_orientation(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);
static void _sensors_callback_accel(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);
static void _sensors_callback_gyro(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);
static void _sensors_callback_mag(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id);


void _imu_callback(uint gpio, uint32_t events) {
    bhy_data_generic_t fifo_packet;
    bhy_data_type_t packet_type;
    int result;
    for(int i=0; i<5; i++){
        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read += bytes_left_in_fifo;
        fifoptr = fifo;
        packet_type = BHY_DATA_TYPE_PADDING;
        while(bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)){
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);
        }
    }
    if (bytes_remaining){
        /* shifts the remaining bytes to the beginning of the buffer */
        while (bytes_left_in_fifo < bytes_read)
        {
            fifo[bytes_left_in_fifo++] = *(fifoptr++);
        }
    }
}

int mbot_imu_init(mbot_bhy_data_t* data){
    if(data == NULL){
        printf("[ERROR] data struct is NULL\n");
        return -1;
    }
    data->accel_to_ms2 = ACCEL_2_MS2;
    data->gyro_to_rads = GYRO_2_RADS;
    data->mag_to_uT = MAG_2_UT;
    data->quat_to_norm = QUAT_2_NORM;
    data->rpy_to_rad = RPY_2_RAD;
    i2c = i2c0;
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &_imu_callback);
    
    printf("Initializing the Bosch IMU...\r\n");
    int8_t init_result = bhy_driver_init(bhy1_fw);
    if(!init_result){
        printf("Success!\n");
    }
    else {
        printf("[ERROR] Failed uploading firmware to BHI160\n");
        return -1;
    }

    printf("Setting Matrix Config...\n");
    int8_t bhy_mapping_matrix_config[3*3] = {1,0,0,0,1,0,0,0,1};
    int8_t mag_mapping_matrix_config[3*3] = {1,0,0,0,-1,0,0,0,-1};
    float sic_array[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
    sleep_ms(10);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config);
    sleep_ms(10);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config);
    sleep_ms(10);
    /* This sic matrix setting affects magnetometer fusion performance. */
    bhy_set_sic_matrix(sic_array);
    
    printf("Installing Sensors...\n");
    /* install the callback function for parse fifo data */
    if(bhy_install_sensor_callback(VS_TYPE_ROTATION_VECTOR, 0, _sensors_callback_quaternion))
    {
        printf("Failed to install sensor callback\n");
        return -1;
    }
    if(bhy_install_sensor_callback(VS_TYPE_ORIENTATION, 0, _sensors_callback_orientation))
    {
        printf("Failed to install sensor callback\n");
        return -1;
    }
    if(bhy_install_sensor_callback(VS_TYPE_ACCELEROMETER, 0, _sensors_callback_accel))
    {
        printf("Failed to install sensor callback\n");
        return -1;
    }
    if(bhy_install_sensor_callback(VS_TYPE_GYROSCOPE, 0, _sensors_callback_gyro))
    {
        printf("Failed to install sensor callback\n");
        return -1;
    }
    if(bhy_install_sensor_callback(VS_TYPE_GEOMAGNETIC_FIELD, 0, _sensors_callback_mag))
    {
        printf("Failed to install sensor callback\n");
        return -1;
    }
    bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_ORIENTATION, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_ACCELEROMETER, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_GYROSCOPE, 0, 25, 0, 0, 0, 0);
    bhy_enable_virtual_sensor(VS_TYPE_GEOMAGNETIC_FIELD, 0, 25, 0, 0, 0, 0);

    return 0;
}

int mbot_imu_print(mbot_bhy_data_t data){
    printf("ACCEL RAW| X: %d | Y: %d | Z: %d \n", data.raw_accel[0], data.raw_accel[1], data.raw_accel[2]);
    printf("ACCEL MS2| X: %f | Y: %f | Z: %f \n", data.accel[0], data.accel[1], data.accel[2]);
    printf(" GYRO RAW| X: %d | Y: %d | Z: %d \n", data.raw_gyro[0], data.raw_gyro[1], data.raw_gyro[2]);
    printf("GYRO RADS| X: %-3.4f | Y: %-3.4f | Z: %-3.4f \r", data.gyro[0], data.gyro[1], data.gyro[2]);
    printf("  MAG RAW| X: %d | Y: %d | Z: %d |\n", data.raw_mag[0], data.raw_mag[1], data.raw_mag[2]);
    printf("   MAG UT| X: %f | Y: %f | Z: %f |\n", data.mag[0], data.mag[1], data.mag[2]);
    printf("  RPY RAW| X: %5d | Y: %5d| Z: %5d |\n", data.raw_rpy[0], data.raw_rpy[1], data.raw_rpy[2]);
    printf("  RPY RAD| X: %-3.3f | Y: %-3.3f | Z: %-3.3f |\n", data.rpy[0], data.rpy[1], data.rpy[2]);
    printf(" QUAT RAW| W: %5d | X: %5d | Y: %5d | Z: %5d |\n", data.raw_quat[0], data.raw_quat[1], data.raw_quat[2], data.raw_quat[3]);
    printf("QUAT NORM| W: %-3.4f | X: %-3.4f | Y: %-3.4f | Z: %-3.4f |\n", data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
}

void _bhy_dump_status(void){
    uint16_t rom_version;
    uint8_t product_id;
    uint8_t revision_id;
    uint16_t ram_version;
    unsigned int host_crc;
    uint8_t ctl_reg;
    bhy_get_chip_control(&ctl_reg);
    bhy_get_rom_version(&rom_version);
    bhy_get_product_id(&product_id);
    bhy_get_revision_id(&revision_id);
    bhy_get_ram_version(&ram_version);
    bhy_get_crc_host(&host_crc);
    printf("CTL Mode=%x\n", ctl_reg);
    printf("ROM Version=%x\n", rom_version);
    printf("RAM Version=%x\n", ram_version);
    printf("Product ID=%x\n", product_id);
    printf("Revision ID=%x\n", revision_id);
    printf("Host CRC: %x\n", host_crc);
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void _sensors_callback_quaternion(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_quat[0] = sensor_data->data_quaternion.w;
    mbot_imu_data.raw_quat[1] = sensor_data->data_quaternion.x;
    mbot_imu_data.raw_quat[2] = sensor_data->data_quaternion.y;
    mbot_imu_data.raw_quat[3] = sensor_data->data_quaternion.z;
    mbot_imu_data.quat_qlty = sensor_data->data_quaternion.estimated_accuracy;
    mbot_imu_data.quat[0] = (float)mbot_imu_data.raw_quat[0] * mbot_imu_data.quat_to_norm;
    mbot_imu_data.quat[1] = (float)mbot_imu_data.raw_quat[1] * mbot_imu_data.quat_to_norm;
    mbot_imu_data.quat[2] = (float)mbot_imu_data.raw_quat[2] * mbot_imu_data.quat_to_norm;
    mbot_imu_data.quat[3] = (float)mbot_imu_data.raw_quat[3] * mbot_imu_data.quat_to_norm;
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void _sensors_callback_orientation(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_rpy[0] = -sensor_data->data_vector.y;
    mbot_imu_data.raw_rpy[1] = -sensor_data->data_vector.z;
    mbot_imu_data.raw_rpy[2] = (int16_t)(sensor_data->data_vector.x << 1)/2; //convert to -2^15 (-PI) to 2^15 (PI)
    mbot_imu_data.rpy[0] = mbot_imu_data.raw_rpy[0] * mbot_imu_data.rpy_to_rad;
    mbot_imu_data.rpy[1] = mbot_imu_data.raw_rpy[1] * mbot_imu_data.rpy_to_rad;
    mbot_imu_data.rpy[2] = mbot_imu_data.raw_rpy[2] * mbot_imu_data.rpy_to_rad;
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void _sensors_callback_accel(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{

    mbot_imu_data.raw_accel[0] = sensor_data->data_vector.x;
    mbot_imu_data.raw_accel[1] = sensor_data->data_vector.y;
    mbot_imu_data.raw_accel[2] = sensor_data->data_vector.z;
    mbot_imu_data.accel[0] = (float)mbot_imu_data.raw_accel[0] * mbot_imu_data.accel_to_ms2;
    mbot_imu_data.accel[1] = (float)mbot_imu_data.raw_accel[1] * mbot_imu_data.accel_to_ms2;
    mbot_imu_data.accel[2] = (float)mbot_imu_data.raw_accel[2] * mbot_imu_data.accel_to_ms2;
    
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void _sensors_callback_gyro(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_gyro[0] = sensor_data->data_vector.x;
    mbot_imu_data.raw_gyro[1] = sensor_data->data_vector.y;
    mbot_imu_data.raw_gyro[2] = sensor_data->data_vector.z;
    mbot_imu_data.gyro[0] = (float)mbot_imu_data.raw_gyro[0] * mbot_imu_data.gyro_to_rads;
    mbot_imu_data.gyro[1] = (float)mbot_imu_data.raw_gyro[1] * mbot_imu_data.gyro_to_rads;
    mbot_imu_data.gyro[2] = (float)mbot_imu_data.raw_gyro[2] * mbot_imu_data.gyro_to_rads;
}

static void _sensors_callback_mag(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    mbot_imu_data.raw_mag[0] = sensor_data->data_vector.x;
    mbot_imu_data.raw_mag[1] = sensor_data->data_vector.y;
    mbot_imu_data.raw_mag[2] = sensor_data->data_vector.z;
    mbot_imu_data.mag[0] = (float)mbot_imu_data.raw_mag[0] * mbot_imu_data.mag_to_uT;
    mbot_imu_data.mag[1] = (float)mbot_imu_data.raw_mag[1] * mbot_imu_data.mag_to_uT;
    mbot_imu_data.mag[2] = (float)mbot_imu_data.raw_mag[2] * mbot_imu_data.mag_to_uT;
}

// implementation of extern function in bosch IMU code
int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{    
    // this is inefficient - shouldnt copy data into a buffer
    // am doing now for ease of debug
    uint8_t buf[size+1];
	buf[0] = reg;
	for(int i=0; i<size; i++) buf[i+1]=p_buf[i];

	if(i2c_write_blocking(i2c, addr, &buf[0], size+1, false) == PICO_ERROR_GENERIC){
		printf("ERROR: failed to write to bosch IMU!\n");
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
    //printf("read %d bytes, expected %d\r\n", bytes_read, size);
   
    if( bytes_read == PICO_ERROR_GENERIC){
		return -1;
	}
    else if(bytes_read != size){
        return -1;
    }
	return 0;
}