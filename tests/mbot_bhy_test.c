/**
 * This file is to test he mpu9250
 */

#include <pico/stdlib.h>
#include <inttypes.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>

#include <mbot/imu/bhy_uc_driver.h>
#include <mbot/imu/bhy_uc_driver_config.h>
#include <mbot/imu/bhy_uc_driver_types.h>
#include <mbot/imu/bhy_uc_driver_constants.h>
#include <mbot/imu/bhy.h>
#include <mbot/imu/bhy_support.h>
#include <mbot/imu/firmware/BHI160B_fw.h>
#include <mbot/fram/fram.h>
#include <mbot/defs/mbot_pins.h>
#define DEBUG

/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/
/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      300
#define ROTATION_VECTOR_SAMPLE_RATE    100
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60

/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/
uint8_t fifo[FIFO_SIZE];
static i2c_inst_t *i2c;
uint8_t interrupt = false;

unsigned int swapByteOrder(unsigned int value) {
    return ((value >> 24) & 0xFF) | ((value >> 8) & 0xFF00) | ((value << 8) & 0xFF0000) | ((value << 24) & 0xFF000000);
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

/********************************************************************************/
/*                                 FUNCTIONS                                    */
/********************************************************************************/
static char event_str[128];
static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};
void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    interrupt = true;
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
      // handle the IRQ
}

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_rotation_vector(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    printf("x=%d, y=%d, z=%d",
    sensor_data->data_vector.x,
    sensor_data->data_vector.y,
    sensor_data->data_vector.z
    );
}

void bhy_dump_status(void){
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

int main() {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));

    stdio_init_all();
    
    sleep_ms(2000); // quick sleep so we can catch the bootup process in terminal
    printf("\033[2J\r");
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

    //gpio_set_function(IMU_INT_PIN,GPIO_IN);
    //gpio_pull_down(IMU_INT_PIN);
    gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    
    
    /* the remapping matrix for BHA or BHI here should be configured according to its placement on customer's PCB. */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    int8_t bhy_mapping_matrix_config[3*3] = {0,1,0,-1,0,0,0,0,1};
    
    /* the remapping matrix for Magnetometer should be configured according to its placement on customer's PCB.  */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    int8_t mag_mapping_matrix_config[3*3] = {0,1,0,1,0,0,0,0,-1};
   
    /* the sic matrix should be calculated for customer platform by logging uncalibrated magnetometer data. */
    /* the sic matrix here is only an example array (identity matrix). Customer should generate their own matrix. */
    /* This affects magnetometer fusion performance. */
    float sic_array[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    // Config Bosch chip
    printf("Initializing the Bosch IMU...\r\n");
    int8_t init_result = bhy_driver_init(bhy1_fw);
    if(!init_result){
        printf("Success!\n");
    }
    while(!interrupt);
    interrupt = false;
    printf("got interrupt\n");

    bhy_dump_status();
    printf("Setting Matrix Config...\n");
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config);
    /* This sic matrix setting affects magnetometer fusion performance. */
    bhy_set_sic_matrix(sic_array);
    //while(!interrupt);
    //interrupt = false;
    printf("got interrupt\n");


    printf("Installing Orientation Sensor...\n");
    /* install the callback function for parse fifo data */
    if(bhy_install_sensor_callback(VS_TYPE_ORIENTATION, VS_WAKEUP, sensors_callback_rotation_vector))
    {
        printf("Failed to install sensor callback\n");
    }
    /* enables the virtual sensor */
    if(bhy_enable_virtual_sensor(VS_TYPE_ORIENTATION, VS_WAKEUP, 100, 0, 0, 0, 0))
    {
        DEBUG("Fail to enable sensor id=%d\n", VS_TYPE_ORIENTATION);
    }
    
    //gpio_set_irq_enabled_with_callback(IMU_INT_PIN, GPIO_IRQ_EDGE_RISE, true, &bhy_callback);
    // VS_TYPE_ACCELEROMETER
    // VS_TYPE_LINEAR_ACCELERATION
    // VS_TYPE_GYROSCOPE
    // VS_TYPE
    printf("Waiting For Interupt!\n");


    /* BHY Variable*/
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_remaining    = 0;
    uint16_t                   bytes_read         = 0;
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;

    while (1){
        while (!interrupt && !bytes_remaining);
        interrupt = false;
        printf("got interrupt\n");
        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHY_DATA_TYPE_PADDING;

         do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

            /* prints all the debug packets */
            //if (packet_type == BHY_DATA_TYPE_DEBUG)
            //{
                bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
            //}
            
            /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
            /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
            /* packet */
        } while ((result == BHY_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

        bytes_left_in_fifo = 0;
        if (bytes_remaining)
        {
            /* shifts the remaining bytes to the beginning of the buffer */
            while (bytes_left_in_fifo < bytes_read)
            {
                fifo[bytes_left_in_fifo++] = *(fifoptr++);
            }
        }
        //interrupt = false;
    }

}
