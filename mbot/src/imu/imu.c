#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include <imu/bhy_support.h>>
#include <imu/bhy_uc_driver.h>>
#include <imu/firmware/Bosch_PCB_7183_di03_BMI160_BMM150-7183_di03.2.1.11696_170103.h>

#define FIFO_SIZE                      300
#define ROTATION_VECTOR_SAMPLE_RATE    100
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60

char out_buffer[OUT_BUFFER_SIZE] = " W: 0.999  X: 0.999  Y: 0.999  Z: 0.999   \r";
uint8_t fifo[FIFO_SIZE];

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_rotation_vector(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    float temp;
    uint8_t index;

    temp = sensor_data->data_quaternion.w / 16384.0f; /* change the data unit by dividing 16384 */
    out_buffer[3] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[4] = floor(temp) + '0';

    for (index = 6; index <= 8; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.x / 16384.0f;
    out_buffer[13] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[14] = floor(temp) + '0';

    for (index = 16; index <= 18; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.y / 16384.0f;
    out_buffer[23] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[24] = floor(temp) + '0';

    for (index = 26; index <= 28; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.z / 16384.0f;
    out_buffer[33] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[34] = floor(temp) + '0';

    for (index = 36; index <= 38; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }


    DEBUG("x=%d, y=%d, z=%d, w=%d\n",
    sensor_data->data_quaternion.x,
    sensor_data->data_quaternion.y,
    sensor_data->data_quaternion.z,
    sensor_data->data_quaternion.w
    );
}

/*!
 * @brief This function is used to run bhy hub
 */
void demo_sensor(void)
{
    int8_t ret;

    /* BHY Variable*/
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_remaining    = 0;
    uint16_t                   bytes_read         = 0;
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;

    /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
    /* to get this information. This feature is only supported for customized firmware. To get this customized */
    /* firmware, you need to contact your local FAE of Bosch Sensortec. */
    //struct cus_version_t      bhy_cus_version;

    bhy_install_meta_event_callback(BHY_META_EVENT_TYPE_INITIALIZED, meta_event_callback);
    bhy_install_meta_event_callback(BHY_META_EVENT_TYPE_SELF_TEST_RESULTS, meta_event_callback);

    DEBUG("version=%s, %s, %s\n", bhy_get_version(), __DATE__, __TIME__);
    DEBUG("start example\n");

    /* init the bhy chip */
    if(bhy_driver_init(&bhy1_fw))
    {
        DEBUG("Fail to init bhy\n");
    }

    /* wait for the bhy trigger the interrupt pin go down and up again */
    while (ioport_get_pin_level(BHY_INT));
    while (!ioport_get_pin_level(BHY_INT));

    /* To get the customized version number in firmware, it is necessary to read Parameter Page 2, index 125 */
    /* to get this information. This feature is only supported for customized firmware. To get this customized */
    /* firmware, you need to contact your local FAE of Bosch Sensortec. */
    //bhy_read_parameter_page(BHY_PAGE_2, PAGE2_CUS_FIRMWARE_VERSION, (uint8_t*)&bhy_cus_version, sizeof(struct cus_version_t));
    //DEBUG("cus version base:%d major:%d minor:%d\n", bhy_cus_version.base, bhy_cus_version.major, bhy_cus_version.minor);

	bhy_set_chip_control(0);
	bhy_set_host_interface_control(BHY_HOST_SELFTEST, ENABLE);
	delay_ms(100);
	
	bhy_set_chip_control(1);
	delay_ms(100);
	
    while(1)
    {
        /* wait until the interrupt fires */
        /* unless we already know there are bytes remaining in the fifo */
        while (!ioport_get_pin_level(BHY_INT) && !bytes_remaining)
        {
        }

        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHY_DATA_TYPE_PADDING;

        do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

            /* prints all the debug packets */
            if (packet_type == BHY_DATA_TYPE_DEBUG)
            {
                bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
            }
            
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
    }
}
/** @}*/