#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>
#include <rc/defs/mbot_defs.h>

#include <rc/current/current.h>
#define ADC_TO_VOLT 0.000732421875

void rc_current_sense_init(void)
{
    // init the adc controller
    adc_init();
    // make sure the GPIO pins are in high-impedance mode
    adc_gpio_init(ADC0_PIN);
    adc_gpio_init(ADC1_PIN);
    adc_gpio_init(ADC2_PIN);
}

int rc_current_sense_get_raw(int motor)
{
    adc_select_input(motor);
    return adc_read();
}

float rc_current_sense_get_amps(int motor)
{
    adc_select_input(motor);
    return (float)adc_read() * ADC_TO_VOLT;
}