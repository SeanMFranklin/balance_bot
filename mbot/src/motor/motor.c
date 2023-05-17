#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <mbot/motor/motor.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>

int mbot_motor_init_freq(uint8_t ch, uint16_t freq) {
    gpio_init(motor_ph[ch]);
    gpio_set_dir(motor_ph[ch], GPIO_OUT);
    gpio_init(motor_en[ch]);
    gpio_set_function(motor_en[ch], GPIO_FUNC_PWM);
    uint16_t slice = pwm_gpio_to_slice_num(motor_en[ch]);

    // Check if we have already set the frequency for the slice, and if we are trying to change it
    if(slice_frequencies[slice] != 0){
        if (slice_frequencies[slice] != freq) {
            printf("Warning: PWM slice %d already initialized with a different frequency: %d Hz\n", slice, slice_frequencies[slice]);
            return MBOT_ERROR;
        }
    }
    slice_frequencies[slice] = freq;

    //  The higher the wrap value, the higher the resolution of the duty cycle. 
    //  The following formulas work out the best value for the clock frequency for any PWM frequency to maximize the duty cycle resolution.
    //  With default frequency of 25 kHz, divider16 equals 16 and WRAP equals 4999.
    //  For more details on the following calculations of the clock divider and wrap value visit:
    //  https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=2
    int32_t roundup = (SYS_CLOCK % (freq * 4096) != 0);
    int32_t divider16 = SYS_CLOCK / freq / 4096 + roundup;
        if (divider16 / 16 == 0) {
        divider16 = 16;
    }
    pwm_wraps[slice] = SYS_CLOCK * 16 / divider16 / freq - 1;
    pwm_set_clkdiv_int_frac(slice, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice, pwm_wraps[slice]);
    pwm_set_enabled(slice, true);
    
    int dir_success = gpio_get_dir(motor_en[ch]);
    int pwm_success = (gpio_get_function(motor_en[ch]) == GPIO_FUNC_PWM);
    return (dir_success & pwm_success) ? MBOT_OK : MBOT_ERROR;
}

//  This starts the motor drivers at the specified frequency f.
int mbot_motor_init(uint8_t ch) {
    uint16_t f = PWM_FREQ;
    return mbot_motor_init_freq(ch, f);
}

int mbot_motor_cleanup(uint8_t ch) {
    uint16_t slice = pwm_gpio_to_slice_num(motor_en[ch]);
    pwm_set_enabled(slice, false);
    gpio_set_function(motor_en[ch], GPIO_FUNC_NULL);
    gpio_set_function(motor_ph[ch], GPIO_FUNC_NULL);
    int success = gpio_get_function(motor_en[ch] == GPIO_FUNC_NULL) & (gpio_get_function(motor_ph[ch]) == GPIO_FUNC_NULL);
    return (success) ? MBOT_OK : MBOT_ERROR;
}

int mbot_motor_set_duty_int16(uint8_t ch, int32_t duty) {
    // check if valid channel
    if((ch < 0) | (ch > 3)){
        fprintf(stderr, "Error: Invalid channel in mbot_motor_set\n");
        return MBOT_ERROR;
    }
    uint16_t slice = pwm_gpio_to_slice_num(motor_en[ch]);
    //Check if out of range
    if(duty > INT16_MAX){
        fprintf(stderr, "Warning: duty cycle out of range, setting to maximum \n");
        duty = INT16_MAX;
    }
    else if(duty < -INT16_MAX){
        fprintf(stderr, "Warning: duty cycle out of range, setting to maximum \n");
        duty = -INT16_MAX;
    }
    // Check for direction
    bool direction = (duty >= 0);
    uint16_t level = pwm_wraps[slice] * (uint16_t) ((direction)? (duty) : (-1 * duty));
    gpio_put(motor_ph[ch], direction);
    pwm_set_chan_level(slice, motor_ph[ch], level);
    return MBOT_OK;
}

int mbot_motor_set_duty(uint8_t ch, float duty){
    int32_t duty_int32 = (int32_t) duty * INT16_MAX;
    return mbot_motor_set_duty_int16(ch, duty_int32);
}