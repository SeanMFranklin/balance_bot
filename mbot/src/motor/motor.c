#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <mbot/motor/motor.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <mbot/defs/mbot_pins.h>
#include <mbot/defs/mbot_params.h>

#define MAX_PWM 32630
#define FREQ 25000

mbot_motor_state MOTOR_STATE;

static uint16_t pwm_wrap;
static uint16_t pwm_freq = PWM_FREQ;


//  Initializes all 3 motors and leaves them in a free-spin (0 throttle) state.
//  This starts the motor drivers at the default value of FREQ. To use another frequency initialize with mbot_motor_init_freq instead.
//  Returns 0 on success, -1 on failure.
int mbot_motor_init() {
    MOTOR_STATE = ON;
    gpio_init(M0_DIR_PIN);
    gpio_init(M1_DIR_PIN);
    gpio_init(M2_DIR_PIN);
    
    gpio_set_dir(M0_DIR_PIN, GPIO_OUT);
    gpio_set_dir(M1_DIR_PIN, GPIO_OUT);
    gpio_set_dir(M2_DIR_PIN, GPIO_OUT);
    
    gpio_set_function(M0_PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(M1_PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(M2_PWM_PIN, GPIO_FUNC_PWM);

    //  The higher the wrap value, the higher the resolution of the duty cycle. 
    //  The following formulas work out the best value for the clock frequency for any PWM frequency to maximize the duty cycle resolution.
    //  With default frequency of 25 kHz, divider16 equals 16 and WRAP equals 4999.
    //  For more details on the following calculations of the clock divider and wrap value visit:
    //  https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=2

    uint16_t divider16 = SYS_CLOCK / PWM_FREQ / 4096;

    if (divider16 / 16 == 0) {
        divider16 = 16;
    }
    
    pwm_set_clkdiv_int_frac(M0_SLICE, divider16/16, divider16 & 0xF);
    pwm_set_clkdiv_int_frac(M1_SLICE, divider16/16, divider16 & 0xF);
    pwm_set_clkdiv_int_frac(M2_SLICE, divider16/16, divider16 & 0xF);
    
    pwm_wrap = SYS_CLOCK * 16 / divider16 / PWM_FREQ - 1;
    
    pwm_set_wrap(M0_SLICE, pwm_wrap);
    pwm_set_wrap(M1_SLICE, pwm_wrap);
    pwm_set_wrap(M2_SLICE, pwm_wrap);
    
    pwm_set_enabled(M0_SLICE, true);
    pwm_set_enabled(M1_SLICE, true);
    pwm_set_enabled(M2_SLICE, true);
    
    mbot_motor_set(0, 0);
    
    int dir_success = gpio_get_dir(M0_DIR_PIN) & gpio_get_dir(M1_DIR_PIN) & gpio_get_dir(M2_DIR_PIN);
    int pwm_success = (gpio_get_function(M0_PWM_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M1_PWM_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M2_PWM_PIN) == GPIO_FUNC_PWM);
    return (dir_success & pwm_success) ? 0 : -1;
}

//  Initializes all 3 motors and leaves them in a free-spin (0 throttle) state. Returns 0 on success, -1 on failure.
//  This starts the motor drivers at the specified frequency f.
int mbot_motor_init_freq(uint16_t f) {
    pwm_freq = f;
    return mbot_motor_init();
}

// Puts all 3 motors into a free-spin (0 throttle) state, puts the h-bridges into standby mode, and closes all file pointers to GPIO and PWM systems.
// Returns 0 on success, -1 on failure
int mbot_motor_cleanup() {
    if (MOTOR_STATE == OFF) return 0;
    MOTOR_STATE = OFF;
    
    pwm_set_enabled(M0_SLICE, false);
    pwm_set_enabled(M1_SLICE, false);
    pwm_set_enabled(M2_SLICE, false);
    
    gpio_set_function(M0_PWM_PIN, GPIO_FUNC_NULL);
    gpio_set_function(M1_PWM_PIN, GPIO_FUNC_NULL);
    gpio_set_function(M2_PWM_PIN, GPIO_FUNC_NULL);
    gpio_set_function(M0_DIR_PIN, GPIO_FUNC_NULL);
    gpio_set_function(M1_DIR_PIN, GPIO_FUNC_NULL);
    gpio_set_function(M2_DIR_PIN, GPIO_FUNC_NULL);

    int set_success = (gpio_get_function(M0_PWM_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M1_PWM_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M2_PWM_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M0_DIR_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M1_DIR_PIN) == GPIO_FUNC_PWM) &
                      (gpio_get_function(M2_DIR_PIN) == GPIO_FUNC_PWM);
    
    return (set_success) ? 0 : -1;
}

//  Sets the bidirectional duty cycle (power) to a single motor or all motors if 0 is provided as a channel.
//  motor = The motor channel (1-3) or 0 for all channels.
//  duty = -2^15 (signed short lower limit) for full reverse, 2^15 (signed short upper limit) for full forward.
//  Returns 0 on success, -1 on failure
int mbot_motor_set(uint8_t ch, int32_t duty) {
    if (duty < -MAX_PWM) duty = -MAX_PWM;
    else if (duty > MAX_PWM) duty = MAX_PWM;

    bool direction = (duty >= 0);

    uint16_t level = pwm_wrap * (uint16_t) ((direction)? (duty) : (-1 * duty));

    switch (ch) {
        case 0:
            gpio_put(M0_DIR_PIN, direction);
            pwm_set_chan_level(M0_SLICE, M0_CHAN, level);
            break;
        case 1:
            gpio_put(M1_DIR_PIN, direction);
            pwm_set_chan_level(M1_SLICE, M1_CHAN, level);
            break;
        case 2:
            gpio_put(M2_DIR_PIN, direction);
            pwm_set_chan_level(M2_SLICE, M2_CHAN, level);
            break;
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    return 0;
}

// Puts a motor into a zero-throttle state allowing it to spin freely.
// Returns 0 on success, -1 on failure
int mbot_motor_free_spin(uint8_t ch) {
    unsigned int slice, channel;
    
    switch (ch) {
        case 0:
            slice = M0_SLICE; channel = M0_CHAN;
            break;
        case 1:
            slice = M1_SLICE; channel = M1_CHAN;
            break;
        case 2:
            slice = M2_SLICE; channel = M2_CHAN;
            break;
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    
    pwm_set_chan_level(slice, channel, 0);
    
    return 0;
}

// Returns 0 on success, -1 on failure
int mbot_motor_brake(uint8_t ch) {
    unsigned int slice, channel, direction;
    
    switch (ch) {
        case 0:
            slice = M0_SLICE; channel = M0_CHAN; direction = M0_DIR_PIN;
            break;
        case 1:
            slice = M1_SLICE; channel = M1_CHAN; direction = M1_DIR_PIN;
            break;
        case 2:
            slice = M2_SLICE; channel = M2_CHAN; direction = M2_DIR_PIN;
            break;
        default:
            fprintf(stderr, "Invalid channel!\n");
            return -1;
    }
    
    gpio_put(direction, !gpio_get_out_level(direction));
    pwm_set_chan_level(slice, channel, 0);
    
    return 0;
}
