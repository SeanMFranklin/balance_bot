#include <stdio.h>
#include <stdint.h>
#include <mbot/motor/motor.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#define INT_16_MAX 32768

void drive_motor_up_down(int);
void blink();

int main() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    //adc_select_input(0);


    mbot_motor_init(0);
    mbot_motor_init(1);
    mbot_motor_init(2);

    blink();
    printf("Testing motor 0...\n");
    drive_motor_up_down(1);
    
    blink();
    printf("Testing motor 1...\n");
    drive_motor_up_down(2);
    
    blink();
    printf("Testing motor 2...\n");
    drive_motor_up_down(3);

    blink();
    printf("Done!\n");
    mbot_motor_cleanup(0);
    mbot_motor_cleanup(1);
    mbot_motor_cleanup(2);
    
    blink();
    return 0;
}

void drive_motor_up_down(int motor) {
    int32_t d = 0;
    printf("\tForward\n");
    for (; d < INT_16_MAX; d += 64) {
        mbot_motor_set_duty_int16(motor, d);
        sleep_ms(4);
    }
    for (; d > 0; d -= 64) {
        mbot_motor_set_duty_int16(motor, d);
        sleep_ms(4);
    }
    printf("\tBackward\n");
    for (; d > -INT_16_MAX; d -= 64) {
        mbot_motor_set_duty_int16(motor, d);
        sleep_ms(4);
    }
    for (; d < 0; d += 64) {
        mbot_motor_set_duty_int16(motor, d);
        sleep_ms(4);
    }
}

void blink() {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
