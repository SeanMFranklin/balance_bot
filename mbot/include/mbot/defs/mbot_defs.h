#ifndef RC_DEFS_H
#define RC_DEFS_H

#define SYS_CLOCK       250000 //system clock in kHz
#define PWM_FREQ        25000

#define MAIN_LOOP_HZ            50.0 // Hz of control loop
#define MAIN_LOOP_PERIOD        (1.0f / MAIN_LOOP_HZ)

// Motor Pin Definitions
#define M0_DIR_PIN      14
#define M1_DIR_PIN      15
#define M2_DIR_PIN      13

#define M0_PWM_PIN      2
#define M1_PWM_PIN      3
#define M2_PWM_PIN      12

// Motor PWM Definitions
#define M0_SLICE        1
#define M1_SLICE        1
#define M2_SLICE        6
#define M0_CHAN         0 
#define M1_CHAN         1 
#define M2_CHAN         0  

// Motor Pin Definitions
#define ENC0_A_PIN      6
#define ENC0_B_PIN      7
#define ENC1_A_PIN      8
#define ENC1_B_PIN      9
#define ENC2_A_PIN      10
#define ENC2_B_PIN      11

#define ADC0_PIN        26
#define ADC1_PIN        27
#define ADC2_PIN        28

#define SDA_PIN         4
#define SCL_PIN         5

// LED on the Pico
#define LED_PIN        25

#define MPU_FINAL_FRAM_ADDR     102 // final address of MPU calibration

// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             20.0 // 40.0 for ROB103 encoders

#endif