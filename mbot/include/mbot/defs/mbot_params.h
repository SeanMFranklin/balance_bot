#ifndef MBOT_PARAM_DEFS_H
#define MBOT_PARAM_DEFS_H

#define MBOT_ERROR -1
#define MBOT_OK 0

#define SYS_CLOCK       125000 //system clock in kHz
#define PWM_FREQ        25000
#define MAIN_LOOP_HZ            50.0 // Hz of control loop
#define MAIN_LOOP_PERIOD        (1.0f / MAIN_LOOP_HZ)


// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             20.0 // 40.0 for ROB103 encoders

typedef enum robot_type{
    DIFFERENTIAL_DRIVE,
    OMNI_120_DRIVE, // 3 omni wheels spaced 120deg
    ACKERMAN_DRIVE
} robot_type;

// MBot Classic Parameters
#define LEFT_MOTOR              0
#define RIGHT_MOTOR             2
#define WHEEL_DIAMETER          0.0837
#define WHEEL_RADIUS            0.04183
#define WHEEL_BASE              0.15571

// MBot Omni Parameters
#define OMNI_BASE_RADIUS        0.10250     // radius of wheel centers
#define OMNI_WHEEL_RADIUS       0.048       // 0.050 for old wheels
#define OMNI_MOTOR_ANGLE_0 (-M_PI / 3.0f)   // Wheel 0 angle in radians (-60 degrees)
#define OMNI_MOTOR_ANGLE_1 (M_PI)           // Wheel 1 angle in radians (180 degrees)
#define OMNI_MOTOR_ANGLE_2 (M_PI / 3.0f)    // Wheel 2 angle in radians (60 degrees)
#define INV_SQRT3               5.7735026918962575E-1

typedef struct mbot_params_t{
    int robot_type;
    float wheel_radius;
    float wheel_base;
    float gear_ratio;
    float encoder_resolution;
    int motor_polarity[3];
    int encoder_polarity[3];
} mbot_params_t;

#endif