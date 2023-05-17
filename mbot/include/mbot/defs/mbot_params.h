#ifndef MBOT_PARAM_DEFS_H
#define MBOT_PARAM_DEFS_H

#define MBOT_ERROR -1
#define MBOT_OK 0

#define SYS_CLOCK       250000 //system clock in kHz
#define PWM_FREQ        25000
#define MAIN_LOOP_HZ            50.0 // Hz of control loop
#define MAIN_LOOP_PERIOD        (1.0f / MAIN_LOOP_HZ)


// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             20.0 // 40.0 for ROB103 encoders

typedef enum{
    DIFFERENTIAL_DRIVE,
    OMNI_120_DRIVE, // 3 omni wheels spaced 120deg
    ACKERMAN_DRIVE
} robot_type;

// MBot Classic Parameters
#define LEFT_MOTOR              0
#define RIGHT_MOTOR             2
#define WHEEL_RADIUS            0.08
#define WHEEL_BASE              0.15

// MBot Omni Parameters
#define OMNI_BASE_RADIUS        0.10250     // radius of wheel centers
#define OMNI_WHEEL_RADIUS       0.048       // 0.050 for old wheels
#define OMNI_MOTOR_ANGLE_0 (-M_PI / 3.0f)   // Wheel 0 angle in radians (-60 degrees)
#define OMNI_MOTOR_ANGLE_1 (M_PI)           // Wheel 1 angle in radians (180 degrees)
#define OMNI_MOTOR_ANGLE_2 (M_PI / 3.0f)    // Wheel 2 angle in radians (60 degrees)
#define INV_SQRT3               5.7735026918962575E-1

typedef struct {
    int robot_type;
    float wheel_radius;
    float encoder_resolution;
    float gear_ratio;
    int wheel_base;
} mbot_params;

#endif