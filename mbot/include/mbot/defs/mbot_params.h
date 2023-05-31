#ifndef MBOT_PARAM_DEFS_H
#define MBOT_PARAM_DEFS_H

#define MBOT_ERROR -1
#define MBOT_OK 0
#define COMMS_ERROR 0
#define COMMS_OK 1
#define MBOT_TIMEOUT_US 1000000


#define SYS_CLOCK       125000 //system clock in kHz
#define PWM_FREQ        10000
#define MAIN_LOOP_HZ            50.0 // Hz of control loop
#define MAIN_LOOP_PERIOD        (1.0f / MAIN_LOOP_HZ)


// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             20.0 // 40.0 for ROB103 encoders

#define DIFFERENTIAL_DRIVE 1
#define OMNI_120_DRIVE 2 // 3 omni wheels spaced 120deg
#define ACKERMAN_DRIVE 3

// MBot Classic Parameters
#define WHEEL_DIAMETER          0.0837
#define WHEEL_RADIUS            0.04183
#define WHEEL_BASE              0.15571

// MBot Omni Parameters
#define OMNI_BASE_RADIUS        0.10250     // radius of wheel centers
#define OMNI_WHEEL_RADIUS       0.048       // 0.050 for old wheels
#define OMNI_MOTOR_ANGLE_LFT (-M_PI / 3.0f)   // Wheel 0 angle in radians (-60 degrees)
#define OMNI_MOTOR_ANGLE_BCK (M_PI)           // Wheel 1 angle in radians (180 degrees)
#define OMNI_MOTOR_ANGLE_RGT (M_PI / 3.0f)    // Wheel 2 angle in radians (60 degrees)
#define INV_SQRT3               5.7735026918962575E-1

typedef struct mbot_params_t{
    int robot_type;
    float wheel_radius;
    float wheel_base;
    float gear_ratio;
    float encoder_resolution;
    int mot_left;
    int mot_right;
    int mot_back;
    int motor_polarity[3];
    int encoder_polarity[3];
    float slope_pos[3];
    float itrcpt_pos[3];
    float slope_neg[3];
    float itrcpt_neg[3];
} mbot_params_t;



#endif