#include "odometry.h"
#include <math.h>

int mbot_calculate_differential_body_vel(float wheel_left_vel, float wheel_right_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  WHEEL_RADIUS * (wheel_left_vel + wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz =  WHEEL_RADIUS * (wheel_right_vel - wheel_left_vel) / WHEEL_BASE;
    return 0; // Return 0 to indicate success
}
int mbot_calculate_differential_body_vel_imu(float wheel_left_vel, float wheel_right_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
        return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel(float wheel0_vel, float wheel1_vel, float wheel2_vel, serial_twist2D_t *mbot_vel){
    // mbot_vel->vx =  (1.0/3.0) * WHEEL_RADIUS * (wheel0_vel * cos(OMNI_MOTOR_ANGLE_0) + wheel1_vel * cos(OMNI_MOTOR_ANGLE_1) + wheel2_vel * cos(OMNI_MOTOR_ANGLE_2));
    // mbot_vel->vy =  (1.0/3.0) * WHEEL_RADIUS * (wheel0_vel * sin(OMNI_MOTOR_ANGLE_0) + wheel1_vel * sin(OMNI_MOTOR_ANGLE_1) + wheel2_vel * sin(OMNI_MOTOR_ANGLE_2));
    // mbot_vel->wz = (wheel0_vel - wheel1_vel + wheel2_vel) / (3.0f * WHEEL_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel_imu(float wheel_0_vel, float wheel_1_vel, float wheel_2_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
    return 0; // Return 0 to indicate success
}

int mbot_calculate_odometry(serial_twist2D_t mbot_vel, float dt, serial_pose2D_t *odometry){
    float vx_space = mbot_vel.vx * cos(odometry->theta) - mbot_vel.vy * sin(odometry->theta);
    float vy_space = mbot_vel.vx * sin(odometry->theta) + mbot_vel.vy * cos(odometry->theta);

    odometry->x += vx_space * dt;
    odometry->y += vy_space * dt;
    odometry->theta += mbot_vel.wz * dt;

    // Normalize theta to be between -pi and pi
    while (odometry->theta > M_PI) odometry->theta -= 2 * M_PI;
    while (odometry->theta <= -M_PI) odometry->theta += 2 * M_PI;

    return 0; // Return 0 to indicate success
}
