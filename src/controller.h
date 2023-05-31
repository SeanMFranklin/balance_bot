#include <mbot_lcm_msgs_serial.h>
#include <mbot/defs/mbot_params.h>

int mbot_init_ctlr(void);
int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_pwm_t &mbot_motor_pwm);
//int mbot_ctlr(serial_twist2D_t vel_cmd, serial_twist2D_t vel, serial_mbot_motor_vel_t &mbot_motor_vel);
