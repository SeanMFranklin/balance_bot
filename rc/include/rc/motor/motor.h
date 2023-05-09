#ifndef RC_MOTOR_H
#define RC_MOTOR_H

typedef enum rc_motor_state {OFF, ON} rc_motor_state;

int rc_motor_init();
int rc_motor_init_freq(uint f);
int rc_motor_cleanup();
int rc_motor_set(uint ch, int32_t duty);
int rc_motor_free_spin(uint ch);
int rc_motor_brake(uint ch);

#endif /* RC_MOTOR_H */
