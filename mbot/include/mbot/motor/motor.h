#ifndef MBOT_MOTOR_H
#define MBOT_MOTOR_H

typedef enum mbot_motor_state {OFF, ON} mbot_motor_state;

int mbot_motor_init();
int mbot_motor_init_freq(uint16_t f);
int mbot_motor_cleanup();
int mbot_motor_set(uint8_t ch, int32_t duty);
int mbot_motor_free_spin(uint8_t ch);
int mbot_motor_brake(uint8_t ch);

#endif /* MBOT_MOTOR_H */
