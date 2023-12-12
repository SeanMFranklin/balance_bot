#ifndef PID_FILTER_H
#define PID_FILTER_H

#include <stdint.h>

typedef struct PID_t {
  float Kp;         // Proportional gain
  float Ki;         // Integral gain
  float Kd;         // Derivative gain
  float Tf;         // Filter time constant for derivative term
  float error_prev; // Previous error
  float integral;   // Integral term
  float dt;         // Sampling time
} PID_t;

void pid_init(PID_t *pid, float Kp, float Ki, float Kd, float Tf, float dt) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->Tf = Tf;
  pid->error_prev = 0.0f;
  pid->integral = 0.0f;
  pid->dt = dt; // Assign sampling time
}

float pid_update(PID_t *pid, float error) {
  // Calculate the derivative term with high-frequency rolloff
  float derivative = (error - pid->error_prev) / pid->dt;
  derivative += (pid->Tf - pid->dt) * derivative / pid->Tf;

  // Update the integral term
  pid->integral += error * pid->dt;

  // Calculate the proportional, integral, and derivative contributions
  float proportional = pid->Kp * error;
  float integral = pid->Ki * pid->integral;
  float derivative_term = pid->Kd * derivative;

  // Update the previous error
  pid->error_prev = error;

  // Sum the contributions to get the PID output
  return proportional + integral + derivative_term;
}

void pid_set_gains(PID_t *pid, float Kp, float Ki, float Kd, float Tf) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->Tf = Tf;
}

#endif // PID_CONTROLLER_H