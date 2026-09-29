/*
 * motor.h - Motor / Encoders / PID (owner: Dave)
 * Covers FR-11, FR-12, FR-13, NFR-13, NFR-14.
 */
#ifndef MOTOR_H
#define MOTOR_H

// Call once at startup.
void motor_init(void);

// Call once per main-loop iteration (target: every 10ms, per NFR-13).
// TODO(Dave): read both encoders, run the PID loop, set PWM duty cycles.
void motor_update(void);

// TODO(Dave): expose whatever Mission State needs to command the motors,
// e.g.:
// void motor_set_target_speed(float cm_per_s);
// void motor_stop(void);

#endif // MOTOR_H
