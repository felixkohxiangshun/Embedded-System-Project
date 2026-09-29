#include "motor.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// TODO(Dave): this is a placeholder. Replace with real PID speed control
// reading PIN_LEFT_ENC_A/B and PIN_RIGHT_ENC_A/B, driving
// PIN_LEFT_MOTOR_FWD/BWD and PIN_RIGHT_MOTOR_FWD/BWD.
//
// See motor_test.c (the button demo) for a working example of driving
// these same motor pins via PWM, and the design review's Subsystem
// Statechart 6.2 for the IDLE / PID_ACTIVE / STALL_FAULT / E_STOPPED
// states this task is expected to implement.

void motor_init(void) {
    // TODO(Dave): gpio_init the encoder pins, set up PWM on the motor pins
}

void motor_update(void) {
    // TODO(Dave): PID loop body
}
