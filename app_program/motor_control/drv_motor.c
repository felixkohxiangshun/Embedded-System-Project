/*
 *	drv_motor.c
 *	Motor control -- motor PWM driver for the Robo Pico H-bridges.
 *
 *	Each motor has two inputs: FWD and BWD.  Forward = PWM on FWD, BWD low;
 *	reverse = the opposite; stop = both low.
 *	  left : GP8 (FWD) / GP9  (BWD)  -> PWM slice 4, channels A / B
 *	  right: GP10(FWD) / GP11 (BWD)  -> PWM slice 5, channels A / B
 *
 *	Only motor_task calls these functions (one owning task per peripheral).
 *	PWM frequency 20 kHz assumes the 125 MHz system clock:
 *	125 MHz / (TOP + 1) = 20 kHz  ->  TOP = 6249.
 */

#include "motor_control.h"

#define MOTOR_PWM_TOP	6249

LOCAL const UINT pin_fwd[2] = { PIN_MOT_LEFT_FWD,  PIN_MOT_RIGHT_FWD };
LOCAL const UINT pin_bwd[2] = { PIN_MOT_LEFT_BWD,  PIN_MOT_RIGHT_BWD };

EXPORT void motor_hw_init(void)
{
	INT side;

	app_pwm_unreset();
	for (side = 0; side < 2; side++) {
		pwm_set_pin(pin_fwd[side]);
		pwm_set_pin(pin_bwd[side]);
		pwm_set_wrap(pin_fwd[side], MOTOR_PWM_TOP);
		pwm_set_cc(pin_fwd[side], 0);
		pwm_set_cc(pin_bwd[side], 0);
		pwm_set_enabled(pin_fwd[side], TRUE);
	}
}

EXPORT void motor_drive(INT side, INT duty_permille)
{
	INT mag = duty_permille < 0 ? -duty_permille : duty_permille;
	UW  cc;

	if (mag > DUTY_MAX) {
		mag = DUTY_MAX;
	}
	cc = ((UW)mag * (MOTOR_PWM_TOP + 1)) / 1000;
	if (duty_permille >= 0) {
		pwm_set_cc(pin_bwd[side], 0);
		pwm_set_cc(pin_fwd[side], cc);
	} else {
		pwm_set_cc(pin_fwd[side], 0);
		pwm_set_cc(pin_bwd[side], cc);
	}
}

EXPORT void motor_stop_all(void)
{
	motor_drive(0, 0);
	motor_drive(1, 0);
}
