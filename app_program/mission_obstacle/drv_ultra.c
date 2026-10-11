/*
 *	drv_ultra.c
 *	Mission/obstacle -- HC-SR04 ultrasonic sensor and the scan servo.
 *
 *	Only scan_task calls these functions.
 *
 *	HC-SR04:  TRIG (GP17) gets a 10 us pulse; ECHO (GP16) then goes high for
 *	          as long as the sound takes to go there and back.
 *	          distance = time * 343 m/s / 2.
 *	          *** The HC-SR04 ECHO output is 5 V.  The RP2040 pin is NOT
 *	          *** 5 V tolerant: use a resistor divider (e.g. 1k / 2k) or a
 *	          *** level shifter on ECHO before connecting it.
 *
 *	The echo is timed by polling the 1 MHz hardware timer.  Higher-priority
 *	tasks and the 1 ms handlers can interrupt the poll, which adds a small
 *	error (tens of microseconds = a few millimetres).  Scan runs below
 *	Motor, Line and Mission on purpose, so its poll never delays them.
 *
 *	Servo:    PWM slice 6, channel A (GP12), 50 Hz.  The slice divider is set
 *	          to 125, so one count is 1 us (125 MHz clock assumed) and the CC
 *	          value is the pulse width in microseconds.
 */

#include "mission_obstacle.h"

#define ECHO_RISE_TIMEOUT_US	5000
#define ECHO_FALL_TIMEOUT_US	12000		/* about 2 m */
#define SERVO_SLICE		6
#define SERVO_TOP		19999		/* 20 ms period at 1 MHz */
#define SERVO_CENTER_US		1500
#define SERVO_SLICE_DIV		(PWM_BASE + 0x14 * SERVO_SLICE + PWM_CHx_DIV)

EXPORT void ultra_hw_init(void)
{
	gpio_set_pin(PIN_ULTRA_TRIG, GPIO_MODE_OUT);
	gpio_set_val(PIN_ULTRA_TRIG, 0);
	gpio_set_pin(PIN_ULTRA_ECHO, GPIO_MODE_IN);

	app_pwm_unreset();
	pwm_set_pin(PIN_SERVO_PWM);
	out_w(SERVO_SLICE_DIV, 125u << 4);		/* divider 125.0 (8.4 fixed point) */
	pwm_set_wrap(PIN_SERVO_PWM, SERVO_TOP);
	pwm_set_cc(PIN_SERVO_PWM, SERVO_CENTER_US);
	pwm_set_enabled(PIN_SERVO_PWM, TRUE);
}

/* Angle in degrees, - = left of straight ahead.  A hobby servo moves about
   90 degrees for 1000 us, so 1 degree is about 5.56 us. [CAL] */
EXPORT void servo_set_deg(INT deg)
{
	pwm_set_cc(PIN_SERVO_PWM, (UW)(SERVO_CENTER_US + (deg * 50) / 9));
}

LOCAL void busy_us(UW us)
{
	UW t0 = APP_NOW_US();

	while ((UW)(APP_NOW_US() - t0) < us) {
		;
	}
}

/* One measurement in mm; SCAN_NO_ECHO (0) if no valid echo came back. */
EXPORT INT ultra_ping_mm(void)
{
	UW t0, t1, us;

	gpio_set_val(PIN_ULTRA_TRIG, 1);
	busy_us(10);
	gpio_set_val(PIN_ULTRA_TRIG, 0);

	t0 = APP_NOW_US();
	while (!gpio_get_val(PIN_ULTRA_ECHO)) {
		if ((UW)(APP_NOW_US() - t0) > ECHO_RISE_TIMEOUT_US) {
			return SCAN_NO_ECHO;
		}
	}
	t1 = APP_NOW_US();
	while (gpio_get_val(PIN_ULTRA_ECHO)) {
		if ((UW)(APP_NOW_US() - t1) > ECHO_FALL_TIMEOUT_US) {
			return SCAN_NO_ECHO;
		}
	}
	us = (UW)(APP_NOW_US() - t1);
	return (INT)((us * 343) / 2000);	/* 0.343 mm/us, there and back */
}
