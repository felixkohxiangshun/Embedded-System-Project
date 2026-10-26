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
 *	The echo is timed by the GPIO interrupt, not by polling (Tribunal 1: a
 *	busy-wait of up to ~17 ms per ping would starve the 10 ms IMU task).
 *	scan_task pulses TRIG, then BLOCKS on flg_echo with a 30 ms timeout.  The
 *	bank-0 GPIO interrupt (motor_control/gpio_isr.c, the only handler for that
 *	interrupt line) calls ultra_echo_isr() on each ECHO edge; that stamps the
 *	1 MHz timer and, on the falling edge, sets flg_echo.  A timeout means no
 *	object in range.  Only the 10 us TRIG pulse is still a busy-wait.
 *	The capture logic is pure and unit-tested on a PC (echo_edge, scan_logic.c).
 *
 *	Servo:    PWM slice 6, channel A (GP12), 50 Hz.  The slice divider is set
 *	          to 125, so one count is 1 us (125 MHz clock assumed) and the CC
 *	          value is the pulse width in microseconds.
 */

#include "mission_obstacle.h"

/* IO_BANK0 register 2 covers pins 16..23 (ECHO is GP16); see gpio_isr.c for the layout */
#define IO_INTR2		(IO_BANK0_BASE + 0x0F0 + 4 * 2)
#define IO_PROC0_INTE2		(IO_BANK0_BASE + 0x100 + 4 * 2)
#define ECHO_EDGE_BITS		(0xCu << (4 * (PIN_ULTRA_ECHO & 7)))	/* edge low | edge high */
#define SERVO_SLICE		6
#define SERVO_TOP		19999		/* 20 ms period at 1 MHz */
#define SERVO_CENTER_US		1500
#define SERVO_SLICE_DIV		(PWM_BASE + 0x14 * SERVO_SLICE + PWM_CHx_DIV)

LOCAL ECHO echo;		/* shared by the interrupt and scan_task */

/* Called from the bank-0 GPIO interrupt (gpio_isr.c).  Never blocks. */
EXPORT void ultra_echo_isr(UW now_us, UW edges)
{
	if (echo_edge(&echo, now_us, edges)) {
		tk_set_flg(flg_echo, ECHO_FLG_DONE);
	}
}

/* Must run after motor_control_start(), which installs the bank-0 handler. */
EXPORT void ultra_hw_init(void)
{
	gpio_set_pin(PIN_ULTRA_TRIG, GPIO_MODE_OUT);
	gpio_set_val(PIN_ULTRA_TRIG, 0);
	gpio_set_pin(PIN_ULTRA_ECHO, GPIO_MODE_IN);

	/* clear anything latched, then let both ECHO edges interrupt core 0 */
	out_w(IO_INTR2, ECHO_EDGE_BITS);
	out_w(IO_PROC0_INTE2, in_w(IO_PROC0_INTE2) | ECHO_EDGE_BITS);

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

/* One measurement in mm; SCAN_NO_ECHO (0) if no valid echo came back.
   Blocks (no CPU use) for at most ECHO_TIMEOUT_MS. */
EXPORT INT ultra_ping_mm(void)
{
	UINT ptn;
	ER   er;
	INT  mm;

	tk_clr_flg(flg_echo, 0);		/* drop a completion left over from a late echo */
	echo_arm(&echo);

	gpio_set_val(PIN_ULTRA_TRIG, 1);
	busy_us(10);
	gpio_set_val(PIN_ULTRA_TRIG, 0);

	er = tk_wai_flg(flg_echo, ECHO_FLG_DONE, TWF_ORW | TWF_BITCLR, &ptn, ECHO_TIMEOUT_MS);
	if (er < E_OK) {
		echo_disarm(&echo);		/* ignore the edge if it arrives later */
		return SCAN_NO_ECHO;
	}
	mm = echo_us_to_mm(echo.width_us);
	return mm;
}
