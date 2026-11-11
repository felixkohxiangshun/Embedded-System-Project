/*
 *	task_scan.c
 *	Mission/obstacle -- the Obstacle Scanning task (design review 6.4).
 *
 *	States  MONITORING        ping ahead while the car drives
 *	        OBSTACLE_DETECTED object <= 30 cm: tell Mission, wait for the stop
 *	        MOVE_SERVO / MEASURE  five positions, far left to far right; settle,
 *	                              take one (median of three) reading each
 *	        EVALUATE          compare left and right (scan_logic.c)
 *	        POST_RESULT       send the chosen side to Mission on mbf_obs
 *	        SCAN_FAULT        no usable echo or no room: report, carry on
 *
 *	Mission arms and disarms pinging and orders the profile through
 *	mbf_scancmd.  This task never commands the motors.  The mbf_scancmd wait
 *	doubles as the ping timer: a timeout of SCAN_PING_MS means "ping now".
 */

#include "mission_obstacle.h"

EXPORT volatile UB s5_scan_state;
EXPORT volatile INT s5_scan_last_mm;
EXPORT volatile INT s5_scan_mm[5];
EXPORT volatile UB  s5_scan_side;

#define SCAN_PING_MS		60
#define OBSTACLE_MM		300	/* FR-07: <= 30 cm */
#define SERVO_SETTLE_MS		350
#define SAMPLE_GAP_MS		30
#define DETECT_CONFIRM		2	/* consecutive close readings */
#define PROFILE_WAIT_MS		10000
#define STATIONARY_HOLD_MS	3000	/* bench test: pause between sweeps */

/* 0 ignores SCANCMD_ARM: no pinging, so the car never stops for an
   obstacle (integrated run without obstacle avoidance; see
   MISSION_USE_OBSTACLE in task_mission.c).  1 for obstacle avoidance. */
#ifndef S5_OBSTACLE_DETECT
#define S5_OBSTACLE_DETECT	0
#endif

LOCAL void post(UB kind, const SCAN_RESULT *r, INT dist)
{
	MSG_OBS m;

	m.kind = kind;
	m.dist_mm = dist;
	m.side = r ? r->side : 0;
	m.width_mm = r ? r->width_mm : 0;
	m.clear_mm = r ? (r->side == OBS_SIDE_RIGHT ? r->clear_right_mm
						    : r->clear_left_mm) : 0;
	MBF_SEND(mbf_obs, m);
}

/* Median of three pings, ignoring none: a missing echo counts as 0 */
LOCAL INT measure(void)
{
	INT a, b, c, t;

	a = ultra_ping_mm();
	tk_dly_tsk(SAMPLE_GAP_MS);
	b = ultra_ping_mm();
	tk_dly_tsk(SAMPLE_GAP_MS);
	c = ultra_ping_mm();
	if (a > b) { t = a; a = b; b = t; }
	if (b > c) { t = b; b = c; c = t; }
	if (a > b) { t = a; a = b; b = t; }
	return b;
}

/* Sweep left, centre, right and choose a side.  report = FALSE on the bench
   test: the result only goes to the dashboard, not to Mission. */
LOCAL void profile(INT first_mm, BOOL report)
{
	INT         d[SCAN_ANGLES];
	SCAN_RESULT r;
	INT         i;

	for (i = 0; i < SCAN_ANGLES; i++) {
		s5_scan_mm[i] = 0;
	}
	for (i = 0; i < SCAN_ANGLES; i++) {
		s5_scan_state = SCAN_MOVE_SERVO;
		servo_set_us(scan_pulse_us(i));
		tk_dly_tsk(SERVO_SETTLE_MS);
		s5_scan_state = SCAN_MEASURE;
		d[i] = measure();
		s5_scan_mm[i] = d[i];
		s5_scan_last_mm = d[i];
	}
	servo_set_us(SCAN_US_CENTRE);

	s5_scan_state = SCAN_EVALUATE;
	if (scan_evaluate(d, &r)) {
		s5_scan_side = (r.side == OBS_SIDE_LEFT) ? SCAN_SIDE_LEFT : SCAN_SIDE_RIGHT;
		s5_scan_state = SCAN_POST_RESULT;
		if (report) {
			post(OBS_RESULT, &r, first_mm);
		}
	} else {
		s5_scan_side = SCAN_SIDE_FAIL;
		s5_scan_state = SCAN_FAULT;
		if (report) {
			post(OBS_FAIL, NULL, first_mm);
		}
	}
}

#if S5_TEST_ULTRA
#define ULTRA_TEST_MS		100

/* Sensor test: never returns.  Ping at one position; GP20 / GP21 step it. */
LOCAL void ultra_test(void)
{
	INT pos = SCAN_CENTRE, i, mm;
	UW  in, prev = 0xFFFFFFFFu;

	gpio_set_pin(PIN_BTN_2, GPIO_MODE_IN);
	set_w(GPIO(PIN_BTN_2), GPIO_PUE);	/* active low, like GP20 */
	for (i = 0; i < SCAN_ANGLES; i++) {
		s5_scan_mm[i] = 0;
	}
	s5_scan_side = SCAN_SIDE_NONE;
	servo_set_us(scan_pulse_us(pos));
	tm_printf((UB*)"[ultra] sensor test: GP20 left, GP21 right, at %d us\n",
		  scan_pulse_us(pos));

	while (1) {
		s5_scan_state = SCAN_MEASURE;
		tk_dly_tsk(ULTRA_TEST_MS);
		in = in_w(GPIO_IN);
		if (((in & (1U << PIN_BTN_START)) == 0U) && ((prev & (1U << PIN_BTN_START)) != 0U)
		    && pos > 0) {
			pos--;
		} else if (((in & (1U << PIN_BTN_2)) == 0U) && ((prev & (1U << PIN_BTN_2)) != 0U)
			   && pos < SCAN_ANGLES - 1) {
			pos++;
		}
		if ((in ^ prev) & ((1U << PIN_BTN_START) | (1U << PIN_BTN_2))) {
			s5_scan_state = SCAN_MOVE_SERVO;
			servo_set_us(scan_pulse_us(pos));
			tk_dly_tsk(SERVO_SETTLE_MS);
		}
		prev = in;

		mm = ultra_ping_raw_mm();
		s5_scan_last_mm = mm;
		s5_scan_mm[pos] = mm;
	}
}
#endif

#if S5_TEST_STATIONARY
/* Bench test: never returns.  Ping, sweep on a close object, pause, repeat. */
LOCAL void stationary_test(void)
{
	INT close_cnt = 0, mm;

	tm_printf((UB*)"[scan] stationary test: sweep on an object within %d mm\n", OBSTACLE_MM);
	while (1) {
		s5_scan_state = SCAN_MONITORING;
		tk_dly_tsk(SCAN_PING_MS);
		mm = ultra_ping_mm();
		s5_scan_last_mm = mm;
		if (mm != SCAN_NO_ECHO && mm <= OBSTACLE_MM) {
			close_cnt++;
		} else {
			close_cnt = 0;
		}
		if (close_cnt < DETECT_CONFIRM) {
			continue;
		}
		s5_scan_state = SCAN_OBSTACLE_DETECTED;
		tk_dly_tsk(SERVO_SETTLE_MS);	/* long enough to see on the page */
		profile(mm, FALSE);
		tm_printf((UB*)"[scan] %d %d %d %d %d mm -> %s\n",
			  s5_scan_mm[0], s5_scan_mm[1], s5_scan_mm[2],
			  s5_scan_mm[3], s5_scan_mm[4],
			  s5_scan_side == SCAN_SIDE_LEFT ? "left" :
			  s5_scan_side == SCAN_SIDE_RIGHT ? "right" : "no way round");
		tk_dly_tsk(STATIONARY_HOLD_MS);
		close_cnt = 0;
	}
}
#endif

#if S5_TEST_SERVO_CAL
#define CAL_POLL_MS		20
#define CAL_STEP_US		10
#define CAL_HOLD_TICKS		25	/* 500 ms held, then repeat */
#define CAL_REPEAT_TICKS	5	/* every 100 ms */

/* Servo calibration: never returns.  GP20 = -10 us, GP21 = +10 us. */
LOCAL void servo_cal(void)
{
	INT us = SCAN_US_CENTRE, held = 0, dir, prev_dir = 0;
	UW  in;

	gpio_set_pin(PIN_BTN_2, GPIO_MODE_IN);
	set_w(GPIO(PIN_BTN_2), GPIO_PUE);	/* active low, like GP20 */
	servo_set_us(us);
	s5_scan_last_mm = us;
	tm_printf((UB*)"[servo cal] GP20 -10 us, GP21 +10 us, now %d us\n", us);

	while (1) {
		tk_dly_tsk(CAL_POLL_MS);
		in = in_w(GPIO_IN);
		dir = 0;
		if ((in & (1U << PIN_BTN_START)) == 0U) {
			dir = -1;
		} else if ((in & (1U << PIN_BTN_2)) == 0U) {
			dir = 1;
		}

		if (dir == 0 || dir != prev_dir) {
			held = 0;
		} else {
			held++;
		}
		if (dir != 0 && (held == 0 ||
		    (held >= CAL_HOLD_TICKS && (held - CAL_HOLD_TICKS) % CAL_REPEAT_TICKS == 0))) {
			us += dir * CAL_STEP_US;
			if (us < 500) { us = 500; }
			if (us > 2500) { us = 2500; }
			servo_set_us(us);
			s5_scan_last_mm = us;
			tm_printf((UB*)"[servo cal] %d us\n", us);
		}
		prev_dir = dir;
	}
}
#endif

EXPORT void scan_task(INT stacd, void *exinf)
{
	MSG_SCANCMD c;
	BOOL        armed = FALSE;
	INT         close_cnt = 0, mm, r;

	(void)stacd; (void)exinf;
#if S5_TEST_SERVO_CAL
	servo_cal();
#endif
#if S5_TEST_ULTRA
	ultra_test();
#endif
#if S5_TEST_STATIONARY
	servo_set_us(SCAN_US_CENTRE);
	stationary_test();
#endif
	servo_set_us(SCAN_US_CENTRE);

	while (1) {
		s5_scan_state = SCAN_MONITORING;
		r = MBF_RECV(mbf_scancmd, c, SCAN_PING_MS);
		if (r > 0) {
			if (c.cmd == SCANCMD_ARM) {
				armed = S5_OBSTACLE_DETECT ? TRUE : FALSE;
				close_cnt = 0;
			} else if (c.cmd == SCANCMD_DISARM) {
				armed = FALSE;
			}
			continue;
		}
		if (!armed) {
			continue;
		}

		mm = ultra_ping_mm();
		s5_scan_last_mm = mm;
		if (mm != SCAN_NO_ECHO && mm <= OBSTACLE_MM) {
			close_cnt++;
		} else {
			close_cnt = 0;
		}
		if (close_cnt < DETECT_CONFIRM) {
			continue;
		}

		/* OBSTACLE_DETECTED: tell Mission, then wait until it has stopped
		   the car and asks for the profile (or disarms us). */
		s5_scan_state = SCAN_OBSTACLE_DETECTED;
		post(OBS_DETECTED, NULL, mm);
		close_cnt = 0;
		armed = FALSE;
		while (1) {
			r = MBF_RECV(mbf_scancmd, c, PROFILE_WAIT_MS);
			if (r <= 0) {
				break;			/* Mission gave up: monitor again */
			}
			if (c.cmd == SCANCMD_PROFILE) {
				profile(mm, TRUE);
				break;
			}
			if (c.cmd == SCANCMD_ARM) {
				armed = TRUE;
			}
			if (c.cmd == SCANCMD_DISARM) {
				break;
			}
		}
	}
}
