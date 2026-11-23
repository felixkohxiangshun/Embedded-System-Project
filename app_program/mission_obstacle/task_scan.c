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
EXPORT volatile INT s5_scan_wide_mm[2];	/* 70 deg left, right (probe only) */
EXPORT volatile UB  s5_scan_side;

#define SCAN_PING_MS		60
#define OBSTACLE_MM		300	/* FR-07: <= 30 cm */
#define OBSTACLE_GONE_MM	400	/* sweep: centre further than this (or no
					   echo) = the obstacle has gone */
#define SERVO_SETTLE_MS		350
#define TRACK_REAIM_MS		60	/* a few degrees of re-aim while tracking */
/* Gentle servo moves (2026-11-22): instead of jumping, the servo is walked
   SERVO_STEP_US every SERVO_STEP_MS (about 100 deg/s), then left to settle
   SWEEP_SETTLE_MS before a sweep reading, so the extended mount stops
   swinging and the pings are not taken mid-move. [CAL] */
#define SERVO_STEP_US		15
#define SERVO_STEP_MS		15
#define SWEEP_SETTLE_MS		400
#define SAMPLE_GAP_MS		30
#define DETECT_CONFIRM		2	/* consecutive close readings */
#define PROFILE_WAIT_MS		10000
#define STATIONARY_HOLD_MS	3000	/* bench test: pause between sweeps */

/* 0 ignores SCANCMD_ARM: no pinging, so the car never stops for an
   obstacle (integrated run without obstacle avoidance; see
   MISSION_USE_OBSTACLE in task_mission.c).  1 for obstacle avoidance. */
#ifndef S5_OBSTACLE_DETECT
#define S5_OBSTACLE_DETECT	1
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
	m.extent = r ? (r->side == OBS_SIDE_RIGHT ? r->extent_right
						  : r->extent_left) : SCAN_EXTENT_FAR;
	m.reach_mm = r ? (r->side == OBS_SIDE_RIGHT ? r->reach_right_mm
						    : r->reach_left_mm) : 0;
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
LOCAL INT servo_now = SCAN_US_CENTRE;	/* pulse last sent */

/* Walk the servo to us in small steps (never a jump) */
LOCAL void servo_go(INT us)
{
	while (servo_now != us) {
		if (us > servo_now + SERVO_STEP_US) {
			servo_now += SERVO_STEP_US;
		} else if (us < servo_now - SERVO_STEP_US) {
			servo_now -= SERVO_STEP_US;
		} else {
			servo_now = us;
		}
		servo_set_us(servo_now);
		tk_dly_tsk(SERVO_STEP_MS);
	}
}

LOCAL INT sweep_at(INT us)
{
	INT mm;

	s5_scan_state = SCAN_MOVE_SERVO;
	servo_go(us);				/* gently */
	tk_dly_tsk(SWEEP_SETTLE_MS);
	s5_scan_state = SCAN_MEASURE;
	mm = measure();
	s5_scan_last_mm = mm;
	return mm;
}

/* 7 angles, left to right: 70, 45, 22.5 left, centre, 22.5, 45, 70 right */
LOCAL void profile(INT first_mm, BOOL report)
{
	INT         d[SCAN_ANGLES];
	INT         wide_l, wide_r;
	SCAN_RESULT r = {0};
	INT         i;

	for (i = 0; i < SCAN_ANGLES; i++) {
		s5_scan_mm[i] = 0;
	}
	s5_scan_wide_mm[0] = s5_scan_wide_mm[1] = 0;

	wide_l = sweep_at(scan_angle_us(-SCAN_WIDE_DEG));
	s5_scan_wide_mm[0] = wide_l;
	for (i = 0; i < SCAN_ANGLES; i++) {
		d[i] = sweep_at(scan_pulse_us(i));
		s5_scan_mm[i] = d[i];
	}
	wide_r = sweep_at(scan_angle_us(SCAN_WIDE_DEG));
	s5_scan_wide_mm[1] = wide_r;
	servo_go(SCAN_US_CENTRE);

	s5_scan_state = SCAN_EVALUATE;
	if (report && (d[SCAN_CENTRE] == SCAN_NO_ECHO ||
		       d[SCAN_CENTRE] > OBSTACLE_GONE_MM)) {
		s5_scan_side = SCAN_SIDE_NONE;	/* nothing ahead any more */
		post(OBS_CLEAR, NULL, first_mm);
		return;
	}
	if (scan_evaluate_wide(d, wide_l, wide_r, &r)) {
		s5_scan_side = (r.side == OBS_SIDE_LEFT) ? SCAN_SIDE_LEFT : SCAN_SIDE_RIGHT;
		s5_scan_state = SCAN_POST_RESULT;
		if (report) {
			/* the centre reading: the car has stopped since
			   the obstacle was first seen */
			post(OBS_RESULT, &r, d[SCAN_CENTRE]);
		}
	} else {
		s5_scan_side = SCAN_SIDE_FAIL;
		s5_scan_state = SCAN_FAULT;
		if (report) {
			/* r.side is the less blocked side: Mission goes that way
			   if "no way round" repeats (BYP_MAX_FAILS) */
			post(OBS_FAIL, &r, d[SCAN_CENTRE]);
		}
	}
}

#if S5_TEST_ULTRA_SIMPLE
#define UT_ECHO_PIN		16	/* HC-SR04 ECHO -> GP16 (input) */
#define UT_TRIG_PIN		17	/* HC-SR04 TRIG <- GP17 (output) */
#define UT_PERIOD_MS		100	/* one ping every 100 ms */
#define UT_WAIT_US		40000	/* longest wait for each edge (> 38 ms) */
/* Trigger pulse lengths tried in turn, 10 pings each (the HC-SR04 needs at
   least 10 us; some clones on 3.3 V only fire properly with a longer one).
   The result of each length goes to one sweep field on the dashboard:
   far left = 10 us, near left = 20, centre = 50, near right = 100,
   far right = 200. */
LOCAL const UW ut_trig_us[SCAN_ANGLES] = { 10, 20, 50, 100, 200 };
#define UT_PINGS_EACH		10

LOCAL BOOL ut_echo_high(void)
{
	return (in_w(GPIO_IN) & (1U << UT_ECHO_PIN)) ? TRUE : FALSE;
}

/* Busy-wait until ECHO reads 'level'.  TRUE with the time in *t, or FALSE
   after limit_us. */
LOCAL BOOL ut_wait(BOOL level, UW start, UW limit_us, UW *t)
{
	UW now;

	do {
		now = APP_NOW_US();
		if (ut_echo_high() == level) {
			*t = now;
			return TRUE;
		}
	} while ((UW)(now - start) < limit_us);
	return FALSE;
}

LOCAL INT ut_slot;	/* which trigger length is in use (sweep field) */

LOCAL void ut_report(INT mm, UW rise_us, UW edges)
{
	s5_scan_last_mm = mm;
	s5_scan_mm[ut_slot] = mm;
	s5_echo_rise_us = rise_us;
	s5_echo_edges = edges;
}

/* Never returns. */
LOCAL void ultra_simple_test(void)
{
	UW  t0, t_trig, t_rise, t_fall, n = 0;
	INT mm, i;

	gpio_set_pin(UT_TRIG_PIN, GPIO_MODE_OUT);
	gpio_set_val(UT_TRIG_PIN, 0);
	gpio_set_pin(UT_ECHO_PIN, GPIO_MODE_IN);
	servo_set_us(SCAN_US_CENTRE);
	for (i = 0; i < SCAN_ANGLES; i++) {
		s5_scan_mm[i] = 0;
	}
	s5_scan_side = SCAN_SIDE_NONE;
	tm_printf((UB*)"[ultra-simple] TRIG GP%d, ECHO GP%d, ping every %d ms\n",
		  UT_TRIG_PIN, UT_ECHO_PIN, UT_PERIOD_MS);

	while (1) {
		s5_scan_state = SCAN_MONITORING;
		tk_dly_tsk(UT_PERIOD_MS);
		s5_scan_state = SCAN_MEASURE;
		n++;
		ut_slot = (INT)((n / UT_PINGS_EACH) % SCAN_ANGLES);

		/* 1. ECHO must be low before a new trigger */
		t0 = APP_NOW_US();
		if (!ut_wait(FALSE, t0, UT_WAIT_US, &t_fall)) {
			ut_report(0, 0, 15);
			tm_printf((UB*)"[ultra-simple] %d: ECHO stuck HIGH before trigger\n", (INT)n);
			continue;
		}

		/* 2. trigger pulse on GP17 (length from ut_trig_us) */
		gpio_set_val(UT_TRIG_PIN, 1);
		t0 = APP_NOW_US();
		while ((UW)(APP_NOW_US() - t0) < ut_trig_us[ut_slot]) {
			;
		}
		gpio_set_val(UT_TRIG_PIN, 0);
		t_trig = APP_NOW_US();

		/* 3. ECHO goes high (about 450-500 us later on an HC-SR04) */
		if (!ut_wait(TRUE, t_trig, UT_WAIT_US, &t_rise)) {
			ut_report(0, 0, 0);
			if (n % 10 == 0) {
				tm_printf((UB*)"[ultra-simple] %d: ECHO never went high\n", (INT)n);
			}
			continue;
		}

		/* 4. ECHO goes low again: its width is the round trip */
		if (!ut_wait(FALSE, t_rise, UT_WAIT_US, &t_fall)) {
			ut_report(0, TIMER_COUNTS_TO_US(t_rise - t_trig), 1);
			if (n % 10 == 0) {
				tm_printf((UB*)"[ultra-simple] %d: rose after %d us, never fell (nothing in range?)\n",
					  (INT)n, (INT)TIMER_COUNTS_TO_US(t_rise - t_trig));
			}
			continue;
		}
		/* timer counts -> us (scan_logic.h), then 0.343 mm/us there and back */
		mm = (INT)((TIMER_COUNTS_TO_US(t_fall - t_rise) * 343) / 2000);
		ut_report(mm, TIMER_COUNTS_TO_US(t_rise - t_trig), 2);
		if (n % 10 == 0) {
			tm_printf((UB*)"[ultra-simple] %d: trig %d us: %d mm (echo %d us, rose after %d us)\n",
				  (INT)n, (INT)ut_trig_us[ut_slot], mm,
				  (INT)TIMER_COUNTS_TO_US(t_fall - t_rise),
				  (INT)TIMER_COUNTS_TO_US(t_rise - t_trig));
		}
	}
}
#endif

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
	BOOL        tracking = FALSE;	/* SCANCMD_TRACK: ping at a held angle */
	INT         close_cnt = 0, mm, r;

	(void)stacd; (void)exinf;
#if S5_TEST_SERVO_CAL
	servo_cal();
#endif
#if S5_TEST_ULTRA_SIMPLE
	ultra_simple_test();
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
			if (tracking && c.cmd != SCANCMD_TRACK) {
				tracking = FALSE;	/* any other command ends it */
				servo_go(SCAN_US_CENTRE);
			}
			if (c.cmd == SCANCMD_TRACK) {
				/* Mission goes round an obstacle: aim the sensor
				   where it says the obstacle is and report every
				   reading; a re-aim while tracking is a small move */
				servo_go(scan_angle_us(c.arg));
				tk_dly_tsk(tracking ? TRACK_REAIM_MS : SERVO_SETTLE_MS);
				armed = FALSE;
				tracking = TRUE;
			} else if (c.cmd == SCANCMD_ARM) {
				armed = S5_OBSTACLE_DETECT ? TRUE : FALSE;
				close_cnt = 0;
			} else if (c.cmd == SCANCMD_DISARM) {
				armed = FALSE;
			} else if (c.cmd == SCANCMD_PROFILE) {
				/* Mission asks again after "no way round": the
				   car is still stopped in front of the obstacle */
				profile(s5_scan_last_mm, TRUE);
			}
			continue;
		}
		if (tracking) {
			s5_scan_state = SCAN_MEASURE;
			mm = ultra_ping_raw_mm();	/* no 40 mm / 2 m filter: */
			if (mm > SCAN_MAX_RANGE_MM) {	/* Mission judges near ones */
				mm = SCAN_NO_ECHO;
			}
			s5_scan_last_mm = mm;
			post(OBS_RANGE, NULL, mm);
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
