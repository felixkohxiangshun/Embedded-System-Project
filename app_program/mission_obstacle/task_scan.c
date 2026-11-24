/*
 *	task_scan.c
 *	Mission/obstacle -- the Obstacle Scanning task (design review 6.4).
 *
 *	States  MONITORING        ping ahead while the car drives
 *	        OBSTACLE_DETECTED object <= 30 cm: tell Mission, wait for the stop
 *	        MOVE_SERVO / MEASURE  every 5 deg: straight ahead, out to 55 deg
 *	                              left, back to the middle, out to 55 deg
 *	                              right; settle, take one (median of
 *	                              three) reading each
 *	        EVALUATE          width and side (scan_profile, scan_logic.c)
 *	        POST_RESULT       send the chosen side to Mission on mbf_obs
 *	        SCAN_FAULT        no usable echo or no room: report, carry on
 *	        (SCANCMD_TRACK)   bypass straight leg: ping at a held sideways
 *	                          angle, every reading to Mission as OBS_RANGE
 *
 *	Mission arms and disarms pinging and orders the profile through
 *	mbf_scancmd.  This task never commands the motors.  The mbf_scancmd wait
 *	doubles as the ping timer: a timeout of SCAN_PING_MS means "ping now".
 */

#include "mission_obstacle.h"

EXPORT volatile UB s5_scan_state;
EXPORT volatile INT s5_scan_last_mm;
EXPORT volatile UB  s5_scan_side;
EXPORT volatile INT s5_scan_width_mm;	/* measured width, 0 = none */
EXPORT volatile INT s5_scan_half_mm[2];	/* left, right half widths */

#define SCAN_PING_MS		60
#define OBSTACLE_MM		300	/* FR-07: <= 30 cm */
#define OBSTACLE_GONE_MM	400	/* sweep: centre further than this (or no
					   echo) = the obstacle has gone */
#define SERVO_SETTLE_MS		350
/* Gentle servo moves (2026-11-22): instead of jumping, the servo is walked
   SERVO_STEP_US every SERVO_STEP_MS (about 100 deg/s), then left to settle
   SWEEP_SETTLE_MS before a sweep reading, so the extended mount stops
   swinging and the pings are not taken mid-move. [CAL] */
#define SERVO_STEP_US		15
#define SERVO_STEP_MS		15
#define SWEEP_SETTLE_MS		400
#define SAMPLE_GAP_MS		30
#define FINE_SETTLE_MS		200	/* a 5 deg step settles sooner than a jump [CAL] */
#define DETECT_CONFIRM		2	/* consecutive close readings */
#define PROFILE_WAIT_MS		10000

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
	m.reach_mm = r ? (r->side == OBS_SIDE_RIGHT ? r->half_right_mm
						    : r->half_left_mm) : 0;
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

LOCAL INT servo_now = SCAN_US_CENTRE;	/* pulse last sent to the servo */

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

/* Point the sensor at step i, settle and measure it */
LOCAL INT read_step(INT i, INT settle_ms)
{
	INT a = SCAN_STEP_DEG10_OF(i), mm;

	s5_scan_state = SCAN_MOVE_SERVO;
	servo_go(scan_angle10_us(a));		/* gently */
	tk_dly_tsk(settle_ms);
	s5_scan_state = SCAN_MEASURE;
	mm = measure();
	s5_scan_last_mm = mm;
	return mm;
}

/* The sweep: straight ahead, then out to SCAN_SWEEP_DEG left in
   SCAN_STEP_DEG10 steps, back to the middle, then out to SCAN_SWEEP_DEG
   right, then scan_profile() for the width and the side (scan_logic.h),
   and the result to Mission */
LOCAL void profile(INT first_mm)
{
	INT         mm[SCAN_STEPS];
	SCAN_RESULT r;
	BOOL        ok;
	INT         i;
	const INT   lo = SCAN_STEP_CENTRE - (SCAN_SWEEP_DEG * 10) / SCAN_STEP_DEG10;
	const INT   hi = SCAN_STEP_CENTRE + (SCAN_SWEEP_DEG * 10) / SCAN_STEP_DEG10;

	for (i = 0; i < SCAN_STEPS; i++) {
		mm[i] = SCAN_NO_ECHO;		/* not read */
	}
	/* the last width stays on the dashboard until this sweep has a new one */

	mm[SCAN_STEP_CENTRE] = read_step(SCAN_STEP_CENTRE, SWEEP_SETTLE_MS);
	for (i = SCAN_STEP_CENTRE - 1; i >= lo; i--) {		/* left */
		mm[i] = read_step(i, FINE_SETTLE_MS);
	}
	servo_go(SCAN_US_CENTRE);				/* back to the middle */
	for (i = SCAN_STEP_CENTRE + 1; i <= hi; i++) {		/* right */
		mm[i] = read_step(i, (i == SCAN_STEP_CENTRE + 1) ? SWEEP_SETTLE_MS : FINE_SETTLE_MS);
	}
	servo_go(SCAN_US_CENTRE);

	s5_scan_state = SCAN_EVALUATE;
	if (mm[SCAN_STEP_CENTRE] == SCAN_NO_ECHO || mm[SCAN_STEP_CENTRE] > OBSTACLE_GONE_MM) {
		s5_scan_side = SCAN_SIDE_NONE;	/* nothing ahead any more */
		post(OBS_CLEAR, NULL, first_mm);
		return;
	}
	ok = scan_profile(mm, &r);
	(void)scan_width_calibrated(&r, mm[SCAN_STEP_CENTRE]);	/* this car's sensor */
	s5_scan_half_mm[0] = r.half_left_mm;
	s5_scan_half_mm[1] = r.half_right_mm;
	s5_scan_width_mm = r.width_mm;
	if (ok) {
		s5_scan_side = (r.side == OBS_SIDE_LEFT) ? SCAN_SIDE_LEFT : SCAN_SIDE_RIGHT;
		s5_scan_state = SCAN_POST_RESULT;
		/* the centre reading: the car has stopped since the obstacle
		   was first seen */
		post(OBS_RESULT, &r, mm[SCAN_STEP_CENTRE]);
	} else {
		s5_scan_side = SCAN_SIDE_FAIL;
		s5_scan_state = SCAN_FAULT;
		/* r.side is the less blocked side: Mission goes that way if
		   "no way round" repeats (BYP_MAX_FAILS) */
		post(OBS_FAIL, &r, mm[SCAN_STEP_CENTRE]);
	}
}

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
				/* bypass straight leg: aim the sensor sideways at
				   the obstacle and report every reading */
				servo_go(scan_angle_us(c.arg));
				tk_dly_tsk(SERVO_SETTLE_MS);
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
				profile(s5_scan_last_mm);
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
				profile(mm);
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
