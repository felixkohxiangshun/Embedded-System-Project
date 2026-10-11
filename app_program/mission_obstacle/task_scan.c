/*
 *	task_scan.c
 *	Mission/obstacle -- the Obstacle Scanning task (design review 6.4).
 *
 *	States  MONITORING        ping ahead while the car drives
 *	        OBSTACLE_DETECTED object <= 30 cm: tell Mission, wait for the stop
 *	        MOVE_SERVO / MEASURE  step through -60..+60 degrees, settle,
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

#define SCAN_PING_MS		60
#define OBSTACLE_MM		300	/* FR-07: <= 30 cm */
#define SERVO_SETTLE_MS		350
#define SAMPLE_GAP_MS		30
#define DETECT_CONFIRM		2	/* consecutive close readings */
#define PROFILE_WAIT_MS		10000

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

LOCAL void profile(INT first_mm)
{
	INT         d[SCAN_ANGLES];
	SCAN_RESULT r;
	INT         i;

	for (i = 0; i < SCAN_ANGLES; i++) {
		s5_scan_state = SCAN_MOVE_SERVO;
		servo_set_deg(scan_angle_deg(i));
		tk_dly_tsk(SERVO_SETTLE_MS);
		s5_scan_state = SCAN_MEASURE;
		d[i] = measure();
	}
	servo_set_deg(0);

	s5_scan_state = SCAN_EVALUATE;
	if (scan_evaluate(d, &r)) {
		s5_scan_state = SCAN_POST_RESULT;
		post(OBS_RESULT, &r, first_mm);
	} else {
		s5_scan_state = SCAN_FAULT;
		post(OBS_FAIL, NULL, first_mm);
	}
}

EXPORT void scan_task(INT stacd, void *exinf)
{
	MSG_SCANCMD c;
	BOOL        armed = FALSE;
	INT         close_cnt = 0, mm, r;

	(void)stacd; (void)exinf;
	servo_set_deg(0);

	while (1) {
		s5_scan_state = SCAN_MONITORING;
		r = MBF_RECV(mbf_scancmd, c, SCAN_PING_MS);
		if (r > 0) {
			if (c.cmd == SCANCMD_ARM) {
				armed = TRUE;
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
