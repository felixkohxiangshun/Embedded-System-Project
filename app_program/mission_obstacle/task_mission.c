/*
 *	task_mission.c
 *	Mission/obstacle -- the Mission State task.
 *
 *	Why a task at all, when the RTOS already schedules by priority?  The
 *	scheduler decides WHEN each task runs; it does not decide WHAT the car
 *	should do when a barcode, a hump and an obstacle are all reported in
 *	the same second.  Mission is the single place where those events are
 *	combined and the system statechart (mission_logic.c) runs.
 *
 *	Every MISSION_TICK_MS it drains its input buffers, runs the statechart,
 *	carries out the resulting actions (event-flag bits for Motor, commands
 *	on mbf_nav and mbf_scancmd), and publishes the results in app_state.
 *	It polls with TMO_POL rather than blocking on one buffer, because it
 *	listens to seven sources and micro T-Kernel cannot wait on several
 *	message buffers at once.  Bounded cost: 5 ms period, short drains.
 */

#include "mission_obstacle.h"
#include "drv_rgb.h"

/* Integrated run (2026-11-09): Mission combines Motor, Line, Barcode and
   IMU.  GP20 starts and stops the car; the line steering goes to Motor; a
   barcode (two bars close together, so not a junction line) makes the car
   back up 5 cm and cross the code at 18 mm/s (normal line tracking is
   60 mm/s); the decoded letter is stored
   and carried out at the next cross junction (Z = U-turn, Q = right turn); an IMU hump slows the car and records the height; an IMU
   fault is reported but the car keeps driving.  An obstacle within 30 cm
   stops the car; it sweeps until one side is clear (or the obstacle goes),
   then goes round it on a 45 degree dog-leg and finds the line again
   (2026-11-17).  Every bench / floor test flag is at its "off" value:
     S5_TEST_STOP_AT_BARCODE 0, S5_TEST_STRAIGHT 0 (here),
     S5_TEST_SERVO_CAL 0, S5_TEST_STATIONARY 0, S5_TEST_ULTRA 0,
     S5_TEST_ULTRA_SIMPLE 0 (mission_obstacle.h), APP_ONLY_OBSTACLE 0
     (app_main.c), S1_TEST_FWD_BTN 0 (motor_ctrl.h). */

/* 0 drops barcode results, so a false read cannot turn the car. */
#ifndef MISSION_USE_BARCODE
#define MISSION_USE_BARCODE	1
#endif

/* 0 drops obstacle reports, so the car never stops for an object.  1 once
   the bypass has been tested (also set S5_OBSTACLE_DETECT 1 in task_scan.c,
   which makes the ultrasonic ping at all). */
#ifndef MISSION_USE_OBSTACLE
#define MISSION_USE_OBSTACLE	1
#endif

/* Barcode floor test: 1 stops the car in IDLE as soon as a barcode has been
   decoded or has failed, instead of acting on it.  Restore 0 after. */
#ifndef S5_TEST_STOP_AT_BARCODE
#define S5_TEST_STOP_AT_BARCODE	0
#endif

/* Obstacle floor test: 1 makes GP20 drive dead straight, ignoring the line
   sensors (no line, no junctions, no line-lost fault).  After a bypass the
   car carries on straight instead of looking for the line.  Restore 0 for
   line following. */
#ifndef S5_TEST_STRAIGHT
#define S5_TEST_STRAIGHT	0
#endif

EXPORT volatile UW s5_mission_ticks;

/* motor_control/motor_ctrl.c: degrees of a pivot whose wheels moved 'counts'
   (half the wheel difference gives the car's heading the same way) */
IMPORT INT counts_to_deg(INT counts);
EXPORT volatile UB s5_mission_state;
EXPORT volatile UB s5_bc_exit;		/* BCX_xxx, for the dashboard */
EXPORT volatile UB s5_byp_result;	/* BYPR_xxx, for the dashboard */

LOCAL void apply(const MISSION_ACT *a)
{
	INT i;

	if (a->evt_flags) {
		tk_set_flg(flg_evt, a->evt_flags);
	}
	if (a->scan_cmd) {
		MSG_SCANCMD c;

		c.cmd = a->scan_cmd;
		c.arg = a->scan_arg;
		MBF_SEND(mbf_scancmd, c);
	}
	for (i = 0; i < a->n_nav; i++) {
		MSG_NAV n = a->nav[i];

		MBF_SEND(mbf_nav, n);
	}
}

LOCAL void publish(const MISSION *m)
{
	if (state_lock(TMO_POL) != E_OK) {
		return;			/* try again next tick */
	}
	app_state.sys_state = m->state;
	app_state.last_barcode = m->last_barcode;
	app_state.stored_char = m->stored_char;
	app_state.hump_mm = m->hump_mm;
	app_state.hump_max_mm = m->hump_max_mm;
	app_state.lifted = m->lifted;
	app_state.line_hint = m->line_hint;
	app_state.obs_dist_mm = m->obs_dist_mm;
	app_state.obs_width_mm = m->obs_width_mm;
	app_state.obs_side = m->obs_side;
	app_state.ir_line = m->ir_line;
	fault_write_locked(FAULT_OWN_MISSION, m->fault_flags);
	state_unlock();
}

/* Status LEDs (the board's two NeoPixels on GP18, drv_rgb.c):
     INIT white, IDLE blue blinking (1 s), line following green, barcode
     purple, hump cyan, obstacle yellow, FAULT_RECOVERY red blinking (0.5 s). */
LOCAL void update_led(UB state, UW ticks)
{
	UW colour, period_ms = 0;

	switch (state) {
	case SYS_INIT:            colour = RGB_WHITE;  break;
	case SYS_IDLE:            colour = RGB_BLUE;   period_ms = 1000; break;
	case SYS_FAULT_RECOVERY:  colour = RGB_RED;    period_ms = 500;  break;
	case SYS_BARCODE_SCAN:
	case SYS_BARCODE_CMD:     colour = RGB_PURPLE; break;
	case SYS_HUMP_TRAVERSE:   colour = RGB_CYAN;   break;
	case SYS_OBSTACLE_SCAN:
	case SYS_OBSTACLE_BYPASS: colour = RGB_YELLOW; break;
	default:                  colour = RGB_GREEN;  break;	/* line follow, reacquire */
	}
	if (period_ms != 0 && ((ticks * MISSION_TICK_MS) % period_ms) >= (period_ms / 2)) {
		colour = RGB_OFF;		/* off half of each blink */
	}
	rgb_set(colour);
}

EXPORT void mission_task(INT stacd, void *exinf)
{
	MISSION      m;
	MISSION_ACT  act;
	MSG_STEER    st;
	MSG_BCCMD    bc;
	MSG_TERRAIN  tr;
	MSG_OBS      ob;
	MSG_MQTTCMD  mq;
	UINT         ptn;
	UB           motor_state = MOT_STOPPED;
	UW           done_seq = 0;
	INT          fwd_mm = 0;
	INT          wheel_diff = 0;	/* left - right encoder counts */

	(void)stacd; (void)exinf;
	(void)rgb_init();		/* status LEDs; dark if no PIO SM is free */
	mission_init(&m);
	m.stop_at_barcode = S5_TEST_STOP_AT_BARCODE;
	m.straight_only = S5_TEST_STRAIGHT;

	while (1) {
		tk_dly_tsk(MISSION_TICK_MS);
		s5_mission_ticks++;
		mission_act_clear(&act);

		/* inputs, highest urgency first: stop requests, then sensors */
		while (MBF_RECV(mbf_mqttcmd, mq, TMO_POL) > 0) {
#if S5_TEST_SERVO_CAL || S5_TEST_STATIONARY || S5_TEST_ULTRA || S5_TEST_ULTRA_SIMPLE
			if (mq.cmd == MQTTCMD_START) {
				continue;	/* bench tests: dashboard Start ignored too */
			}
#endif
			mission_on_mqtt(&m, &mq, &act);
		}
		if (tk_wai_flg(flg_evt, EVT_BTN, TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK) {
#if !S5_TEST_SERVO_CAL && !S5_TEST_STATIONARY && !S5_TEST_ULTRA && !S5_TEST_ULTRA_SIMPLE
			mission_on_button(&m, &act);	/* the bench tests keep the car still */
#endif
		}
		while (MBF_RECV(mbf_terrain, tr, TMO_POL) > 0) {
			mission_on_terrain(&m, &tr, &act);
		}
		while (MBF_RECV(mbf_obs, ob, TMO_POL) > 0) {
#if MISSION_USE_OBSTACLE
			mission_on_obs(&m, &ob, &act);
#endif
		}
		while (MBF_RECV(mbf_bccmd, bc, TMO_POL) > 0) {
#if MISSION_USE_BARCODE
			mission_on_barcode(&m, &bc, &act);
#endif
		}
		while (MBF_RECV(mbf_steer, st, TMO_POL) > 0) {
			mission_on_steer(&m, &st, &act);
		}

		/* what is the Motor doing?  (the completion path from Motor) */
		if (state_lock(TMO_POL) == E_OK) {
			motor_state = app_state.motor_state;
			done_seq = app_state.cmd_done_seq;
			m.net_online = app_state.net_online;
			fwd_mm = app_state.fwd_mm;
			wheel_diff = app_state.enc_left - app_state.enc_right;
			state_unlock();
		}
		mission_on_forward(&m, fwd_mm, &act);
		mission_on_heading(&m, counts_to_deg(wheel_diff / 2));	/* + right */
		mission_on_tick(&m, MISSION_TICK_MS, motor_state, done_seq, &act);

		apply(&act);
		publish(&m);
		s5_mission_state = m.state;
		s5_bc_exit = m.bc_exit;
		s5_byp_result = m.byp_result;
		update_led(m.state, s5_mission_ticks);
	}
}
