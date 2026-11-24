/*
 *	mission_obstacle_test.c
 *	On-target tests for the Mission/obstacle subsystem (Mission, Scan).
 *	Built only with: make TEST=1.  Results: testing/mission_obstacle/TEST_LOG.md
 *
 *	With S5_TEST_REMOTE on, the last test sends START then STOP through
 *	mbf_mqttcmd (the wheels turn for 50 ms: lift the car).
 */

#include "mission_obstacle.h"
#include "../test_report.h"

#ifndef S5_TEST_REMOTE
#define S5_TEST_REMOTE		0	/* INT-03: drives the wheels briefly */
#endif

#if APP_TEST

LOCAL MSG_BCCMD bc(UB kind, UB cmd) { MSG_BCCMD m; m.kind = kind; m.cmd = cmd; return m; }
LOCAL MSG_OBS  ob(UB kind, INT side, INT w) { MSG_OBS m = {0}; m.kind = kind; m.side = side; m.width_mm = w; return m; }

EXPORT void mission_obstacle_test(void)
{
	MISSION     m;
	MISSION_ACT a;
	MSG_BCCMD   b;
	MSG_OBS     o;
	MSG_TERRAIN t = {0};
	MSG_STEER   s = {0};
	MSG_MQTTCMD q;
	SCAN_RESULT r;
	INT         i;

	TEST_INIT();

	/* S5-U01: INIT -> IDLE, button starts, button stops */
	mission_init(&m);
	mission_act_clear(&a);
	mission_on_tick(&m, MS_INIT, MOT_STOPPED, 0, &a);
	TEST_CHECK("S5-U01d", m.state == SYS_INIT, "INIT waits while WiFi/MQTT are not up");
	mission_on_tick(&m, MS_INIT_NET, MOT_STOPPED, 0, &a);
	TEST_CHECK("S5-U01e", m.state == SYS_IDLE, "INIT gives up waiting: car usable offline");
	mission_init(&m);
	m.net_online = 1;
	mission_on_tick(&m, MS_INIT, MOT_STOPPED, 0, &a);
	TEST_CHECK("S5-U01a", m.state == SYS_IDLE, "INIT -> IDLE once online and settled");
	mission_on_button(&m, &a);
	TEST_CHECK("S5-U01b", m.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_START),
		   "button in IDLE -> LINE_FOLLOW, START sent");
	mission_act_clear(&a);
	mission_on_button(&m, &a);
	TEST_CHECK("S5-U01c", m.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "button while running -> IDLE, STOP sent");

	/* S5-U02: steering is forwarded only in line-following states */
	mission_on_button(&m, &a);
	mission_act_clear(&a);
	s.steer = 321;
	mission_on_steer(&m, &s, &a);
	TEST_CHECK("S5-U02a", a.n_nav == 1 && a.nav[0].type == NAV_STEER && a.nav[0].arg == 321,
		   "LINE_FOLLOW forwards steering to Motor");
	b = bc(BC_SEEN, 0); mission_on_barcode(&m, &b, &a);
	TEST_CHECK("S5-U02b", m.state == SYS_BARCODE_SCAN, "barcode seen -> BARCODE_SCAN");

	/* S5-U03: a decoded letter is stored; the move waits for a junction */
	mission_act_clear(&a);
	b = bc(BC_OK, 'Z'); mission_on_barcode(&m, &b, &a);
	TEST_CHECK("S5-U03", m.state == SYS_LINE_FOLLOW && a.n_nav == 0 && m.stored_char == 'Z' &&
		   mission_junction_deg(&m, 'Z') == 180 && mission_junction_deg(&m, 'Q') == 90 &&
		   mission_junction_deg(&m, 'A') == 0,
		   "decoded Z -> stored, no turn now; Z = U-turn, Q = right turn, A unmapped");

	/* S5-U04: a failed decode returns to LINE_FOLLOW with no turn */
	mission_act_clear(&a);
	b = bc(BC_SEEN, 0); mission_on_barcode(&m, &b, &a);
	b = bc(BC_FAIL, 0); mission_on_barcode(&m, &b, &a);
	TEST_CHECK("S5-U04", m.state == SYS_LINE_FOLLOW && a.n_nav == 0, "decode failure -> LINE_FOLLOW, no turn");

	/* S5-U05: hump slows the car and the result is recorded (FR-04/05) */
	mission_act_clear(&a);
	t.kind = TERR_HUMP_START; mission_on_terrain(&m, &t, &a);
	TEST_CHECK("S5-U05a", m.state == SYS_HUMP_TRAVERSE && a.nav[0].type == NAV_SLOW, "hump start -> HUMP_TRAVERSE, slow");
	t.kind = TERR_HUMP_DONE; t.height_mm = 12; t.max_mm = 15; mission_on_terrain(&m, &t, &a);
	TEST_CHECK("S5-U05b", m.state == SYS_LINE_FOLLOW && m.hump_max_mm == 15, "hump done -> LINE_FOLLOW, tallest recorded");
	t.kind = TERR_FAULT; mission_on_terrain(&m, &t, &a);
	TEST_CHECK("S5-U05c", m.state == SYS_LINE_FOLLOW && (m.fault_flags & FAULT_IMU),
		   "IMU fault is noted but the car keeps driving (FR-19)");

	/* S5-U06: obstacle: stop, wait for the car to stop, profile, bypass, reacquire */
	mission_act_clear(&a);
	o = ob(OBS_DETECTED, 0, 0); o.dist_mm = 280; mission_on_obs(&m, &o, &a);
	TEST_CHECK("S5-U06a", m.state == SYS_OBSTACLE_SCAN && (a.evt_flags & EVT_STOP), "obstacle -> OBSTACLE_SCAN, STOP sent");
	mission_act_clear(&a);
	mission_on_tick(&m, 5, MOT_DRIVING, 0, &a);
	TEST_CHECK("S5-U06b", a.scan_cmd == 0, "no profile while the car is still moving");
	mission_on_tick(&m, 5, MOT_STOPPED, 0, &a);
	TEST_CHECK("S5-U06c", a.scan_cmd == SCANCMD_PROFILE, "car stopped -> profile ordered");
	mission_act_clear(&a);
	o = ob(OBS_RESULT, OBS_SIDE_RIGHT, 100); mission_on_obs(&m, &o, &a);
	TEST_CHECK("S5-U06d", m.state == SYS_OBSTACLE_BYPASS && a.nav[0].type == NAV_TURN &&
		   a.nav[0].arg == BYP_TURN_CMD_DEG, "result -> OBSTACLE_BYPASS, turn 45 right");
	for (i = 0; i < 7; i++) {		/* the seven legs of the dog-leg */
		mission_act_clear(&a);
		mission_on_tick(&m, 5, MOT_TURNING, (UW)(7 + i), &a);
		mission_on_tick(&m, 5, MOT_STOPPED, (UW)(8 + i), &a);
	}
	TEST_CHECK("S5-U06e", m.state == SYS_LINE_REACQUIRE, "bypass done -> LINE_REACQUIRE");
	s.flags = STEER_F_LOST;
	for (i = 0; i < LINE_FOUND_TICKS; i++) { mission_on_steer(&m, &s, &a); }
	TEST_CHECK("S5-U06f", m.state == SYS_LINE_REACQUIRE, "line still lost -> keep creeping");
	s.flags = 0;
	for (i = 0; i < LINE_FOUND_TICKS; i++) { mission_on_steer(&m, &s, &a); }
	TEST_CHECK("S5-U06g", m.state == SYS_LINE_FOLLOW, "line found -> LINE_FOLLOW");

	/* S5-U07: faults recover by themselves */
	mission_act_clear(&a);
	s.flags = STEER_F_TIMEOUT; mission_on_steer(&m, &s, &a);
	TEST_CHECK("S5-U07a", m.state == SYS_FAULT_RECOVERY && (a.evt_flags & EVT_STOP), "line lost too long -> FAULT_RECOVERY");
	mission_act_clear(&a);
	mission_on_tick(&m, MS_FAULT_RECOVERY, MOT_STOPPED, 0, &a);
	TEST_CHECK("S5-U07b", m.state == SYS_IDLE && !(a.evt_flags & (EVT_START | EVT_RESUME)),
		   "FAULT_RECOVERY -> IDLE, no restart without BTN_START");

	/* S5-U08: dashboard START and STOP (FR-17) */
	mission_act_clear(&a);
	q.cmd = MQTTCMD_START; mission_on_mqtt(&m, &q, &a);
	TEST_CHECK("S5-U08a", m.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_START), "START drives");
	mission_act_clear(&a);
	q.cmd = MQTTCMD_STOP; mission_on_mqtt(&m, &q, &a);
	TEST_CHECK("S5-U08b", m.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "STOP halts, IDLE");
	q.cmd = MQTTCMD_START; mission_on_mqtt(&m, &q, &a);
	TEST_CHECK("S5-U08c", m.state == SYS_LINE_FOLLOW, "START again -> LINE_FOLLOW");

	/* S5-U09: dead encoder while driving -> FAULT_RECOVERY */
	mission_act_clear(&a);
	mission_on_tick(&m, 5, MOT_ENCODER_FAULT, 0, &a);
	TEST_CHECK("S5-U09", m.state == SYS_FAULT_RECOVERY, "ENCODER_FAULT -> FAULT_RECOVERY");

	/* S5-U10: the 5-degree sweep profile (scan_profile; the full set with a
	   simulated sensor is S5-H31 in the host tests) */
	{
		INT mm[SCAN_STEPS];
		INT k, a;

		/* a box 250 mm ahead seen from 10 deg left to 15 deg right, open
		   floor beyond: on the face = 250 / cos(angle) */
		for (k = 0; k < SCAN_STEPS; k++) {
			a = SCAN_STEP_DEG10_OF(k);
			mm[k] = (a >= -100 && a <= 150) ? (250 * 1000) / scan_cos_pm((a < 0) ? -a : a)
							: SCAN_NO_ECHO;
		}
		TEST_CHECK("S5-U10a", scan_profile(mm, &r) && r.width_mm > 0 && r.width_mm < 300,
			   "box ahead, open floor: profiled, narrow");
		TEST_CHECK("S5-U10b", r.side == OBS_SIDE_LEFT && r.half_right_mm > r.half_left_mm,
			   "it reaches further right: go left");
		/* a flat wall across the whole sweep */
		for (k = 0; k < SCAN_STEPS; k++) {
			a = SCAN_STEP_DEG10_OF(k);
			mm[k] = (250 * 1000) / scan_cos_pm((a < 0) ? -a : a);
		}
		TEST_CHECK("S5-U10c", scan_profile(mm, &r) && r.width_mm > 0 && !r.out_left && !r.out_right,
			   "a flat face across the sweep: the 3 cm band ends where it angles away (their method)");
		mm[SCAN_STEP_CENTRE] = SCAN_NO_ECHO;
		TEST_CHECK("S5-U10d", !scan_profile(mm, &r), "no centre echo -> no profile");
	}

	/* INT-03 (IF-07): a dashboard command reaches Mission. */
	/* INIT waits up to MS_INIT_NET for WiFi/MQTT, so allow for that */
	for (i = 0; i < (MS_INIT_NET / 10) + 200 && s5_mission_state != SYS_IDLE; i++) {
		tk_dly_tsk(10);			/* wait for INIT -> IDLE */
	}
	TEST_CHECK("S5-U11", s5_mission_state == SYS_IDLE, "Mission starts in IDLE after INIT");
#if S5_TEST_REMOTE
	q.cmd = MQTTCMD_START;
	MBF_SEND(mbf_mqttcmd, q);
	tk_dly_tsk(50);
	TEST_CHECK("INT-03a", s5_mission_state == SYS_LINE_FOLLOW, "START on mbf_mqttcmd -> LINE_FOLLOW");
	q.cmd = MQTTCMD_STOP;
	MBF_SEND(mbf_mqttcmd, q);
	tk_dly_tsk(50);
	TEST_CHECK("INT-03b", s5_mission_state == SYS_IDLE, "STOP on mbf_mqttcmd -> IDLE");
#endif

	TEST_SUMMARY("mission_obstacle");
}

#endif /* APP_TEST */
