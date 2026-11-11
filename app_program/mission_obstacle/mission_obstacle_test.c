/*
 *	mission_obstacle_test.c
 *	On-target tests for the Mission/obstacle subsystem (Mission, Scan).
 *	Built only with: make TEST=1.  Results: testing/mission_obstacle/TEST_LOG.md
 *
 *	With S5_TEST_ESTOP on, the last test sends E_STOP, so it must stay the
 *	last thing that runs.  Off, GP20 keeps working after the tests.
 */

#include "mission_obstacle.h"
#include "../test_report.h"

#ifndef S5_TEST_ESTOP
#define S5_TEST_ESTOP		0	/* INT-03: leaves the system in E_STOP */
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
	b = bc(BC_OK, 'F'); mission_on_barcode(&m, &b, &a);
	TEST_CHECK("S5-U03", m.state == SYS_LINE_FOLLOW && a.n_nav == 0 && m.stored_char == 'F' &&
		   mission_junction_deg(&m, 'F') == 90 && mission_junction_deg(&m, 'A') == 0,
		   "decoded F -> stored, no turn now; F = right turn at the junction, A unmapped");

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
	TEST_CHECK("S5-U06d", m.state == SYS_OBSTACLE_BYPASS && a.nav[0].type == NAV_BYPASS &&
		   a.nav[0].arg == OBS_SIDE_RIGHT, "result -> OBSTACLE_BYPASS");
	mission_on_tick(&m, 5, MOT_BYPASS, 7, &a);
	mission_act_clear(&a);
	mission_on_tick(&m, 5, MOT_STOPPED, 8, &a);
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

	/* S5-U08: remote E_STOP and RESUME (FR-17) */
	mission_act_clear(&a);
	q.cmd = MQTTCMD_ESTOP; mission_on_mqtt(&m, &q, &a);
	TEST_CHECK("S5-U08a", m.state == SYS_E_STOP && (a.evt_flags & EVT_STOP), "E_STOP halts the car");
	mission_on_button(&m, &a);
	TEST_CHECK("S5-U08b", m.state == SYS_E_STOP, "the button cannot clear E_STOP");
	q.cmd = MQTTCMD_RESUME; mission_on_mqtt(&m, &q, &a);
	TEST_CHECK("S5-U08c", m.state == SYS_LINE_FOLLOW, "RESUME -> LINE_FOLLOW");

	/* S5-U09: dead encoder while driving -> FAULT_RECOVERY */
	mission_act_clear(&a);
	mission_on_tick(&m, 5, MOT_ENCODER_FAULT, 0, &a);
	TEST_CHECK("S5-U09", m.state == SYS_FAULT_RECOVERY, "ENCODER_FAULT -> FAULT_RECOVERY");

	/* S5-U10: obstacle profile, a 100 mm box 250 mm ahead, room on both sides */
	{
		/* far left .. far right (45 / 22.5 deg); the outer beams see far walls */
		INT d[SCAN_ANGLES] = { 1200, 900, 250, 700, 800 };

		TEST_CHECK("S5-U10a", scan_evaluate(d, &r) && r.width_mm < 300 && r.width_mm >= 0,
			   "narrow obstacle is profiled");
		TEST_CHECK("S5-U10b", r.clear_left_mm > r.clear_right_mm ? r.side == OBS_SIDE_LEFT
								       : r.side == OBS_SIDE_RIGHT,
			   "the side with more clearance is chosen");
		/* wall of the same distance across the whole sweep: nowhere to go */
		d[0] = 354; d[1] = 271; d[2] = 250; d[3] = 271; d[4] = 354;
		TEST_CHECK("S5-U10c", !scan_evaluate(d, &r), "obstacle wider than the sweep -> fault");
		d[2] = 0;
		TEST_CHECK("S5-U10d", !scan_evaluate(d, &r), "no centre echo -> fault");
	}

	/* INT-03 (IF-07): an E_STOP from the dashboard reaches Mission.
	   Run last: it leaves the system in E_STOP. */
	/* INIT waits up to MS_INIT_NET for WiFi/MQTT, so allow for that */
	for (i = 0; i < (MS_INIT_NET / 10) + 200 && s5_mission_state != SYS_IDLE; i++) {
		tk_dly_tsk(10);			/* wait for INIT -> IDLE */
	}
	TEST_CHECK("S5-U11", s5_mission_state == SYS_IDLE, "Mission starts in IDLE after INIT");
#if S5_TEST_ESTOP
	q.cmd = MQTTCMD_ESTOP;
	MBF_SEND(mbf_mqttcmd, q);
	tk_dly_tsk(50);
	TEST_CHECK("INT-03", s5_mission_state == SYS_E_STOP, "MQTT E_STOP on mbf_mqttcmd -> Mission E_STOP");
#endif

	TEST_SUMMARY("mission_obstacle");
}

#endif /* APP_TEST */
