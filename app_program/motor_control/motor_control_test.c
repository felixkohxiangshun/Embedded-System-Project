/*
 *	motor_control_test.c
 *	On-target tests for the Motor control subsystem (Motor).  Built only with: make TEST=1
 *	Results go in testing/motor_control/TEST_LOG.md.
 *
 *	Every test that moves the car is OFF by default, so a plain TEST=1
 *	build stays still.  Switch one block on at a time:
 *	  S1_TEST_MOVE      S1-M01         wheels lifted
 *	  S1_TEST_FLOOR     S1-M03..M05, M10  car on the floor
 *	  S1_TEST_CMDS      S1-M11..M18    wheels lifted
 *	  S1_TEST_BYPASS    S1-M19         car on the floor
 *	  S1_TEST_DIST      S1-M21         car on the floor beside a ruler
 *	  S1_TEST_TURN_BTN  button turns   car on the floor, never returns
 *	  S1_TEST_FWD_BTN   GP21 forward   car on the floor, never returns
 *	                    (switch and speed are in motor_ctrl.h)
 */

#include "motor_control.h"
#include "../test_report.h"

/* Motion test blocks: switch on one at a time (see TEST_LOG.md) */
#ifndef S1_TEST_CMDS
#define S1_TEST_CMDS		0	/* S1-M11..S1-M18, wheels lifted */
#endif
#ifndef S1_TEST_BYPASS
#define S1_TEST_BYPASS		0	/* S1-M19, car on the floor */
#endif
#ifndef S1_TEST_DIST
#define S1_TEST_DIST		0	/* S1-M21, car on the floor beside a ruler */
#endif
#ifndef S1_TEST_TURN_BTN
#define S1_TEST_TURN_BTN	0	/* never returns, so no SUMMARY line */
#endif

#if APP_TEST

#if S1_TEST_CMDS || S1_TEST_BYPASS || S1_TEST_DIST || S1_TEST_FWD_BTN
IMPORT ID tid_mission;

#define S1_BYPASS_WIDTH_MM	100	/* obstacle width sent with NAV_BYPASS */

/* Pause Mission so only the test talks to Motor.  Never pause it while it
   holds or waits for the app_state lock. */
LOCAL void s1_pause_mission(void)
{
	T_RMTX rmtx;
	INT    k;

	for (k = 0; k < 100; k++) {
		(void)tk_sus_tsk(tid_mission);
		if ((tk_ref_mtx(mtx_state, &rmtx) == E_OK) &&
		    (rmtx.htsk != tid_mission) && (rmtx.wtsk != tid_mission)) {
			return;
		}
		(void)tk_rsm_tsk(tid_mission);
		tk_dly_tsk(1);
	}
}

LOCAL void s1_nav(UB type, INT arg, INT arg2)
{
	MSG_NAV m = {0};

	m.type = type;
	m.arg = arg;
	m.arg2 = arg2;
	(void)MBF_SEND(mbf_nav, m);
}

#if S1_TEST_DIST
#define S1_DIST_GOAL_MM		450	/* 50 cm ruler, room left to coast; use 1000 with a tape measure */
#define S1_DIST_RUNS		3

/* The distance Motor publishes for Telemetry (IF-11, FR-12) */
LOCAL INT s1_dist_mm(void)
{
	INT d = 0;

	if (state_lock(100) == E_OK) {
		d = (INT)app_state.dist_mm;
		state_unlock();
	}
	return d;
}
#endif /* S1_TEST_DIST */

#if S1_TEST_CMDS || S1_TEST_BYPASS
/* The command-done counter Motor publishes for Mission (IF-11) */
LOCAL UW s1_done_seq(void)
{
	UW seq = 0U;

	if (state_lock(100) == E_OK) {
		seq = app_state.cmd_done_seq;
		state_unlock();
	}
	return seq;
}

/* Wait up to max_ms while Motor stays in state 'st'; returns the ms waited */
LOCAL INT s1_wait_leave(UB st, INT max_ms)
{
	INT t;

	for (t = 0; (t < max_ms) && (s1_state == st); t += 10) {
		tk_dly_tsk(10);
	}
	return t;
}

/* Run one bypass on 'side' and check it completes.  Returns TRUE when it
   started turning the right way, ended STOPPED and moved the done counter
   by exactly one. */
LOCAL BOOL s1_bypass(INT side)
{
	UW  seq0 = s1_done_seq();
	INT bl = enc_cnt[0], br = enc_cnt[1];
	INT dl, dr, t;
	BOOL ok;

	s1_nav(NAV_BYPASS, side, S1_BYPASS_WIDTH_MM);
	tk_dly_tsk(300);
	dl = enc_cnt[0] - bl;
	dr = enc_cnt[1] - br;
	ok = (s1_state == MOT_BYPASS) &&
	     ((side > 0) ? (dl > 0 && dr < 0) : (dl < 0 && dr > 0));
	t = s1_wait_leave(MOT_BYPASS, 30000);
	tk_dly_tsk(50);
	tm_printf((UB*)"[CMDS] bypass side=%d: first 300 ms enc_l=%d enc_r=%d, took %d ms, state=%d done_seq +%d\n",
		  side, dl, dr, t + 300, (INT)s1_state, (INT)(s1_done_seq() - seq0));
	return ok && (s1_state == MOT_STOPPED) && (s1_done_seq() == seq0 + 1U);
}
#endif /* S1_TEST_CMDS || S1_TEST_BYPASS */
#endif /* S1_TEST_CMDS || S1_TEST_BYPASS || S1_TEST_DIST || S1_TEST_FWD_BTN */

#if S1_TEST_CMDS
/* Drive 2 s at 'steer', sending NAV_STEER every 50 ms so the Mission
   watchdog stays fed.  The counts are those of the last second only, after
   the speed has settled. */
LOCAL void s1_drive(INT steer, INT *dl, INT *dr)
{
	INT t, bl = 0, br = 0;

	for (t = 0; t < 2000; t += 50) {
		if (t == 1000) {
			bl = enc_cnt[0];
			br = enc_cnt[1];
		}
		s1_nav(NAV_STEER, steer, 0);
		tk_dly_tsk(50);
	}
	*dl = enc_cnt[0] - bl;
	*dr = enc_cnt[1] - br;
}

/* Counts in one second -> mm/s */
LOCAL INT s1_mmps(INT counts)
{
	return (counts * WHEEL_MM_PER_REV) / ENC_COUNTS_PER_REV;
}

/* |a - b| within pct percent of a */
LOCAL BOOL s1_near(INT a, INT b, INT pct)
{
	INT d = a - b;

	return ((d < 0 ? -d : d) * 100 <= (a < 0 ? -a : a) * pct) ? TRUE : FALSE;
}
#endif /* S1_TEST_CMDS */

EXPORT void motor_control_test(void)
{
	UW  t0;
	INT i, n;

	TEST_INIT();

	/* USB needs a moment to reconnect after a reset: give the serial
	   monitor time to open before the first result is printed. */
	for (i = 5; i > 0; i--) {
		tm_printf((UB*)"[TEST] motor_control tests start in %d s\n", i);
		tk_dly_tsk(1000);
	}

	/* S1-U01: quadrature decoder, forward sequence 00 01 11 10 00 */
	n = qdec_step(0, 1) + qdec_step(1, 3) + qdec_step(3, 2) + qdec_step(2, 0);
	TEST_CHECK("S1-U01", n == 4, "quadrature: 4 forward edges = +4");

	/* S1-U02: reverse sequence and an impossible jump */
	n = qdec_step(0, 2) + qdec_step(2, 3) + qdec_step(3, 1) + qdec_step(1, 0);
	TEST_CHECK("S1-U02", n == -4 && qdec_step(0, 3) == 0,
		   "quadrature: 4 reverse edges = -4, a double jump counts 0");

	/* S1-U03: unit conversions agree with the constants.  Integer
	   rounding (2654 / 10 = 265 counts per tick) allows 1 % on the speed. */
	n = counts_to_mmps(ENC_COUNTS_PER_REV / 10) - WHEEL_MM_PER_REV * 10;
	TEST_CHECK("S1-U03",
		   mm_to_counts(WHEEL_MM_PER_REV) == ENC_COUNTS_PER_REV &&
		   (n < 0 ? -n : n) * 100 <= WHEEL_MM_PER_REV * 10,
		   "one turn = ENC_COUNTS_PER_REV counts = WHEEL_MM_PER_REV mm; 10 turns/s = 10x mm/s (1%)");

	/* S1-U04: PID output follows the sign of the error and is clamped */
	{
		PID p;

		pid_init(&p, PID_KP, PID_KI, PID_KD, -DUTY_MAX, DUTY_MAX);
		TEST_CHECK("S1-U04", pid_step(&p, 100, 0) > 0, "PID: too slow -> push harder");
		pid_reset(&p);
		TEST_CHECK("S1-U05", pid_step(&p, 0, 100) < 0, "PID: too fast -> back off");
		pid_reset(&p);
		for (i = 0; i < 1000; i++) {
			n = pid_step(&p, 10000, 0);
		}
		TEST_CHECK("S1-U06", n == DUTY_MAX, "PID: output clamped to DUTY_MAX");
	}

	/* S1-U07: the Motor loop runs every 10 ms (NFR-14): ~100 in 1 s */
	t0 = s1_ticks;
	tk_dly_tsk(1000);
	n = (INT)(s1_ticks - t0);
	TEST_CHECK("S1-U07", n >= 95 && n <= 105, "motor loop runs every 10 ms");

	/* S1-U08: after boot the motors are STOPPED */
	TEST_CHECK("S1-U08", s1_state == MOT_STOPPED, "motor state is STOPPED after boot");

	/* S1-U09: the GPIO interrupt is installed: pins idle, count is stable */
	n = enc_cnt[0];
	tk_dly_tsk(100);
	TEST_CHECK("S1-U09", enc_cnt[0] == n, "encoder count is steady with the wheels still");

#if defined(S1_TEST_MOVE) && S1_TEST_MOVE
	/* S1-M01: START -> DRIVING -> STOP -> STOPPED */
	tk_set_flg(flg_evt, EVT_START);
	tk_dly_tsk(50);
	TEST_CHECK("S1-M01a", s1_state == MOT_DRIVING, "EVT_START -> DRIVING");
	tk_set_flg(flg_evt, EVT_STOP);
	tk_dly_tsk(50);
	TEST_CHECK("S1-M01b", s1_state == MOT_STOPPED, "EVT_STOP -> STOPPED");
#endif
	#if defined(S1_TEST_FLOOR) && S1_TEST_FLOOR
	{
		static const INT turn_deg[3] = { 90, -90, 180 };
		MSG_NAV nav_msg = {0};
		INT     k, w;
 
		/* S1-M03 / S1-M04: straight run */
		tm_printf((UB*)"[FLOOR] straight run starts in 5 s - place the car\n");
		tk_dly_tsk(5000);
		tk_set_flg(flg_evt, EVT_START);
		for (k = 1; k <= 30; k++) {
			/* Mission is IDLE, so stand in for it: a NAV message every
			   100 ms keeps the Mission watchdog (NAV_STOP_TICKS) happy. */
			for (w = 0; w < 5; w++) {
				nav_msg.type = NAV_STEER;
				nav_msg.arg = 0;
				nav_msg.arg2 = 0;
				(void)MBF_SEND(mbf_nav, nav_msg);
				tk_dly_tsk(100);
			}
			if (state_lock(100) == E_OK) {
				tm_printf((UB*)"[FLOOR] t=%dms speed=%dmm/s dist=%dmm enc_l=%d enc_r=%d\n",
					  k * 500, app_state.speed_mmps,
					  (INT)app_state.dist_mm,
					  app_state.enc_left, app_state.enc_right);
				state_unlock();
			}
		}
		tk_set_flg(flg_evt, EVT_STOP);
		tk_dly_tsk(500);
		tm_printf((UB*)"[FLOOR] stopped - measure the distance now\n");
 
		/* S1-M05: turns */
		for (k = 0; k < 3; k++) {
			tm_printf((UB*)"[FLOOR] turn %d deg in 5 s - mark the start heading\n",
				  turn_deg[k]);
			tk_dly_tsk(5000);
			nav_msg.type = NAV_TURN;
			nav_msg.arg = turn_deg[k];
			nav_msg.arg2 = 0;
			(void)MBF_SEND(mbf_nav, nav_msg);
			tk_dly_tsk(100);
			for (w = 0; (w < 100) && (s1_state == MOT_TURNING); w++) {
				tk_dly_tsk(50);		/* wait up to 5 s for the turn */
			}
			TEST_CHECK("S1-M05", s1_state == MOT_STOPPED,
				   "turn finished and motors stopped (measure the angle)");
		}
		/* S1-M10: Mission watchdog and the fault bitmask (wheels in the air
		   is fine).  Drive with NO NAV messages: the Motor task must stop
		   by itself and set FAULT_MISSION_TMO, then clear it when a NAV
		   message arrives again. */
		tm_printf((UB*)"[FLOOR] watchdog test in 5 s - wheels will spin about 0.5 s\n");
		tk_dly_tsk(5000);
		tk_set_flg(flg_evt, EVT_START);
		tk_dly_tsk(1000);			/* > NAV_STOP_TICKS, no NAV sent */
		k = 0;
		if (state_lock(100) == E_OK) {
			k = (INT)app_state.fault_flags;
			state_unlock();
		}
		TEST_CHECK("S1-M10a", (k & FAULT_MISSION_TMO) != 0,
			   "no NAV for 1 s -> motors held, FAULT_MISSION_TMO set");
		nav_msg.type = NAV_STEER;
		nav_msg.arg = 0;
		nav_msg.arg2 = 0;
		(void)MBF_SEND(mbf_nav, nav_msg);
		tk_dly_tsk(50);
		k = 0xFF;
		if (state_lock(100) == E_OK) {
			k = (INT)app_state.fault_flags;
			state_unlock();
		}
		TEST_CHECK("S1-M10b", (k & FAULT_MISSION_TMO) == 0,
			   "NAV arrives again -> FAULT_MISSION_TMO cleared");
		tk_set_flg(flg_evt, EVT_STOP);
		tm_printf((UB*)"[FLOOR] done\n");
	}
#endif
#if S1_TEST_CMDS
	/* S1-M11..S1-M18: every command Mission can send, with the wheels
	   lifted.  Mission is paused; this test stands in for it. */
	{
		INT  l0, r0, l1, r1, dl, dr, bl, br, k;
		UW   seq0;

		s1_pause_mission();
		tm_printf((UB*)"[CMDS] lift the wheels off the ground - starts in 3 s\n");
		tk_dly_tsk(3000);
		tk_set_flg(flg_evt, EVT_STOP);
		tk_dly_tsk(50);
		tk_set_flg(flg_evt, EVT_START);
		tk_dly_tsk(50);

		/* S1-M11: steer sign.  + = right, so the left wheel runs faster */
		s1_drive(0, &l0, &r0);
		s1_drive(+500, &l1, &r1);
		s1_drive(-500, &dl, &dr);
		tm_printf((UB*)"[CMDS] counts/s  steer 0: L=%d R=%d  +500: L=%d R=%d  -500: L=%d R=%d\n",
			  l0, r0, l1, r1, dl, dr);
		TEST_CHECK("S1-M11a", l1 * 10 > r1 * 12, "NAV_STEER +500 -> left wheel >= 20% faster (curves right)");
		TEST_CHECK("S1-M11b", dr * 10 > dl * 12, "NAV_STEER -500 -> right wheel >= 20% faster (curves left)");

		/* S1-M12: steer beyond STEER_MAX is clamped */
		s1_drive(+1000, &l0, &r0);
		s1_drive(+2000, &l1, &r1);
		tm_printf((UB*)"[CMDS] counts/s  steer +1000: L=%d R=%d  +2000: L=%d R=%d\n",
			  l0, r0, l1, r1);
		TEST_CHECK("S1-M12", s1_near(l0, l1, 10) && s1_near(r0, r1, 10),
			   "NAV_STEER +2000 drives the same as +1000 (clamped)");

		/* S1-M13: speed commands */
		s1_nav(NAV_SLOW, 0, 0);
		s1_drive(0, &dl, &dr);
		k = s1_mmps((dl + dr) / 2);
		tm_printf((UB*)"[CMDS] NAV_SLOW: %d mm/s (target %d)\n", k, SPEED_SLOW_MMPS);
		TEST_CHECK("S1-M13a", s1_near(SPEED_SLOW_MMPS, k, 15), "NAV_SLOW -> slow speed within 15%");
		s1_nav(NAV_RESUME, 0, 0);
		s1_drive(0, &dl, &dr);
		k = s1_mmps((dl + dr) / 2);
		tm_printf((UB*)"[CMDS] NAV_RESUME: %d mm/s (target %d)\n", k, SPEED_NORMAL_MMPS);
		TEST_CHECK("S1-M13b", s1_near(SPEED_NORMAL_MMPS, k, 15), "NAV_RESUME -> normal speed within 15%");

		/* S1-M14: turn while driving, the way Mission does a barcode turn */
		seq0 = s1_done_seq();
		s1_nav(NAV_TURN, 90, 0);
		tk_dly_tsk(50);
		TEST_CHECK("S1-M14a", s1_state == MOT_TURNING, "NAV_TURN while DRIVING -> TURNING");
		(void)s1_wait_leave(MOT_TURNING, 5000);
		tk_dly_tsk(50);
		TEST_CHECK("S1-M14b", s1_state == MOT_STOPPED && s1_done_seq() == seq0 + 1U,
			   "turn ends STOPPED and cmd_done_seq goes up by exactly 1");
		tk_set_flg(flg_evt, EVT_RESUME);
		tk_dly_tsk(50);
		TEST_CHECK("S1-M14c", s1_state == MOT_DRIVING, "EVT_RESUME after the turn -> DRIVING");
		tk_set_flg(flg_evt, EVT_STOP);
		tk_dly_tsk(500);

		/* S1-M15: a second command during a turn is ignored, not queued */
		seq0 = s1_done_seq();
		bl = enc_cnt[0];
		br = enc_cnt[1];
		s1_nav(NAV_TURN, 90, 0);
		tk_dly_tsk(200);
		s1_nav(NAV_TURN, -90, 0);
		s1_nav(NAV_BYPASS, OBS_SIDE_LEFT, S1_BYPASS_WIDTH_MM);
		(void)s1_wait_leave(MOT_TURNING, 5000);
		tk_dly_tsk(300);
		dl = enc_cnt[0] - bl;
		dr = enc_cnt[1] - br;
		tm_printf((UB*)"[CMDS] busy turn: enc_l=%d enc_r=%d state=%d done_seq +%d\n",
			  dl, dr, (INT)s1_state, (INT)(s1_done_seq() - seq0));
		TEST_CHECK("S1-M15", s1_state == MOT_STOPPED && s1_done_seq() == seq0 + 1U &&
			   dl > 0 && dr < 0,
			   "NAV_TURN / NAV_BYPASS during a turn ignored; first turn finishes");

		/* S1-M16: EVT_STOP aborts a turn and does not report it done */
		seq0 = s1_done_seq();
		s1_nav(NAV_TURN, 180, 0);
		tk_dly_tsk(300);
		k = (s1_state == MOT_TURNING) ? 1 : 0;
		tk_set_flg(flg_evt, EVT_STOP);
		tk_dly_tsk(20);
		TEST_CHECK("S1-M16a", k == 1 && s1_state == MOT_STOPPED, "EVT_STOP mid-turn -> STOPPED within 20 ms");
		tk_dly_tsk(300);			/* let the wheels coast down */
		bl = enc_cnt[0];
		br = enc_cnt[1];
		tk_dly_tsk(200);
		dl = enc_cnt[0] - bl;
		dr = enc_cnt[1] - br;
		TEST_CHECK("S1-M16b", (dl >= -2 && dl <= 2) && (dr >= -2 && dr <= 2) &&
			   s1_done_seq() == seq0,
			   "aborted turn: wheels still, cmd_done_seq unchanged");

		/* S1-M17: EVT_START during a turn is ignored */
		seq0 = s1_done_seq();
		s1_nav(NAV_TURN, -90, 0);
		tk_dly_tsk(200);
		tk_set_flg(flg_evt, EVT_START);
		tk_dly_tsk(50);
		k = (s1_state == MOT_TURNING) ? 1 : 0;
		(void)s1_wait_leave(MOT_TURNING, 5000);
		tk_dly_tsk(50);
		TEST_CHECK("S1-M17", k == 1 && s1_state == MOT_STOPPED && s1_done_seq() == seq0 + 1U,
			   "EVT_START during a turn ignored; turn finishes");

		/* S1-M18: the bypass path runs all its segments, both sides */
		TEST_CHECK("S1-M18a", s1_bypass(OBS_SIDE_RIGHT), "NAV_BYPASS right: turns right first, ends STOPPED, done +1");
		tk_dly_tsk(500);
		TEST_CHECK("S1-M18b", s1_bypass(OBS_SIDE_LEFT), "NAV_BYPASS left: turns left first, ends STOPPED, done +1");

		tk_set_flg(flg_evt, EVT_STOP);
		tm_printf((UB*)"[CMDS] done\n");
	}
#endif
#if S1_TEST_BYPASS
	/* S1-M19: bypass path on the floor.  The car should end on its start
	   line, BYPASS_LEG_MM + width ahead, facing the same way. */
	{
		static const INT side[2] = { OBS_SIDE_RIGHT, OBS_SIDE_LEFT };
		INT k;

		s1_pause_mission();
		tk_set_flg(flg_evt, EVT_STOP);
		for (k = 0; k < 2; k++) {
			tm_printf((UB*)"[BYPASS] side %d in 5 s - mark the start position and heading\n",
				  side[k]);
			tk_dly_tsk(5000);
			TEST_CHECK("S1-M19", s1_bypass(side[k]), "floor bypass completes (measure the end position)");
			/* BYPASS_LEG_MM (task_motor.c) + the width */
			tm_printf((UB*)"[BYPASS] expected: %d mm ahead, 0 mm sideways, 0 deg - measure now\n",
				  150 + S1_BYPASS_WIDTH_MM);
			tk_dly_tsk(10000);		/* time to measure */
		}
	}
#endif
#if S1_TEST_DIST
	/* S1-M21: encoder distance against a ruler (FR-12).  The car drives
	   straight until dist_mm has grown by S1_DIST_GOAL_MM, stops, and
	   prints how far it thinks it went, coasting included.  Compare that
	   with the ruler: the two must agree, not the goal and the ruler. */
	{
		INT k, t, d, bl, br, dl, dr;

		s1_pause_mission();
		tk_set_flg(flg_evt, EVT_STOP);
		for (k = 1; k <= S1_DIST_RUNS; k++) {
			tm_printf((UB*)"[DIST] run %d of %d: put the front axle on 0 of the ruler - starts in 10 s\n",
				  k, S1_DIST_RUNS);
			tk_dly_tsk(10000);
			d = s1_dist_mm();
			bl = enc_cnt[0];
			br = enc_cnt[1];
			tk_set_flg(flg_evt, EVT_START);
			for (t = 0; (t < 10000) && (s1_dist_mm() - d < S1_DIST_GOAL_MM); t += 20) {
				if ((t % 100) == 0) {
					s1_nav(NAV_STEER, 0, 0);	/* keep the Mission watchdog fed */
				}
				tk_dly_tsk(20);
			}
			tk_set_flg(flg_evt, EVT_STOP);
			tk_dly_tsk(1000);			/* let it coast to a stop */
			d = s1_dist_mm() - d;
			dl = enc_cnt[0] - bl;
			dr = enc_cnt[1] - br;
			tm_printf((UB*)"[DIST] run %d: dist_mm=%d  enc_l=%d enc_r=%d - read the ruler at the front axle now\n",
				  k, d, dl, dr);
			TEST_CHECK("S1-M21", dl > 0 && dr > 0 &&
				   d >= S1_DIST_GOAL_MM && d <= (S1_DIST_GOAL_MM * 115) / 100,
				   "drove forward on both wheels and stopped near the goal (compare dist_mm with the ruler)");
		}
		tm_printf((UB*)"[DIST] if the ruler differs: WHEEL_MM_PER_REV = %d x ruler_mm / dist_mm\n",
			  WHEEL_MM_PER_REV);
	}
#endif
#if S1_TEST_TURN_BTN
	{
		extern ID tid_mission;
		T_RMTX    rmtx;
		MSG_NAV   nav_msg = {0};
		UW        in, prev = 0U;
		INT       bl, br, dl, dr, w, k, f, deg;
 
		/* Pause Mission so the buttons only drive this test.  Never pause
		   it while it holds or waits for the app_state lock. */
		for (k = 0; k < 100; k++) {
			(void)tk_sus_tsk(tid_mission);
			if ((tk_ref_mtx(mtx_state, &rmtx) == E_OK) &&
			    (rmtx.htsk != tid_mission) && (rmtx.wtsk != tid_mission)) {
				break;
			}
			(void)tk_rsm_tsk(tid_mission);
			tk_dly_tsk(1);
		}
		gpio_set_pin(PIN_BTN_2, GPIO_MODE_IN);
		set_w(GPIO(PIN_BTN_2), GPIO_PUE);	/* active low, like GP20 */
		tk_set_flg(flg_evt, EVT_STOP);
		tk_dly_tsk(100);
		tm_printf((UB*)"[TURN] ready: GP20 = right 90, GP21 = left 90\n");
 
		while (1) {
			in = in_w(GPIO_IN);
			/* pressed now (low) and not pressed last time */
			deg = 0;
			if (((in & (1U << PIN_BTN_START)) == 0U) && ((prev & (1U << PIN_BTN_START)) != 0U)) {
				deg = 90;
			} else if (((in & (1U << PIN_BTN_2)) == 0U) && ((prev & (1U << PIN_BTN_2)) != 0U)) {
				deg = -90;
			}
			prev = in;
			if (deg == 0) {
				tk_dly_tsk(20);
				continue;
			}
 
			tk_dly_tsk(300);		/* let go of the car */
			bl = enc_cnt[0];
			br = enc_cnt[1];
			nav_msg.type = NAV_TURN;
			nav_msg.arg = deg;
			nav_msg.arg2 = 0;
			(void)MBF_SEND(mbf_nav, nav_msg);
			tk_dly_tsk(50);
			for (w = 0; (w < 100) && (s1_state == MOT_TURNING); w++) {
				tk_dly_tsk(50);		/* up to 5 s */
			}
			tk_dly_tsk(300);		/* include any coasting */
			dl = enc_cnt[0] - bl;
			dr = enc_cnt[1] - br;
			f = 0;
			if (state_lock(100) == E_OK) {
				f = (INT)app_state.fault_flags;
				state_unlock();
			}
			k = ((dl < 0 ? -dl : dl) + (dr < 0 ? -dr : dr)) / 2;
			tm_printf((UB*)"[TURN] cmd=%d deg  enc_l=%d enc_r=%d (goal %d each)  code angle=%d deg  state=%d faults=%d\n",
				  deg, dl, dr, deg_to_counts(90), counts_to_deg(k),
				  (INT)s1_state, f);
			if (s1_state == MOT_ENCODER_FAULT) {
				tk_set_flg(flg_evt, EVT_STOP);	/* clear it for the next try */
			}
			prev = in_w(GPIO_IN);
		}
	}
#endif
#if S1_TEST_FWD_BTN
	/* GP21 starts and stops a straight run at SPEED_NORMAL_MMPS.  No line
	   tracking: Mission is paused and the steering stays 0. */
	{
		UW   in, prev;
		BOOL driving = FALSE;
		INT  t = 0;

		s1_pause_mission();
		gpio_set_pin(PIN_BTN_2, GPIO_MODE_IN);
		set_w(GPIO(PIN_BTN_2), GPIO_PUE);	/* active low, like GP20 */
		tk_set_flg(flg_evt, EVT_STOP);
		tk_dly_tsk(100);
		prev = in_w(GPIO_IN);
		tm_printf((UB*)"[FWD] ready: GP21 = start/stop at %d mm/s\n", SPEED_NORMAL_MMPS);

		while (1) {
			in = in_w(GPIO_IN);
			/* pressed now (low) and not pressed last time */
			if (((in & (1U << PIN_BTN_2)) == 0U) && ((prev & (1U << PIN_BTN_2)) != 0U)) {
				driving = driving ? FALSE : TRUE;
				tk_set_flg(flg_evt, driving ? EVT_START : EVT_STOP);
				tm_printf((UB*)"[FWD] %s\n", driving ? "start" : "stop");
				tk_dly_tsk(200);		/* ride out the switch bounce */
				in = in_w(GPIO_IN);
			}
			prev = in;
			if (driving && (s1_state == MOT_ENCODER_FAULT)) {
				driving = FALSE;
				tk_set_flg(flg_evt, EVT_STOP);	/* clear it for the next try */
				tm_printf((UB*)"[FWD] stopped: wheel stall or encoder fault\n");
			}
			if (driving && ((t % 100) == 0)) {
				s1_nav(NAV_STEER, 0, 0);	/* keep the Mission watchdog fed */
			}
			t += 20;
			tk_dly_tsk(20);
		}
	}
#endif

	TEST_SUMMARY("motor_control");
}

#endif /* APP_TEST */
