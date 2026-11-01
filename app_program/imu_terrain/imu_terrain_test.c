/*
 *	imu_terrain_test.c
 *	On-target tests for the IMU/terrain subsystem (IMU / terrain).
 *	Built only with: make TEST=1.  Results: testing/imu_terrain/TEST_LOG.md
 *	S3-U01..U11 are the same checks as testing/host (same IDs).
 *
 *	The test that moves the car is OFF by default, so a plain TEST=1 build
 *	stays still.  Switch it on by changing the 0 to 1 below:
 *	  S3_TEST_HUMP      S3-M02         car on the floor, a hump ahead of it
 *	  S3_TEST_LOG       S3-M07         same drive, recording every raw sample;
 *	                                   press START again to print it (CSV)
 *	It needs no cable while driving: each run starts with the START button
 *	(GP20), the results are kept, and after the last run a summary is
 *	printed every 3 s until RST, so plug the USB in afterwards to read it.
 */

#include "imu_terrain.h"
#include "imu_test_hump.h"
#include "../test_report.h"

/* Motion test: switch on only when the car is on the floor (see TEST_LOG.md) */
#ifndef S3_TEST_HUMP
#define S3_TEST_HUMP		0	/* S3-M02: drive straight over a hump, 5 runs */
#endif
#ifndef S3_TEST_LOG
#define S3_TEST_LOG		0	/* S3-M07: one recorded run, then print it on each START press */
#endif

#if APP_TEST

LOCAL IMU_LOGIC l;			/* ~1 KB: not on the test task's stack */

#if S3_TEST_HUMP || S3_TEST_LOG
#define S3_HUMP_RUN_MM		700	/* distance per run: start ~25 cm before the hump [set] */
#define S3_HUMP_RUNS		5
#define S3_HUMP_SLOW		0	/* 1 = whole run at 60 mm/s */
#define S3_HUMP_LIKE_MISSION	1	/* 1 = slow down when a hump is detected, speed up
					   after it, as Mission does on the real course */
#define S3_HUMP_HANDS_OFF_MS	2000	/* after the button: time to take your hand away */
#define S3_HUMP_REPLAY_MS	3000	/* summary repeated this often after the last run */

LOCAL INT s3_res_n[S3_HUMP_RUNS];	/* humps seen in each run */
LOCAL INT s3_res_mm[S3_HUMP_RUNS];	/* height of that hump */
LOCAL INT s3_res_resid[S3_HUMP_RUNS];	/* residual, um */
LOCAL INT s3_res_dist[S3_HUMP_RUNS];	/* distance driven, mm */
LOCAL INT s3_res_peak[S3_HUMP_RUNS];	/* highest lean height seen, um */
LOCAL INT s3_res_dev[S3_HUMP_RUNS][3];	/* largest change of fwd, side, up, mg */

IMPORT ID tid_mission;

/* Pause Mission so only this test talks to Motor (no line needed, and
   Mission's line-lost timeout cannot stop the car).  Same method as Dave's
   S1-M21: never pause it while it holds or waits for the app_state lock. */
LOCAL void s3_pause_mission(void)
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

LOCAL void s3_nav(UB type, INT arg)
{
	MSG_NAV m = {0};

	m.type = type;
	m.arg = arg;
	(void)MBF_SEND(mbf_nav, m);
}

/* Distance Motor publishes (IF-11) */
LOCAL INT s3_dist_mm(void)
{
	INT d = 0;

	if (state_lock(100) == E_OK) {
		d = (INT)app_state.dist_mm;
		state_unlock();
	}
	return d;
}

/* One result line.  "lean" = the highest wheel height the roll showed
   (even if no hump was counted); "change" = how far each car axis moved
   from rest: on a one-wheel hump "side" should change most. */
LOCAL void s3_print_run(INT i)
{
	tm_printf((UB*)"[HUMP] run %d: %d hump(s), height %d mm, residual %d um, lean %d.%d mm,"
		  " change fwd %d side %d up %d mg, drove %d mm%s\n",
		  i + 1, s3_res_n[i], s3_res_mm[i], s3_res_resid[i],
		  s3_res_peak[i] / 1000, (s3_res_peak[i] % 1000) / 100,
		  s3_res_dev[i][0], s3_res_dev[i][1], s3_res_dev[i][2], s3_res_dist[i],
		  s3_calibrated ? "" : "  (NOT CALIBRATED: keep the car still after power-on)");
}

/* Ask imu_task to do something with its logic state, and wait for it */
LOCAL void s3_cmd(UB cmd)
{
	INT k;

	s3_run_cmd = cmd;
	for (k = 0; (k < 50) && (s3_run_cmd != S3_CMD_NONE); k++) {
		tk_dly_tsk(10);
	}
}

/* S3-M02, one run: drive straight (steer 0, Motor's drift trim keeps it
   straight) for S3_HUMP_RUN_MM, stop, close any hump still under way, and
   print what the IMU measured.  Compare each height with the ruler and log
   it in TEST_LOG.md.  Returns TRUE if the car drove the whole distance. */
LOCAL BOOL s3_hump_run(INT k)
{
	INT  t, d0;
	UW   h0;
	BOOL slowed;

	UINT ptn;

	tm_printf((UB*)"[HUMP] IMU %s, rest fwd %d side %d up %d mg\n",
		  s3_calibrated ? "calibrated" : "NOT calibrated yet (keep still)",
		  s3_rest[0], s3_rest[1], s3_rest[2]);
	tm_printf((UB*)"[HUMP] run %d of %d: put the car ~25 cm before the hump, pointing straight at it,"
		  " then press START (GP20)\n", k, S3_HUMP_RUNS);
	(void)tk_clr_flg(flg_evt, (UINT)~EVT_BTN);	/* forget earlier presses */
	(void)tk_wai_flg(flg_evt, EVT_BTN, TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
	tk_dly_tsk(S3_HUMP_HANDS_OFF_MS);
#if S3_TEST_LOG
	s3_log_n = 0;
	s3_log_on = TRUE;		/* record from the start of the drive */
#endif
	s3_cmd(S3_CMD_RESET);
	h0 = s3_humps;
	d0 = s3_dist_mm();
	tk_set_flg(flg_evt, EVT_START);
	if (S3_HUMP_SLOW) {
		s3_nav(NAV_SLOW, 0);
	}
	slowed = FALSE;
	for (t = 0; (t < 20000) && ((s3_dist_mm() - d0) < S3_HUMP_RUN_MM); t += 20) {
		if ((t % 100) == 0) {
			s3_nav(NAV_STEER, 0);		/* keep Motor's watchdog fed */
		}
		if (S3_HUMP_LIKE_MISSION && !S3_HUMP_SLOW) {
			if (!slowed && (s3_state == IMU_HUMP_TRACKING)) {
				s3_nav(NAV_SLOW, 0);	/* Mission: TERR_HUMP_START */
				slowed = TRUE;
			} else if (slowed && (s3_humps != h0)) {
				s3_nav(NAV_RESUME, 0);	/* Mission: TERR_HUMP_DONE */
				slowed = FALSE;
			}
		}
		tk_dly_tsk(20);
	}
	tk_set_flg(flg_evt, EVT_STOP);
	tk_dly_tsk(1000);				/* coast to a stop */
	s3_cmd(S3_CMD_FINISH);				/* a hump just before the stop counts */
	tk_dly_tsk(50);
	s3_res_n[k - 1] = (INT)(s3_humps - h0);
	s3_res_mm[k - 1] = (s3_res_n[k - 1] > 0) ? s3_last_mm : 0;
	s3_res_resid[k - 1] = s3_residual_um;
	s3_res_dist[k - 1] = s3_dist_mm() - d0;
	s3_res_peak[k - 1] = s3_run_peak_um;
	s3_res_dev[k - 1][0] = s3_run_dev[0];
	s3_res_dev[k - 1][1] = s3_run_dev[1];
	s3_res_dev[k - 1][2] = s3_run_dev[2];
	s3_print_run(k - 1);
	return (s3_res_dist[k - 1] >= S3_HUMP_RUN_MM) ? TRUE : FALSE;
}

#if S3_TEST_HUMP
/* After the runs: repeat the results until RST, for reading over USB later */
LOCAL void s3_hump_replay(void)
{
	INT k;

	while (1) {
		tm_printf((UB*)"\n[HUMP] ===== results (tallest %d mm) - press RST to stop =====\n", s3_max_mm);
		for (k = 0; k < S3_HUMP_RUNS; k++) {
			s3_print_run(k);
		}
		tk_dly_tsk(S3_HUMP_REPLAY_MS);
	}
}
#endif
#if S3_TEST_LOG
LOCAL void s3_wait_button(void)
{
	UINT ptn;

	(void)tk_clr_flg(flg_evt, (UINT)~EVT_BTN);
	(void)tk_wai_flg(flg_evt, EVT_BTN, TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
}

/* Print the recording as CSV each time START is pressed.  Turn on PuTTY's
   logging (Session > Logging > All session output) before pressing. */
LOCAL void s3_log_dump_loop(void)
{
	INT i;

	while (1) {
		tm_printf((UB*)"[LOG] recorded %d samples. Plug in USB, open PuTTY with logging on,"
			  " then press START to print them\n", s3_log_n);
		s3_wait_button();
		tm_printf((UB*)"LOG_BEGIN axis_fwd=%d axis_up=%d track_mm=%d run_mm=%d calibrated=%d rest=%d,%d,%d\n",
			  IMU_AXIS_FWD, IMU_AXIS_UP, CAR_TRACK_MM, S3_HUMP_RUN_MM,
			  (INT)s3_calibrated, s3_rest[0], s3_rest[1], s3_rest[2]);
		tm_printf((UB*)"i,x_mg,y_mg,z_mg,ds_um,roll_h_0p1mm,motor,imu\n");
		for (i = 0; i < s3_log_n; i++) {
			tm_printf((UB*)"%d,%d,%d,%d,%d,%d,%d,%d\n", i,
				  s3_log_raw[i][0], s3_log_raw[i][1], s3_log_raw[i][2],
				  s3_log_ds[i], s3_log_h[i], s3_log_st[i] >> 4, s3_log_st[i] & 0x0F);
			if ((i % 10) == 9) {
				tk_dly_tsk(20);		/* let the 4 KB USB buffer drain */
			}
		}
		tm_printf((UB*)"LOG_END\n");
	}
}
#endif

#endif /* S3_TEST_HUMP || S3_TEST_LOG */

EXPORT void imu_terrain_test(void)
{
	TH_HUMP flat = { 0, 0, 1, 0, 0 };
	TH_HUMP h20 = { 400, 20, 60, 50, 1 };	/* 20 mm under the right wheel */
	TH_HUMP h10 = { 400, 10, 40, 30, 1 };
	TH_HUMP h70 = { 400, 70, 200, 40, 1 };	/* above HUMP_MAX_MM */
	TH_HUMP w20 = { 400, 20, 60, 50, 0 };	/* 20 mm across the whole track */
	IMU_VEC rest = { 0, 0, 1000 };
	TH_OUT  o;
	INT     rem = 0, k, sum = 0, first, n, ev;
	UW      t0;

	TEST_INIT();

	/* S3-U01..U03: calibration, flat road, two humps, tallest kept (FR-04/05) */
	imu_logic_init(&l);
	th_calibrate(&l);
	o = th_drive(&l, &flat, 0, 1500, -1, 0, 0, -1);
	TEST_CHECK("S3-U01", l.state == IMU_MONITORING && l.calibrated &&
		   th_abs(l.rest.fwd - 35) <= 10 && o.starts == 0 && o.dones == 0,
		   "calibrates (2 deg mount tilt), flat road is quiet");
	o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, -1);
	first = o.height_mm;
	TEST_CHECK("S3-U02", o.starts == 1 && o.dones == 1 && th_abs(first - 20) <= 2,
		   "20 mm hump under one wheel: start + done, height within 2 mm");
	o = th_drive(&l, &h10, 0, 1000, -1, 0, 0, -1);
	TEST_CHECK("S3-U03", o.dones == 1 && th_abs(o.height_mm - 10) <= 2 &&
		   l.max_height_mm == first, "10 mm hump measured, tallest (20) kept");

	/* S3-U04/U05/U08: shock (FR-06) */
	imu_logic_init(&l);
	th_calibrate(&l);
	o = th_drive(&l, &flat, 0, 400, 200, 3500, 2, -1);
	TEST_CHECK("S3-U04", o.shocks == 1, "2-sample 3.5 g knock = SHOCK");
	TEST_CHECK("S3-U08", o.starts == 0 && o.dones == 0, "a knock does not look like a hump");
	o = th_drive(&l, &flat, 0, 400, 200, 3500, 5, -1);
	TEST_CHECK("S3-U05", o.shocks == 0, "5-sample reading is not a SHOCK");

	/* S3-U06: fault and recovery (FR-19) */
	imu_fault(&l);
	TEST_CHECK("S3-U06a", l.state == IMU_SENSOR_FAULT &&
		   imu_feed(&l, &rest, 1200, FALSE, TRUE) == IMU_EV_NONE, "fault ignores samples");
	imu_recovered(&l);
	TEST_CHECK("S3-U06b", l.state == IMU_MONITORING && l.calibrated, "recovery keeps the baseline");
	imu_logic_init(&l);
	imu_fault(&l);
	imu_recovered(&l);
	TEST_CHECK("S3-U06c", l.state == IMU_CALIBRATING, "recovery before calibration -> CALIBRATING");

	/* S3-U09/U10: out of range, turn during a hump */
	imu_logic_init(&l);
	th_calibrate(&l);
	o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, -1);
	first = l.max_height_mm;
	o = th_drive(&l, &h70, 0, 1200, -1, 0, 0, -1);
	TEST_CHECK("S3-U09", o.dones == 1 && o.height_mm > HUMP_MAX_MM && l.max_height_mm == first,
		   "hump above 63 mm reported, not kept as tallest");
	o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, 450);
	TEST_CHECK("S3-U10", o.dones == 0 && l.state == IMU_MONITORING, "turn during a hump: given up");

	/* S3-U11: distance conversion */
	for (k = 0; k < 1000; k++) {
		sum += imu_counts_to_um(&rem, 30, WHEEL_MM_PER_REV, 2 * ENC_COUNTS_PER_REV);
	}
	TEST_CHECK("S3-U11", th_abs(sum - 1220799) <= 1, "encoder counts -> um, no rounding drift");

	/* S3-U12: the full-width method still works (if the course changes) */
	imu_logic_init(&l);
	imu_set_mode(&l, HUMP_MODE_PITCH);
	th_calibrate(&l);
	o = th_drive(&l, &w20, 0, 1000, -1, 0, 0, -1);
	TEST_CHECK("S3-U12", o.dones == 1 && th_abs(o.height_mm - 20) <= 2,
		   "pitch mode: 20 mm hump across the track within 2 mm");

	/* S3-U13: the car stops on the hump (end of a test run) */
	imu_logic_init(&l);
	th_calibrate(&l);
	o = th_drive(&l, &h20, 0, 508, -1, 0, 0, -1);
	TEST_CHECK("S3-U13", o.dones == 0 && imu_finish(&l) == IMU_EV_HUMP_DONE &&
		   th_abs(l.height_mm - 20) <= 2 && l.run_peak_um >= 18000,
		   "car stops on the hump: finish gives the height");

	/* S3-U14: the bug seen on the car (2026-11-01): never calibrated, so the
	   board's mounting tilt looked like a lifted wheel and a "hump" was
	   reported at power-up and never ended.  Now nothing is measured until
	   the car has stood still for the calibration. */
	imu_logic_init(&l);
	{
		IMU_VEC tilted = { -30, 40, 884 };	/* tilted board, as read on the car */

		for (k = 0; k < 300; k++) {
			ev = imu_feed(&l, &tilted, (k % 3 == 0) ? -10 : 0, FALSE, TRUE);
			if (ev != IMU_EV_NONE) {
				break;
			}
		}
		TEST_CHECK("S3-U14a", k == 300 && l.state == IMU_CALIBRATING && !l.calibrated,
		        "not calibrated: no hump from a tilted board, stays CALIBRATING");
		for (k = 0; k < IMU_CAL_SAMPLES + 5; k++) {
			ev = imu_feed(&l, &tilted, 0, TRUE, TRUE);
		}
		TEST_CHECK("S3-U14b", l.calibrated && l.state == IMU_MONITORING && ev == IMU_EV_NONE &&
		        imu_height_now_um(&l) == 0, "calibrates at rest even with |a| = 885 mg; then quiet");
	}

	/* S3-U07: the IMU task runs every 10 ms (NFR-06) even if the sensor is
	   missing, because a failed read must not stop the task */
	t0 = s3_ticks;
	tk_dly_tsk(1000);
	n = (INT)(s3_ticks - t0);
	TEST_CHECK("S3-U07", n >= 95 && n <= 105, "IMU task runs every 10 ms");

	/* S3-H01 (hardware): the sensor answers on I2C0 */
	TEST_CHECK("S3-H01", s3_state != IMU_SENSOR_FAULT, "LSM303DLHC answers on I2C (needs the sensor wired)");

	tm_printf((UB*)"[IMU] raw X %d Y %d Z %d mg, pitch %d deg\n",
		  s3_raw_mg[0], s3_raw_mg[1], s3_raw_mg[2], s3_pitch_deg);
#if S3_TEST_LOG
	s3_pause_mission();
	tk_set_flg(flg_evt, EVT_STOP);
	s3_log_n = 0;
	(void)s3_hump_run(1);			/* starts recording after START */
	s3_log_on = FALSE;
	s3_log_dump_loop();		/* never returns */
#endif
#if S3_TEST_HUMP
	s3_pause_mission();
	tk_set_flg(flg_evt, EVT_STOP);
	for (n = 1; n <= S3_HUMP_RUNS; n++) {
		TEST_CHECK("S3-M02", s3_hump_run(n),
			   "drove the whole run (compare the height with the ruler)");
	}
#endif
	TEST_SUMMARY("imu_terrain");
#if S3_TEST_HUMP
	s3_hump_replay();		/* never returns */
#endif
}

#endif /* APP_TEST */
