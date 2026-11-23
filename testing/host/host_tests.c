/*
 *	host_tests.c
 *	Unit tests for the pure logic files, run on a PC.  The same checks (and
 *	test IDs) as the on-target tests in each subsystem's _test.c file, so
 *	the logic can be proven before it is flashed.   cd testing/host && make run
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor_control/motor_control.h"
#include "imu_terrain/imu_terrain.h"
#include "imu_terrain/imu_test_hump.h"
#include "dashboard/dashboard.h"
#include "mission_obstacle/mission_obstacle.h"
#include "dashboard/protocol_bits.h"

static int pass_, fail_;

#define CHECK_T(id, cond, desc) do { \
	if (cond) { pass_++; printf("PASS  %-9s %s\n", id, desc); } \
	else { fail_++; printf("FAIL  %-9s %s  (line %d)\n", id, desc, __LINE__); } } while (0)

static int has(const char *s, const char *sub)
{
	return strstr(s, sub) != NULL;
}

static MISSION_ACT bypass_last;	/* actions of the last leg bypass_legs() ran */

/* Run n bypass legs: each one captures Motor's counter, then finishes.
   Puts the command Mission sent after each finished leg in out[]. */
static int bypass_legs(MISSION *m, UW *seq, int n, MSG_NAV *out, int max);

static MISSION_ACT clear_act(void)
{
	MISSION_ACT a;

	mission_act_clear(&a);
	return a;
}

static int bypass_legs(MISSION *m, UW *seq, int n, MSG_NAV *out, int max)
{
	int k = 0, i;

	for (i = 0; i < n; i++) {
		bypass_last = clear_act();
		mission_on_tick(m, 5, MOT_TURNING, *seq, &bypass_last);	/* leg running */
		(*seq)++;
		bypass_last = clear_act();
		mission_on_tick(m, 5, MOT_STOPPED, *seq, &bypass_last);	/* leg done */
		if (bypass_last.n_nav > 0 && k < max) {
			out[k++] = bypass_last.nav[0];
		}
	}
	return k;
}

int main(void)
{
	int n;

	/* ---------------- subsystem 1 ---------------- */
	n = qdec_step(0, 1) + qdec_step(1, 3) + qdec_step(3, 2) + qdec_step(2, 0);
	CHECK_T("S1-U01", n == 4, "quadrature: 4 forward edges = +4");
	n = qdec_step(0, 2) + qdec_step(2, 3) + qdec_step(3, 1) + qdec_step(1, 0);
	CHECK_T("S1-U02", n == -4 && qdec_step(0, 3) == 0, "quadrature: 4 reverse edges = -4, double jump 0");
	/* integer rounding (2654 / 10 = 265 counts per tick) allows 1 % on the speed */
	n = counts_to_mmps(ENC_COUNTS_PER_REV / 10) - WHEEL_MM_PER_REV * 10;
	CHECK_T("S1-U03", mm_to_counts(WHEEL_MM_PER_REV) == ENC_COUNTS_PER_REV &&
		abs(n) * 100 <= WHEEL_MM_PER_REV * 10, "unit conversions agree (1%)");
	{
		PID p;
		int i;

		pid_init(&p, PID_KP, PID_KI, PID_KD, -DUTY_MAX, DUTY_MAX);
		CHECK_T("S1-U04", pid_step(&p, 100, 0) > 0, "PID: too slow -> push harder");
		pid_reset(&p);
		CHECK_T("S1-U05", pid_step(&p, 0, 100) < 0, "PID: too fast -> back off");
		pid_reset(&p);
		for (i = 0; i < 1000; i++) { n = pid_step(&p, 10000, 0); }
		CHECK_T("S1-U06", n == DUTY_MAX, "PID: output clamped");
		/* closed loop on a simple first-order motor model settles on target */
		{
			int speed = 0, k, duty;

			pid_reset(&p);
			for (k = 0; k < 300; k++) {
				duty = DUTY_FF_PER_MMPS * 120 + pid_step(&p, 120, speed);
				if (duty > DUTY_MAX) duty = DUTY_MAX;
				speed += (duty / 3 - speed) / 4;	/* motor: 3 permille per mm/s */
			}
			CHECK_T("S1-H01", speed > 115 && speed < 125, "PID reaches 120 mm/s on a model motor");
		}
	}
	{
		int back = counts_to_deg(deg_to_counts(90));
		CHECK_T("S1-H03", back >= 85 && back <= 95, "heading from encoders round-trips a 90 degree turn (FR-16)");
	}
	/* integer rounding: the 180 degree goal may differ from 2 x 90 by 1 % */
	n = deg_to_counts(180) - 2 * deg_to_counts(90);
	CHECK_T("S1-H02", deg_to_counts(90) > 0 && abs(n) * 100 <= 2 * deg_to_counts(90),
		"turn distance scales with the angle (1%)");
	{
		FAULT_MON fm;
		INT duty[2] = { 500, 500 };
		INT d_enc[2] = { 0, 5 };	/* left silent, right counting */
		INT d_stall[2] = { 0, 0 };	/* both silent */
		UB f = 0;
		int k, first = -1;

		fault_mon_reset(&fm);
		for (k = 0; k < ENC_FAULT_TICKS; k++) {
			f = fault_mon_step(&fm, duty, d_enc);
			if (f != 0U && first < 0) { first = k + 1; }
		}
		CHECK_T("S1-H04", f == FAULT_ENCODER && first == ENC_FAULT_TICKS,
			"one wheel silent while the other turns -> encoder fault after 300 ms");
		fault_mon_reset(&fm);
		for (k = 0; k < ENC_FAULT_TICKS; k++) { f = fault_mon_step(&fm, duty, d_stall); }
		CHECK_T("S1-H05", f == FAULT_MOTOR_STALL, "both wheels driven but silent -> motor stall");
		duty[0] = duty[1] = ENC_FAULT_DUTY;	/* low duty: wheels may stand still */
		fault_mon_reset(&fm);
		for (k = 0; k < 3 * ENC_FAULT_TICKS; k++) { f = fault_mon_step(&fm, duty, d_stall); }
		CHECK_T("S1-H06", f == 0U, "no fault at low duty with no counts");
	}
	CHECK_T("S1-H07", nav_watchdog_speed(0, SPEED_NORMAL_MMPS) == SPEED_NORMAL_MMPS &&
		nav_watchdog_speed(NAV_SLOW_TICKS, SPEED_NORMAL_MMPS) == SPEED_SLOW_MMPS &&
		nav_watchdog_speed(NAV_STOP_TICKS, SPEED_NORMAL_MMPS) == 0,
		"Mission watchdog: normal, then slow after 200 ms, then stop after 500 ms");

	/* ---------------- subsystem 2 ---------------- */
	/* The line and barcode tests (S2-xxx) have their own, larger runner:
	   cd testing/line_barcode && make run */

	/* ---------------- subsystem 3 ---------------- */
	{
		static IMU_LOGIC l;		/* ~1 KB */
		TH_HUMP flat = { 0, 0, 1, 0, 0 };
		TH_HUMP h20 = { 400, 20, 60, 50, 1 };	/* 20 mm under the right wheel */
		TH_HUMP h10 = { 400, 10, 40, 30, 1 };
		TH_HUMP h70 = { 400, 70, 200, 40, 1 };	/* above HUMP_MAX_MM */
		TH_HUMP w20 = { 400, 20, 60, 50, 0 };	/* 20 mm across the whole track */
		IMU_VEC rest = { 0, 0, 1000 };
		TH_OUT o;
		INT rem = 0, k, sum = 0, first, ev;

		imu_logic_init(&l);
		th_calibrate(&l);
		o = th_drive(&l, &flat, 0, 1500, -1, 0, 0, -1);
		CHECK_T("S3-U01", l.state == IMU_MONITORING && l.calibrated &&
			th_abs(l.rest.fwd - 35) <= 10 && o.starts == 0 && o.dones == 0,
			"calibrates (2 deg mount tilt), flat road is quiet");
		o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, -1);
		first = o.height_mm;
		CHECK_T("S3-U02", o.starts == 1 && o.dones == 1 && th_abs(first - 20) <= 2,
			"20 mm hump under one wheel: start + done, height within 2 mm");
		o = th_drive(&l, &h10, 0, 1000, -1, 0, 0, -1);
		CHECK_T("S3-U03", o.dones == 1 && th_abs(o.height_mm - 10) <= 2 &&
			l.max_height_mm == first, "10 mm hump measured, tallest (20) kept");

		imu_logic_init(&l);
		th_calibrate(&l);
		o = th_drive(&l, &flat, 0, 400, 200, 3500, 2, -1);
		CHECK_T("S3-U08", o.starts == 0 && o.dones == 0 && o.lifts == 0,
			"a 3.5 g knock is neither a hump nor a lift");
		o = th_drive(&l, &flat, 0, 400, 200, 3500, 5, -1);
		CHECK_T("S3-U05a", o.lifts == 0 && l.state != IMU_LIFTED, "a long hard knock is not a lift");
		/* S3-U04: lifted and turned upside down (2026-11-17, replaces SHOCK) */
		k = th_hold(&l, 0, 0, -950, LIFT_CONFIRM_SAMPLES - 1);
		CHECK_T("S3-U04a", k == 0 && l.state != IMU_LIFTED, "upside down under 100 ms: not yet");
		k = th_hold(&l, 0, 0, -950, 1);
		CHECK_T("S3-U04b", k == 1 && l.state == IMU_LIFTED, "upside down for 100 ms: LIFTED");
		k = th_hold(&l, 300, -200, -700, 100) + th_hold(&l, 0, 0, -1000, 100);
		CHECK_T("S3-U04c", k == 0 && l.state == IMU_LIFTED, "held upside down and shaken: reported once");
		th_hold(&l, 0, 0, 1000, LIFT_CLEAR_SAMPLES);
		CHECK_T("S3-U04d", l.state == IMU_MONITORING && l.calibrated,
			"back on its wheels for 0.5 s: monitoring again, baseline kept");
		k = th_hold(&l, 0, 1000, 0, 100);
		CHECK_T("S3-U05b", k == 0 && l.state != IMU_LIFTED, "lying on its side is not a lift");
		imu_logic_init(&l);
		k = th_hold(&l, 0, 0, -1000, LIFT_CONFIRM_SAMPLES);
		CHECK_T("S3-U05c", k == 1, "lifted before calibration still counts");
		imu_logic_init(&l);
		th_calibrate(&l);
		o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, -1);
		CHECK_T("S3-U05d", o.dones == 1 && o.lifts == 0, "a hump is never a lift");

		imu_fault(&l);
		CHECK_T("S3-U06a", l.state == IMU_SENSOR_FAULT &&
			imu_feed(&l, &rest, 1200, FALSE, TRUE) == IMU_EV_NONE, "fault ignores samples");
		imu_recovered(&l);
		CHECK_T("S3-U06b", l.state == IMU_MONITORING && l.calibrated, "recovery keeps the baseline");
		imu_logic_init(&l);
		imu_fault(&l);
		imu_recovered(&l);
		CHECK_T("S3-U06c", l.state == IMU_CALIBRATING, "recovery before calibration -> CALIBRATING");

		imu_logic_init(&l);
		th_calibrate(&l);
		o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, -1);
		first = l.max_height_mm;
		o = th_drive(&l, &h70, 0, 1200, -1, 0, 0, -1);
		CHECK_T("S3-U09", o.dones == 1 && o.height_mm > HUMP_MAX_MM && l.max_height_mm == first,
			"hump above 63 mm reported, not kept as tallest");
		o = th_drive(&l, &h20, 0, 1000, -1, 0, 0, 450);
		CHECK_T("S3-U10", o.dones == 0 && l.state == IMU_MONITORING, "turn during a hump: given up");

		for (k = 0; k < 1000; k++) {
			sum += imu_counts_to_um(&rem, 30, WHEEL_MM_PER_REV, 2 * ENC_COUNTS_PER_REV);
		}
		/* 15000 counts per wheel * 216 / 2654 mm = 1220.799 mm */
		CHECK_T("S3-U11", th_abs(sum - 1220799) <= 1, "encoder counts -> um, no rounding drift");

		imu_logic_init(&l);
		imu_set_mode(&l, HUMP_MODE_PITCH);
		th_calibrate(&l);
		o = th_drive(&l, &w20, 0, 1000, -1, 0, 0, -1);
		CHECK_T("S3-U12", o.dones == 1 && th_abs(o.height_mm - 20) <= 2,
			"pitch mode: 20 mm hump across the track within 2 mm");

		imu_logic_init(&l);
		th_calibrate(&l);
		o = th_drive(&l, &h20, 0, 508, -1, 0, 0, -1);	/* stops on the top */
		CHECK_T("S3-U13", o.dones == 0 && imu_finish(&l) == IMU_EV_HUMP_DONE &&
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
			CHECK_T("S3-U14a", k == 300 && l.state == IMU_CALIBRATING && !l.calibrated,
			        "not calibrated: no hump from a tilted board, stays CALIBRATING");
			for (k = 0; k < IMU_CAL_SAMPLES + 5; k++) {
				ev = imu_feed(&l, &tilted, 0, TRUE, TRUE);
			}
			CHECK_T("S3-U14b", l.calibrated && l.state == IMU_MONITORING && ev == IMU_EV_NONE &&
			        imu_height_now_um(&l) == 0, "calibrates at rest even with |a| = 885 mg; then quiet");
		}
	}

	/* ---------------- subsystem 4: binary frames (TEAM.md 7) ---------------- */
	{
		MSG_TELEM t;
		DASH_DEBUG g;
		UB f[DASH_DEBUG_BYTES];
		uint32_t cur = 0, v[25];
		static const uint32_t tmax[12] = { 255, 15, 511, 65535, 359, 1, 255, 31, 31, 127, 1, 255 };
		static const uint32_t gmax[25] = { 255, 1023, 1023, 1023, 2047, 3, 255, 255, 255, 31, 4095, 7, 15, 63,
						   7, 4095, 4095, 4095, 4095, 4095, 3, 7, 4095, 15, 3 };
		int i, ok = 1;

		memset(&t, 0, sizeof t);
		t.sys_state = SYS_LINE_FOLLOW; t.speed_mmps = -40; t.dist_mm = 70000; t.heading_deg = -90;
		t.ir_line = 1; t.obs_dist_mm = 3000; t.last_barcode = 'Z'; t.hump_max_mm = 200; t.lifted = 1;
		t.fault_flags = 0x85;
		CHECK_T("S4-U10a", dash_pack_telem(&t, 9, 2, f, DASH_TELEM_BYTES) == DASH_TELEM_BYTES,
			"telemetry packs into 11 bytes");
		for (i = 0; i < 12; i++) {
			ok &= protocol_bits_get(f, DASH_TELEM_BYTES, &cur, tmax[i], &v[i]);
		}
		CHECK_T("S4-U10b", ok && cur == 81 && v[0] == 9 && v[1] == SYS_LINE_FOLLOW && (int)v[2] - 256 == -40 &&
			v[3] == 70000 % 65536 && v[4] == 270 && v[5] == 1 && v[6] == 255 && v[7] == 25 &&
			v[8] == 2 && v[9] == 127 && v[10] == 1 && v[11] == 0x85,
			"every field unpacks; out-of-range values clamped, heading wrapped");
		CHECK_T("S4-U10c", dash_pack_telem(&t, 0, 0, f, 10) == -1 && dash_char_code('a') == DASH_CHAR_NONE &&
			dash_char_code('A') == 0, "short buffer refused; only A-Z have a code");

		memset(&g, 0, sizeof g);
		g.dark_c = 600; g.dark_r = 1500; g.err = -700; g.steer = 999; g.mode = 2;
		g.bc_edges = 300; g.bc_ok = 1; g.bc_fail = 3; g.bc_last = 'B'; g.ultra_mm = 289; g.motor_state = MOT_DRIVING;
		g.bc_status = 6; g.bc_count = 90;
		g.scan_state = 3; g.scan_mm[0] = 1200; g.scan_mm[1] = 900; g.scan_mm[2] = 250; g.scan_mm[3] = 700;
		g.scan_mm[4] = 9000; g.scan_side = 2; g.bc_exit = 3;
		g.echo_rise_us = 480; g.echo_edges = 40; g.byp_result = BYPR_NO_LINE;
		cur = 0; ok = 1;
		CHECK_T("S4-U12a", dash_pack_debug(&g, 200, f, DASH_DEBUG_BYTES) == DASH_DEBUG_BYTES,
			"debug frame packs into 24 bytes");
		for (i = 0; i < 25; i++) {
			ok &= protocol_bits_get(f, DASH_DEBUG_BYTES, &cur, gmax[i], &v[i]);
		}
		CHECK_T("S4-U12b", ok && cur == 191 && v[24] == BYPR_NO_LINE && v[22] == 480 && v[23] == 15 && v[21] == 3 && v[14] == 3 && v[15] == 1200 && v[16] == 900 &&
			v[17] == 250 && v[18] == 700 && v[19] == 4095 && v[20] == 2 && v[12] == 6 && v[13] == 63 && v[0] == 200 && v[1] == 600 && v[2] == 1023 && (int)v[3] - 512 == -512 &&
			(int)v[4] - 1024 == 999 && v[5] == 2 && v[6] == 300 % 256 && v[7] == 1 && v[8] == 3 &&
			v[9] == 1 && v[10] == 289 && v[11] == MOT_DRIVING,
			"every debug field unpacks; out-of-range values clamped");
	}
	{
		UW e[4] = { 1000, 13300, 20000, 20000 + 7000000 };
		UB f[DASH_BC_TRACE_MAX];
		INT n = dash_pack_bc_trace(9, 2, e, 4, f, (INT)sizeof(f));

		CHECK_T("S4-U13", n == 9 && f[0] == 9 && f[1] == 2 && f[2] == 4 &&
			f[3] == 123 && f[4] == 0 && f[5] == 67 && f[7] == 0xFF && f[8] == 0xFF &&
			dash_pack_bc_trace(0, 0, e, 41, f, (INT)sizeof(f)) == -1 &&
			dash_pack_bc_trace(0, 0, e, 4, f, 8) == -1,
			"barcode trace: widths in 0.1 ms, long ones clamped, bad sizes refused");
	}

	/* ---------------- subsystem 4 ---------------- */
	{
		MSG_TELEM t;
		char buf[DASH_JSON_MAX];
		int len;
		DASH d;
		DASH_IN in;
		DASH_OUT o;

		memset(&t, 0, sizeof t);
		t.sys_state = SYS_LINE_FOLLOW; t.speed_mmps = 120; t.last_barcode = 'B'; t.heading_deg = -45;
		len = dash_format_telem(&t, buf, sizeof buf);
		CHECK_T("S4-U01", len > 0 && buf[0] == '{' && buf[len - 1] == '}' &&
			has(buf, "\"state\":\"LINE_FOLLOW\"") && has(buf, "\"barcode\":\"B\"") &&
			has(buf, "\"speed_mmps\":120") && has(buf, "\"heading\":-45"), "telemetry JSON fields incl. heading");
		t.speed_mmps = INT32_MIN; t.dist_mm = 0xFFFFFFFFu; t.enc_left = INT32_MIN;
		t.enc_right = INT32_MAX; t.heading_deg = INT32_MIN; t.obs_dist_mm = INT32_MAX; t.hump_max_mm = INT32_MIN;
		t.uptime_ms = 0xFFFFFFFFu;
		len = dash_format_telem(&t, buf, sizeof buf);
		printf("      worst-case frame is %d bytes\n", len);
		CHECK_T("S4-U02", len > 0 && len <= DASH_JSON_MAX, "worst-case frame fits 512 bytes");
		CHECK_T("S4-U03", dash_format_telem(&t, buf, 20) == -1, "overflow detected");
		CHECK_T("S4-U04", dash_parse_cmd("\x01", 1) == MQTTCMD_START &&
			dash_parse_cmd("\x00", 1) == MQTTCMD_STOP,
			"1-byte commands: bit 0 = 1 START, 0 STOP");
		CHECK_T("S4-U04b", dash_parse_cmd("\x03", 1) == 0 && dash_parse_cmd("\x80", 1) == 0 &&
			dash_parse_cmd("\x01\x00", 2) == 0 && dash_parse_cmd("", 0) == 0 &&
			dash_parse_cmd("START", 5) == 0 && dash_parse_cmd(NULL, 1) == 0,
			"stray bits, wrong length and old text commands are ignored");

		dash_init(&d);
		#define STEP(w, e, tel, dt) do { in.wifi_up = (w); in.mqtt_event = (e); in.got_telem = (tel); in.dt_ms = (dt); dash_step(&d, &in, &o); } while (0)
		STEP(0, 0, 0, 100);
		CHECK_T("S4-U05a", d.state == DASH_WIFI_CONNECTING, "no WiFi: wait");
		STEP(1, 0, 0, 100);
		CHECK_T("S4-U05b", d.state == DASH_MQTT_CONNECTING, "WiFi -> MQTT_CONNECTING");
		STEP(1, MQTTM_CONNECTED, 0, 100);
		CHECK_T("S4-U05c", d.state == DASH_ONLINE, "broker -> ONLINE");
		STEP(1, 0, 1, 100);
		CHECK_T("S4-U05d", o.publish_telem, "ONLINE publishes");
		STEP(1, 0, 0, DASH_HEARTBEAT_AFTER_MS);
		CHECK_T("S4-U05e", d.state == DASH_HEARTBEAT, "silence -> HEARTBEAT");
		STEP(1, 0, 0, DASH_HEARTBEAT_EVERY_MS);
		CHECK_T("S4-U05f", o.publish_heartbeat, "heartbeat published");
		STEP(1, 0, 1, 100);
		CHECK_T("S4-U05g", d.state == DASH_ONLINE, "telemetry -> ONLINE");
		STEP(0, 0, 0, 100);
		CHECK_T("S4-U05h", d.state == DASH_RECONNECTING, "link lost -> RECONNECTING");
		STEP(0, 0, 0, DASH_RECONNECT_MS);
		CHECK_T("S4-U05i", d.state == DASH_WIFI_CONNECTING, "-> WIFI_CONNECTING");
	}

	/* ---------------- subsystem 5 ---------------- */
	{
		MISSION m; MISSION_ACT a = clear_act();
		MSG_BCCMD b; MSG_OBS o; MSG_TERRAIN t; MSG_STEER s; MSG_MQTTCMD q;
		SCAN_RESULT r; int i;

		memset(&t, 0, sizeof t); memset(&s, 0, sizeof s); memset(&o, 0, sizeof o);
		mission_init(&m);
		mission_on_tick(&m, MS_INIT, MOT_STOPPED, 0, &a);
		CHECK_T("S5-U01d", m.state == SYS_INIT, "INIT waits for WiFi/MQTT");
		mission_on_tick(&m, MS_INIT_NET, MOT_STOPPED, 0, &a);
		CHECK_T("S5-U01e", m.state == SYS_IDLE, "INIT times out: usable offline");
		mission_init(&m); m.net_online = 1; m.track_enabled = 0;	/* U06: the dog-leg */
		mission_on_tick(&m, MS_INIT, MOT_STOPPED, 0, &a);
		CHECK_T("S5-U01a", m.state == SYS_IDLE, "INIT -> IDLE once online");
		mission_on_button(&m, &a);
		CHECK_T("S5-U01b", m.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_START), "button -> LINE_FOLLOW");
		a = clear_act(); mission_on_button(&m, &a);
		CHECK_T("S5-U01c", m.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "button -> IDLE");
		mission_on_button(&m, &a); a = clear_act();
		s.steer = 500; s.flags = STEER_F_JUNCTION; mission_on_steer(&m, &s, &a);
		CHECK_T("S5-H05a", m.state == SYS_LINE_FOLLOW && a.n_nav == 1 && a.nav[0].type == NAV_STEER &&
			a.nav[0].arg == 0, "side branch: drives straight on");
		a = clear_act(); s.flags = STEER_F_LOST | STEER_F_TIMEOUT; mission_on_steer(&m, &s, &a);
		CHECK_T("S5-H05b", m.state == SYS_LINE_FOLLOW && a.nav[0].arg == 0, "no line ahead: no fault during the creep");
		a = clear_act(); mission_on_tick(&m, JUNCTION_CREEP_MS - 5, MOT_DRIVING, 0, &a);
		CHECK_T("S5-H05c", m.state == SYS_LINE_FOLLOW && !(a.evt_flags & EVT_STOP) && m.creeping,
			"still driving straight 5 ms short of 1.5 s");
		mission_on_tick(&m, 5, MOT_DRIVING, 0, &a);
		CHECK_T("S5-H05d", m.state == SYS_LINE_FOLLOW && a.n_nav == 0 && !m.creeping,
			"no letter stored: straight over the junction, no turn");
		a = clear_act(); s.steer = 200; s.flags = 0; mission_on_steer(&m, &s, &a);
		CHECK_T("S5-H05e", a.n_nav == 1 && a.nav[0].type == NAV_STEER && a.nav[0].arg == 200,
			"after the junction: line following again");
		a = clear_act(); mission_on_button(&m, &a);	/* back to IDLE for the tests below */
		/* S5-U20: 'Z' stored -> U-turn at the next junction, then drive on */
		{
			MISSION   mj;
			MSG_BCCMD bj;

			mission_init(&mj); mj.state = SYS_LINE_FOLLOW; a = clear_act();
			bj.kind = BC_OK; bj.cmd = 'Z'; mission_on_barcode(&mj, &bj, &a);
			CHECK_T("S5-U20a", mj.state == SYS_LINE_FOLLOW && a.n_nav == 0 && mj.stored_char == 'Z',
				"barcode Z: stored, no turn yet, keeps following the line");
			s.steer = 0; s.flags = STEER_F_JUNCTION; mission_on_steer(&mj, &s, &a);
			a = clear_act(); mission_on_tick(&mj, JUNCTION_CREEP_MS, MOT_DRIVING, 0, &a);
			CHECK_T("S5-U20b", mj.state == SYS_BARCODE_CMD && a.n_nav == 1 && a.nav[0].type == NAV_TURN &&
				a.nav[0].arg == 180 && mj.stored_char == 0,
				"1.5 s after the junction: U-turn sent, stored letter cleared");
			a = clear_act(); mission_on_tick(&mj, 10, MOT_TURNING, 7, &a);
			mission_on_tick(&mj, 10, MOT_TURNING, 7, &a);
			CHECK_T("S5-U20c", mj.state == SYS_BARCODE_CMD && a.evt_flags == 0, "still turning");
			mission_on_tick(&mj, 10, MOT_STOPPED, 8, &a);
			CHECK_T("S5-U20d", mj.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_RESUME) &&
				!(a.evt_flags & EVT_STOP), "turn done: line following continues");
			a = clear_act(); s.flags = STEER_F_JUNCTION; mission_on_steer(&mj, &s, &a);
			a = clear_act(); mission_on_tick(&mj, JUNCTION_CREEP_MS, MOT_DRIVING, 8, &a);
			CHECK_T("S5-U20e", mj.state == SYS_LINE_FOLLOW && a.n_nav == 0,
				"same junction on the way back: letter used up, straight over");
			/* S5-U21: an unmapped letter goes forward; left/right when mapped */
			CHECK_T("S5-U21a", mission_junction_deg(&mj, 'A') == 0 && mission_junction_deg(&mj, 'F') == 0 &&
				mission_junction_deg(&mj, 0) == 0 && mission_junction_deg(&mj, 'Z') == 180 &&
				mission_junction_deg(&mj, 'Q') == 90, "Z = U-turn, Q = right turn; others go forward");
			mj.move_map[JMOVE_LEFT] = 'L'; mj.move_map[JMOVE_RIGHT] = 'R';
			CHECK_T("S5-U21b", mission_junction_deg(&mj, 'L') == -90 && mission_junction_deg(&mj, 'R') == 90,
				"left and right follow the mapping table");
			s.flags = 0;
		}
		/* S5-H12: integrated run, Mission calling Motor, Line, Barcode and IMU */
		{
			MISSION     mi;
			MSG_BCCMD   bi;
			MSG_TERRAIN ti;

			mission_init(&mi); mi.net_online = 1; a = clear_act();
			mission_on_tick(&mi, MS_INIT, MOT_STOPPED, 0, &a);
			mission_on_button(&mi, &a);
			CHECK_T("S5-H12a", mi.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_START) &&
				a.nav[0].type == NAV_RESUME, "INIT -> IDLE -> GP20: line following, Motor started");
			a = clear_act(); s.flags = 0; s.steer = 250; mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12b", a.n_nav == 1 && a.nav[0].type == NAV_STEER && a.nav[0].arg == 250,
				"Line steering passed to Motor");
			a = clear_act(); bi.kind = BC_SEEN; bi.cmd = 0; mission_on_barcode(&mi, &bi, &a);
			mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12c", mi.state == SYS_BARCODE_SCAN && a.n_nav == 1 && a.nav[0].arg == 250,
				"barcode seen: keeps following the line over it");
			a = clear_act(); bi.kind = BC_OK; bi.cmd = 'Z'; mission_on_barcode(&mi, &bi, &a);
			CHECK_T("S5-H12d", mi.state == SYS_LINE_FOLLOW && mi.stored_char == 'Z' && a.n_nav == 0,
				"Z decoded: stored for the junction, no turn yet");
			a = clear_act(); memset(&ti, 0, sizeof ti); ti.kind = TERR_HUMP_START; mission_on_terrain(&mi, &ti, &a);
			mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12e", mi.state == SYS_HUMP_TRAVERSE && a.nav[0].type == NAV_SLOW &&
				a.nav[1].type == NAV_STEER && a.nav[1].arg == 250, "IMU hump: slow down, still follow the line");
			a = clear_act(); mission_on_tick(&mi, 8000, MOT_DRIVING, 0, &a);
			CHECK_T("S5-H12f", mi.state == SYS_HUMP_TRAVERSE, "a slow 8 s hump crossing is not cut short");
			ti.kind = TERR_HUMP_DONE; ti.height_mm = 20; ti.max_mm = 22; mission_on_terrain(&mi, &ti, &a);
			CHECK_T("S5-H12g", mi.state == SYS_LINE_FOLLOW && a.nav[0].type == NAV_RESUME &&
				mi.hump_max_mm == 22 && mi.stored_char == 'Z', "hump done: normal speed, height kept, letter kept");
			a = clear_act(); ti.kind = TERR_FAULT; mission_on_terrain(&mi, &ti, &a);
			mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12h", mi.state == SYS_LINE_FOLLOW && (mi.fault_flags & FAULT_IMU) &&
				!(a.evt_flags & EVT_STOP) && a.nav[0].type == NAV_STEER, "IMU fault: reported, car keeps driving");
			a = clear_act(); s.flags = STEER_F_JUNCTION; mission_on_steer(&mi, &s, &a);
			mission_on_tick(&mi, JUNCTION_CREEP_MS, MOT_DRIVING, 0, &a);
			CHECK_T("S5-H12i", mi.state == SYS_BARCODE_CMD && a.nav[a.n_nav - 1].type == NAV_TURN &&
				a.nav[a.n_nav - 1].arg == 180 && mi.stored_char == 0, "cross junction: U-turn for the stored Z");
			a = clear_act(); mission_on_tick(&mi, 5, MOT_TURNING, 11, &a);
			mission_on_tick(&mi, 5, MOT_STOPPED, 12, &a);
			s.flags = 0; mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12j", mi.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_RESUME) &&
				a.nav[a.n_nav - 1].type == NAV_STEER && a.nav[a.n_nav - 1].arg == 250,
				"turn done: back to following the line");
			a = clear_act(); mission_on_button(&mi, &a);
			CHECK_T("S5-H12k", mi.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "GP20 again: stop");
			s.flags = 0;
		}
		/* S5-H13: first bar -> back up 2 cm -> read slowly -> normal speed */
		{
			MISSION   mr;
			MSG_BCCMD br;

			mission_init(&mr); mr.state = SYS_LINE_FOLLOW; a = clear_act();
			br.kind = BC_FIRST; br.cmd = 0; mission_on_barcode(&mr, &br, &a);
			CHECK_T("S5-H13a", mr.state == SYS_BARCODE_SCAN && mr.bc_phase == BCP_BACKUP &&
				(a.evt_flags & EVT_BC_HOLD) && a.n_nav == 1 && a.nav[0].type == NAV_MOVE &&
				a.nav[0].arg == -BC_BACKUP_MM, "code starts: reader on hold, back up 10 cm");
			a = clear_act(); s.steer = 400; s.flags = STEER_F_JUNCTION | STEER_F_LOST | STEER_F_TIMEOUT;
			mission_on_steer(&mr, &s, &a);
			CHECK_T("S5-H13b", a.n_nav == 0 && !mr.creeping && mr.state == SYS_BARCODE_SCAN &&
				!(a.evt_flags & EVT_STOP), "backing up: no steering, no junction, no line-lost fault");
			a = clear_act(); mission_on_tick(&mr, 5, MOT_TURNING, 3, &a);
			mission_on_tick(&mr, 5, MOT_TURNING, 3, &a);
			CHECK_T("S5-H13c", mr.bc_phase == BCP_BACKUP && a.evt_flags == 0, "still backing up");
			mission_on_tick(&mr, 5, MOT_STOPPED, 4, &a);
			CHECK_T("S5-H13d", mr.state == SYS_BARCODE_SCAN && mr.bc_phase == BCP_READ &&
				(a.evt_flags & EVT_RESUME) && (a.evt_flags & EVT_BC_ARM) && a.n_nav == 2 &&
				a.nav[0].type == NAV_SLOW && a.nav[1].type == NAV_STEER,
				"backed up: fresh capture, forward at the slow speed");
			a = clear_act(); mission_on_barcode(&mr, &br, &a);
			CHECK_T("S5-H13e", mr.bc_phase == BCP_READ && a.n_nav == 0 && a.evt_flags == 0,
				"first bar again on the slow pass: no second back-up");
			a = clear_act(); s.steer = 300; s.flags = 0; mission_on_steer(&mr, &s, &a);
			CHECK_T("S5-H13f", a.n_nav == 1 && a.nav[0].type == NAV_STEER && a.nav[0].arg == 300,
				"slow pass follows the line");
			a = clear_act(); br.kind = BC_SEEN; mission_on_barcode(&mr, &br, &a);
			br.kind = BC_OK; br.cmd = 'F'; mission_on_barcode(&mr, &br, &a);
			CHECK_T("S5-H13g", mr.state == SYS_LINE_FOLLOW && mr.stored_char == 'F' && a.n_nav == 1 &&
				a.nav[0].type == NAV_RESUME && mr.bc_exit == BCX_DECODED,
				"decoded: letter stored, back to the normal speed (exit: decoded)");
			mission_init(&mr); mr.state = SYS_LINE_FOLLOW; a = clear_act();
			br.kind = BC_FIRST; mission_on_barcode(&mr, &br, &a);
			mission_on_tick(&mr, 5, MOT_TURNING, 0, &a);
			a = clear_act(); mission_on_tick(&mr, MS_BC_BACKUP, MOT_TURNING, 0, &a);
			CHECK_T("S5-H13h", mr.state == SYS_FAULT_RECOVERY && (a.evt_flags & EVT_STOP) &&
				mr.bc_exit == BCX_BACKUP_FAULT,
				"back-up that never ends -> FAULT_RECOVERY");
			mission_init(&mr); mr.state = SYS_LINE_FOLLOW; mr.bc_phase = BCP_READ; a = clear_act();
			br.kind = BC_FIRST; mission_on_barcode(&mr, &br, &a);
			mission_on_tick(&mr, 5, MOT_TURNING, 0, &a);
			mission_on_tick(&mr, 5, MOT_STOPPED, 1, &a);
			a = clear_act(); mission_on_tick(&mr, MS_BARCODE_SCAN, MOT_DRIVING, 1, &a);
			CHECK_T("S5-H13i", mr.state == SYS_LINE_FOLLOW && a.n_nav == 1 && a.nav[0].type == NAV_RESUME &&
				mr.bc_exit == BCX_TIMEOUT,
				"slow pass with no result: times out at the normal speed");
			/* S5-H13k: the slow pass never crawls further than the code needs */
			mission_init(&mr); mr.state = SYS_LINE_FOLLOW; a = clear_act();
			br.kind = BC_FIRST; mission_on_barcode(&mr, &br, &a);
			mission_on_tick(&mr, 5, MOT_TURNING, 0, &a);
			mission_on_tick(&mr, 5, MOT_STOPPED, 1, &a);
			mission_on_forward(&mr, 1000, &a);
			a = clear_act(); mission_on_forward(&mr, 1000 + BC_READ_MAX_MM - 1, &a);
			CHECK_T("S5-H13k", mr.state == SYS_BARCODE_SCAN && a.n_nav == 0,
				"slow pass still over the code (steering does not count as progress)");
			mission_on_forward(&mr, 1000 + BC_READ_MAX_MM, &a);
			CHECK_T("S5-H13l", mr.state == SYS_LINE_FOLLOW && a.n_nav == 1 && a.nav[0].type == NAV_RESUME &&
				mr.bc_exit == BCX_DISTANCE,
				"past the code with no result: normal speed again (does not wait for the barcode time-out)");
			mission_init(&mr); mr.state = SYS_IDLE; a = clear_act();
			mission_on_barcode(&mr, &br, &a);
			mr.state = SYS_HUMP_TRAVERSE; mission_on_barcode(&mr, &br, &a);
			CHECK_T("S5-H13j", mr.state == SYS_HUMP_TRAVERSE && a.n_nav == 0 && a.evt_flags == 0,
				"first bar while IDLE (hand swipe) or on a hump: no back-up");
			s.flags = 0;
		}
		/* S5-H15: forward read fails -> reverse slowly over the code, read again */
		{
			MISSION   mv;
			MSG_BCCMD bv;

			mission_init(&mv); mv.state = SYS_LINE_FOLLOW; a = clear_act();
			mission_on_forward(&mv, 1000, &a);
			bv.kind = BC_FIRST; bv.cmd = 0; mission_on_barcode(&mv, &bv, &a);
			mission_on_tick(&mv, 5, MOT_TURNING, 0, &a);
			mission_on_tick(&mv, 5, MOT_STOPPED, 1, &a);		/* backed up */
			mission_on_forward(&mv, 900, &a);			/* slow pass starts */
			mission_on_forward(&mv, 1240, &a);			/* 340 mm later */
			a = clear_act(); bv.kind = BC_FAIL; mission_on_barcode(&mv, &bv, &a);
			CHECK_T("S5-H15a", mv.state == SYS_BARCODE_SCAN && mv.bc_phase == BCP_REVERSE &&
				(a.evt_flags & EVT_BC_ARM) && a.n_nav == 1 && a.nav[0].type == NAV_MOVE &&
				a.nav[0].arg == -340 && a.nav[0].arg2 == BC_REVERSE_MMPS,
				"forward read failed: fresh capture, reverse 340 mm at 18 mm/s to the start");
			a = clear_act(); s.steer = 500; s.flags = STEER_F_JUNCTION | STEER_F_TIMEOUT;
			mission_on_steer(&mv, &s, &a);
			CHECK_T("S5-H15b", a.n_nav == 0 && !(a.evt_flags & EVT_STOP) && !mv.creeping,
				"reversing: no steering, junction or line-lost fault");
			s.flags = 0;
			bv.kind = BC_OK; bv.cmd = 'F'; mission_on_barcode(&mv, &bv, &a);
			CHECK_T("S5-H15c", mv.state == SYS_BARCODE_SCAN && mv.bc_phase == BCP_REVERSE &&
				mv.stored_char == 'F' && mv.bc_exit == BCX_DECODED_REV,
				"read backwards: letter stored, keeps reversing to the start");
			a = clear_act(); mission_on_tick(&mv, 5, MOT_TURNING, 1, &a);
			mission_on_tick(&mv, 5, MOT_STOPPED, 2, &a);
			CHECK_T("S5-H15d", mv.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_RESUME) && mv.bc_skip,
				"reverse done: forward at the normal speed");
			a = clear_act(); mission_on_forward(&mv, 1100, &a);
			bv.kind = BC_FIRST; mission_on_barcode(&mv, &bv, &a);
			bv.kind = BC_OK; bv.cmd = 'Q'; mission_on_barcode(&mv, &bv, &a);
			CHECK_T("S5-H15e", mv.state == SYS_LINE_FOLLOW && a.n_nav == 0 && mv.stored_char == 'F',
				"driving over the same code again: ignored, no back-up, letter kept");
			mission_on_forward(&mv, 1240 + BC_SKIP_MARGIN_MM, &a);
			a = clear_act(); bv.kind = BC_FIRST; mission_on_barcode(&mv, &bv, &a);
			CHECK_T("S5-H15f", !mv.bc_skip && mv.state == SYS_BARCODE_SCAN && mv.bc_phase == BCP_BACKUP,
				"past the code: the next barcode is read again");
			/* both reads fail: give up after the reverse, no second reverse */
			mission_on_tick(&mv, 5, MOT_TURNING, 2, &a);
			mission_on_tick(&mv, 5, MOT_STOPPED, 3, &a);
			mission_on_forward(&mv, 1400, &a); mission_on_forward(&mv, 1500, &a);
			bv.kind = BC_FAIL; mission_on_barcode(&mv, &bv, &a);
			a = clear_act(); mission_on_barcode(&mv, &bv, &a);
			CHECK_T("S5-H15g", mv.bc_phase == BCP_REVERSE && a.n_nav == 0, "a second failure while reversing: no new move");
			mission_on_tick(&mv, 5, MOT_TURNING, 3, &a);
			mission_on_tick(&mv, 5, MOT_STOPPED, 4, &a);
			CHECK_T("S5-H15h", mv.state == SYS_LINE_FOLLOW && mv.bc_exit == BCX_FAILED,
				"both reads failed: carry on, reported as failed");
		}
		/* S5-H10: obstacle floor test, straight driving with no line */
		{
			MISSION   ms;
			MSG_OBS   os;

			mission_init(&ms); ms.straight_only = 1; ms.state = SYS_IDLE; a = clear_act();
			mission_on_button(&ms, &a);
			CHECK_T("S5-H10a", ms.state == SYS_LINE_FOLLOW && a.scan_cmd == SCANCMD_ARM,
				"GP20: drives, obstacle detection armed");
			a = clear_act(); s.steer = 700; s.flags = STEER_F_LOST | STEER_F_TIMEOUT | STEER_F_JUNCTION;
			mission_on_steer(&ms, &s, &a);
			CHECK_T("S5-H10b", ms.state == SYS_LINE_FOLLOW && a.n_nav == 1 && a.nav[0].type == NAV_STEER &&
				a.nav[0].arg == 0 && !ms.creeping && !(a.evt_flags & EVT_STOP),
				"line ignored: steer 0, no junction creep, no line-lost fault");
			a = clear_act(); memset(&os, 0, sizeof os); os.kind = OBS_DETECTED; os.dist_mm = 250;
			mission_on_obs(&ms, &os, &a);
			CHECK_T("S5-H10c", ms.state == SYS_OBSTACLE_SCAN && (a.evt_flags & EVT_STOP),
				"obstacle within 30 cm: stop");
			a = clear_act(); mission_on_tick(&ms, 5, MOT_STOPPED, 0, &a);
			CHECK_T("S5-H10d", a.scan_cmd == SCANCMD_PROFILE, "stopped: servo sweep ordered");
			a = clear_act(); os.kind = OBS_RESULT; os.side = OBS_SIDE_LEFT; os.width_mm = 120;
			mission_on_obs(&ms, &os, &a);
			CHECK_T("S5-H10e", ms.state == SYS_OBSTACLE_BYPASS && a.n_nav == 1 &&
				a.nav[0].type == NAV_TURN && a.nav[0].arg == -BYP_TURN_CMD_DEG,
				"bypass on the clear side: first a 45 degree turn left");
			{
				UW      sq = 3;
				MSG_NAV legs[8];

				bypass_legs(&ms, &sq, 7, legs, 8);
				a = bypass_last;
			}
			CHECK_T("S5-H10f", ms.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_RESUME) &&
				a.scan_cmd == SCANCMD_ARM, "bypass done: straight on, detection armed again (no line search)");
			a = clear_act(); mission_on_tick(&ms, MS_LINE_REACQUIRE + 100, MOT_DRIVING, 4, &a);
			CHECK_T("S5-H10g", ms.state == SYS_LINE_FOLLOW && !(a.evt_flags & EVT_STOP),
				"no line-reacquire timeout fault");
			s.flags = 0;
		}
		/* barcode floor test: stop once a barcode is read or has failed */
		{
			MISSION mb;
			MSG_BCCMD bb;

			mission_init(&mb); mb.stop_at_barcode = 1; mb.state = SYS_LINE_FOLLOW; a = clear_act();
			bb.kind = BC_SEEN; bb.cmd = 0; mission_on_barcode(&mb, &bb, &a);
			CHECK_T("S5-H07a", mb.state == SYS_BARCODE_SCAN && !(a.evt_flags & EVT_STOP),
				"barcode seen: keeps driving over it");
			bb.kind = BC_OK; bb.cmd = 'Q'; mission_on_barcode(&mb, &bb, &a);
			CHECK_T("S5-H07b", mb.state == SYS_IDLE && (a.evt_flags & EVT_STOP) && a.n_nav == 0 &&
				mb.last_barcode == 'Q', "barcode read: stops in IDLE, letter kept, no turn");
			mb.state = SYS_LINE_FOLLOW; a = clear_act();
			bb.kind = BC_FAIL; mission_on_barcode(&mb, &bb, &a);
			CHECK_T("S5-H07c", mb.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "failed barcode also stops");
			mission_init(&mb); mb.state = SYS_IDLE; a = clear_act();
			bb.kind = BC_OK; bb.cmd = 'F'; mission_on_barcode(&mb, &bb, &a);
			CHECK_T("S5-H08", mb.state == SYS_IDLE && a.n_nav == 0 && a.evt_flags == 0 &&
				mb.last_barcode == 'F' && mb.stored_char == 'F',
				"letter read while IDLE (hand swipe): shown and stored, car does not move");
		}
		s.flags = 0; a = clear_act(); mission_on_button(&m, &a); a = clear_act();
		s.steer = 321; mission_on_steer(&m, &s, &a);
		CHECK_T("S5-U02a", a.n_nav == 1 && a.nav[0].type == NAV_STEER && a.nav[0].arg == 321, "steering forwarded");
		b.kind = BC_SEEN; b.cmd = 0; mission_on_barcode(&m, &b, &a);
		CHECK_T("S5-U02b", m.state == SYS_BARCODE_SCAN, "-> BARCODE_SCAN");
		a = clear_act(); b.kind = BC_OK; b.cmd = 'B'; mission_on_barcode(&m, &b, &a);
		CHECK_T("S5-U03", m.state == SYS_LINE_FOLLOW && a.n_nav == 0 && m.stored_char == 'B',
			"barcode read: no immediate turn, letter stored for the junction");
		a = clear_act(); b.kind = BC_SEEN; mission_on_barcode(&m, &b, &a);
		b.kind = BC_FAIL; mission_on_barcode(&m, &b, &a);
		CHECK_T("S5-U04", m.state == SYS_LINE_FOLLOW && a.n_nav == 0, "failed decode: no turn");
		t.kind = TERR_HUMP_START; mission_on_terrain(&m, &t, &a);
		CHECK_T("S5-U05a", m.state == SYS_HUMP_TRAVERSE, "hump -> HUMP_TRAVERSE");
		t.kind = TERR_HUMP_DONE; t.height_mm = 12; t.max_mm = 15; mission_on_terrain(&m, &t, &a);
		CHECK_T("S5-U05b", m.state == SYS_LINE_FOLLOW && m.hump_max_mm == 15, "hump done");
		t.kind = TERR_FAULT; mission_on_terrain(&m, &t, &a);
		CHECK_T("S5-U05c", m.state == SYS_LINE_FOLLOW && (m.fault_flags & FAULT_IMU), "IMU fault does not stop the car");
		a = clear_act(); o.kind = OBS_DETECTED; o.dist_mm = 280; mission_on_obs(&m, &o, &a);
		CHECK_T("S5-U06a", m.state == SYS_OBSTACLE_SCAN && (a.evt_flags & EVT_STOP), "obstacle -> stop");
		a = clear_act(); mission_on_tick(&m, 5, MOT_DRIVING, 0, &a);
		CHECK_T("S5-U06b", a.scan_cmd == 0, "no profile while moving");
		mission_on_tick(&m, 5, MOT_STOPPED, 0, &a);
		CHECK_T("S5-U06c", a.scan_cmd == SCANCMD_PROFILE, "profile ordered");
		a = clear_act(); o.kind = OBS_RESULT; o.side = OBS_SIDE_RIGHT; o.width_mm = 100; mission_on_obs(&m, &o, &a);
		CHECK_T("S5-U06d", m.state == SYS_OBSTACLE_BYPASS && a.nav[0].type == NAV_TURN &&
			a.nav[0].arg == BYP_TURN_CMD_DEG, "-> OBSTACLE_BYPASS, turn 45 right");
		{
			UW      sq = 7;
			MSG_NAV legs[8];

			bypass_legs(&m, &sq, 7, legs, 8);	/* line never seen */
		}
		CHECK_T("S5-U06e", m.state == SYS_LINE_REACQUIRE, "-> LINE_REACQUIRE");
		s.flags = STEER_F_LOST;
		for (i = 0; i < LINE_FOUND_TICKS; i++) { mission_on_steer(&m, &s, &a); }
		CHECK_T("S5-U06f", m.state == SYS_LINE_REACQUIRE, "line lost: keep creeping");
		s.flags = 0;
		for (i = 0; i < LINE_FOUND_TICKS; i++) { mission_on_steer(&m, &s, &a); }
		CHECK_T("S5-U06g", m.state == SYS_LINE_FOLLOW, "line found");
		a = clear_act(); s.flags = STEER_F_TIMEOUT; mission_on_steer(&m, &s, &a);
		CHECK_T("S5-U07a", m.state == SYS_FAULT_RECOVERY, "line lost too long -> FAULT_RECOVERY");
		a = clear_act(); mission_on_tick(&m, MS_FAULT_RECOVERY, MOT_STOPPED, 0, &a);
		CHECK_T("S5-U07b", m.state == SYS_IDLE && !(a.evt_flags & (EVT_START | EVT_RESUME)), "recovery -> IDLE, no restart without button");
		a = clear_act(); q.cmd = MQTTCMD_START; mission_on_mqtt(&m, &q, &a);
		CHECK_T("S5-U08a", m.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_START), "dashboard START drives");
		a = clear_act(); q.cmd = MQTTCMD_STOP; mission_on_mqtt(&m, &q, &a);
		CHECK_T("S5-U08b", m.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "dashboard STOP halts, IDLE");
		a = clear_act(); q.cmd = MQTTCMD_START; mission_on_mqtt(&m, &q, &a);
		CHECK_T("S5-U08c", m.state == SYS_LINE_FOLLOW, "START again");
		a = clear_act(); mission_on_tick(&m, 5, MOT_ENCODER_FAULT, 0, &a);
		CHECK_T("S5-U09", m.state == SYS_FAULT_RECOVERY, "encoder fault -> FAULT_RECOVERY");
		/* NFR-10: a slow bypass is recorded, never aborted (review 8.2) */
		mission_init(&m); m.state = SYS_OBSTACLE_BYPASS; m.done_seq_base = 1; a = clear_act();
		m.byp_phase = BYPP_ALIGN_TURN;		/* the last leg */
		mission_on_tick(&m, BYPASS_TARGET_MS + 3000, MOT_BYPASS, 1, &a);
		CHECK_T("S5-H03a", m.state == SYS_OBSTACLE_BYPASS, "bypass over the 30 s target is not a fault");
		mission_on_tick(&m, 5, MOT_STOPPED, 2, &a);
		CHECK_T("S5-H03b", m.state == SYS_LINE_REACQUIRE && m.bypass_late, "slow bypass finishes, flagged late");

		/* S5-H20: lifted upside down at the end of the run: motors stop */
		{
			MISSION     ml;
			MSG_TERRAIN tl;

			memset(&tl, 0, sizeof tl); tl.kind = TERR_LIFTED;
			mission_init(&ml); ml.state = SYS_OBSTACLE_BYPASS;
			a = clear_act(); mission_on_terrain(&ml, &tl, &a);
			CHECK_T("S5-H20a", ml.state == SYS_IDLE && (a.evt_flags & EVT_STOP) &&
				a.scan_cmd == SCANCMD_DISARM && ml.lifted,
				"lifted mid-bypass: motors stop at once, IDLE");
			ml.state = SYS_LINE_FOLLOW; a = clear_act(); mission_on_terrain(&ml, &tl, &a);
			CHECK_T("S5-H20b", ml.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "lifted while following: stop");
			ml.state = SYS_IDLE; a = clear_act(); mission_on_terrain(&ml, &tl, &a);
			CHECK_T("S5-H20c", ml.state == SYS_IDLE && (a.evt_flags & EVT_STOP), "lifted while idle: stop again, stays IDLE");
			ml.state = SYS_IDLE; a = clear_act(); mission_on_button(&ml, &a);
			CHECK_T("S5-H20d", ml.state == SYS_LINE_FOLLOW && !ml.lifted, "a new start clears the flag");
		}
		CHECK_T("S5-H24", scan_angle_us(0) == SCAN_US_CENTRE && scan_angle_us(-45) == SCAN_US_FAR_LEFT &&
			scan_angle_us(45) == SCAN_US_FAR_RIGHT && scan_angle_us(-90) == SCAN_US_SIDE_LEFT &&
			scan_angle_us(90) == SCAN_US_SIDE_RIGHT && scan_angle_us(-120) == SCAN_US_SIDE_LEFT &&
			scan_angle_us(-22) > SCAN_US_CENTRE && scan_angle_us(-23) < SCAN_US_FAR_LEFT &&
			scan_angle_us(-60) == SCAN_US_60_LEFT && scan_angle_us(70) == SCAN_US_SIDE_RIGHT &&
			scan_angle_us(-65) > SCAN_US_60_LEFT && scan_angle_us(-65) < SCAN_US_SIDE_LEFT &&
			scan_angle_us(10) < SCAN_US_CENTRE && scan_angle_us(10) > SCAN_US_NEAR_RIGHT,
			"servo aim by angle: calibrated points exact, linear in between, clamped at 70");
		/* S5-H23: tracking bypass: follow the obstacle at 15 cm, clear its
		   end, curve in to the line, curve back along it */
		{
			MISSION     mt;
			MSG_OBS     ot;
			MSG_STEER   st;
			UW          sq = 70;
			INT         k, side, far_steer, near_steer;

			memset(&ot, 0, sizeof ot); memset(&st, 0, sizeof st);
			mission_init(&mt); mt.state = SYS_OBSTACLE_SCAN; mt.track_enabled = 1;
			mission_on_heading(&mt, 0); mission_on_forward(&mt, 1000, &a);
			a = clear_act(); ot.kind = OBS_RESULT; ot.side = OBS_SIDE_RIGHT; ot.dist_mm = 250;
			mission_on_obs(&mt, &ot, &a);
			CHECK_T("S5-H23a", mt.byp_phase == BYPP_OUT && (a.evt_flags & EVT_RESUME) &&
				a.scan_cmd == SCANCMD_TRACK && a.scan_arg == 0 &&
				a.nav[0].type == NAV_SPEED && a.nav[0].arg == BYP_OUT_MMPS,
				"no fixed turn: drive off at once, sensor straight at the obstacle");
			side = 1;
			ot.kind = OBS_RANGE; ot.dist_mm = 280;
			mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			far_steer = a.nav[0].arg;
			ot.dist_mm = 150;
			mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			near_steer = a.nav[0].arg;
			CHECK_T("S5-H23r", far_steer >= BYP_OUT_STEER && near_steer > far_steer,
				"curves right, away from it; harder the nearer its front");
			mission_on_heading(&mt, 20);
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23s", a.scan_cmd == SCANCMD_TRACK && a.scan_arg == -20,
				"turned 20 deg right: sensor turns 20 deg left to keep looking at its front");
			mission_on_heading(&mt, BYP_OUT_MAX_DEG + 5);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23t", a.nav[0].arg < 0, "never turned out further than BYP_OUT_MAX_DEG");
			mission_on_heading(&mt, 30);
			ot.dist_mm = 0;
			mission_on_obs(&mt, &ot, &a); mission_on_obs(&mt, &ot, &a); mission_on_obs(&mt, &ot, &a);
			CHECK_T("S5-H23u", mt.byp_phase == BYPP_OUT_CLEAR, "beam misses it 3 times: sensor past its edge");
			mission_on_forward(&mt, mt.fwd_mm + 200, &a);	/* 200 mm at 30 deg: 100 mm out */
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23v", mt.byp_phase == BYPP_OUT_CLEAR, "not yet a car half-width + margin beyond it");
			mission_on_forward(&mt, mt.fwd_mm + 200, &a);	/* 200 mm out: > 165 */
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23w", mt.byp_phase == BYPP_FOLLOW && a.scan_cmd == SCANCMD_TRACK && a.scan_arg == -BYP_FOLLOW_SIDE_DEG &&
				a.nav[0].type == NAV_SPEED && a.nav[0].arg == BYP_MMPS,
				"room for the body (+ left sensor): follow its side, sensor 70 deg left");
			mission_on_heading(&mt, 0);		/* straightened up beside it */
			ot.kind = OBS_RANGE; ot.dist_mm = 350;	/* ~329 mm sideways: too far */
			a = clear_act(); mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			far_steer = a.nav[0].arg;
			ot.dist_mm = 120;			/* ~113 mm: too near */
			mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			near_steer = a.nav[0].arg;
			CHECK_T("S5-H23b", a.nav[0].type == NAV_STEER && far_steer < 0 && near_steer > 0,
				"too far: steer left in towards it; too near: steer right away");
			ot.dist_mm = 60;			/* 56 mm: dangerously close */
			mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23c", a.nav[0].arg == BYP_STEER_MAX * side, "under 70 mm sideways: full steer away");
			mission_on_heading(&mt, 10);		/* drifted 10 deg right (away) */
			ot.dist_mm = 173;			/* x sin 60 = 150 mm: on the gap */
			mission_on_obs(&mt, &ot, &a); mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23d", a.nav[0].arg < 0, "on the gap but angled away: steer back parallel");
			mission_on_heading(&mt, 0);
			ot.dist_mm = 0;
			mission_on_obs(&mt, &ot, &a); mission_on_obs(&mt, &ot, &a);
			CHECK_T("S5-H23e", mt.byp_phase == BYPP_FOLLOW, "two missed pings: still following");
			a = clear_act(); mission_on_obs(&mt, &ot, &a);
			CHECK_T("S5-H23f", mt.byp_phase == BYPP_CLEAR && a.scan_cmd == SCANCMD_DISARM,
				"third: the sensor has passed its end, sensor back to centre");
			mission_on_forward(&mt, mt.byp_mark_mm + BYP_CLEAR_MM, &a);
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23g", mt.byp_phase == BYPP_RETURN, "then BYP_CLEAR_MM on: curve in to the line");
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23h", a.nav[0].arg < 0 && a.nav[0].arg >= -BYP_RETURN_STEER,
				"right pass returns by curving left, gently");
			mission_on_heading(&mt, -BYP_RETURN_DEG);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23i", a.nav[0].arg == 0, "at 30 deg in: hold that heading");
			st.flags = STEER_F_DARK_C; mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23j", mt.byp_phase == BYPP_RETURN, "first sensor on the line: keep going");
			st.flags = STEER_F_DARK_R; a = clear_act(); mission_on_steer(&mt, &st, &a);
			CHECK_T("S5-H23k", mt.byp_phase == BYPP_REJOIN && !(a.evt_flags & EVT_STOP) && a.nav[0].arg > 0,
				"second sensor: curve back right while moving, no stop");
			st.flags = 0;
			for (k = -BYP_RETURN_DEG; k < -BYP_ALIGN_DEG; k += 5) {
				mission_on_heading(&mt, k);
				a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			}
			CHECK_T("S5-H23l", mt.state == SYS_OBSTACLE_BYPASS, "still turning back");
			mission_on_heading(&mt, -3);
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23m", mt.state == SYS_LINE_FOLLOW && mt.line_hint == OBS_SIDE_RIGHT &&
				a.scan_cmd == SCANCMD_ARM && a.nav[0].type == NAV_RESUME,
				"along the line: follow it, told the line is to the right if it is lost");
			mission_on_tick(&mt, BYP_HINT_MS, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23n", mt.line_hint == 0, "hint dropped once the follower has had it");
			/* left pass, and an obstacle whose end never comes: stop */
			mission_init(&mt); mt.state = SYS_OBSTACLE_SCAN; mt.track_enabled = 1; sq = 90;
			mission_on_forward(&mt, 0, &a); mission_on_heading(&mt, 0);
			ot.kind = OBS_RESULT; ot.side = OBS_SIDE_LEFT; ot.dist_mm = 250;
			a = clear_act(); mission_on_obs(&mt, &ot, &a);
			ot.kind = OBS_RANGE; ot.dist_mm = 250; mission_on_obs(&mt, &ot, &a);
			a = clear_act(); mission_on_steer(&mt, &st, &a);
			k = a.nav[0].arg;
			mission_on_heading(&mt, -20);
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23o", k < 0 && a.scan_arg == 20,
				"left pass: curves left, sensor turns right to keep looking at its front");
			ot.dist_mm = 0;
			mission_on_obs(&mt, &ot, &a); mission_on_obs(&mt, &ot, &a); mission_on_obs(&mt, &ot, &a);
			mission_on_heading(&mt, -30);
			mission_on_forward(&mt, 400, &a);
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23x", mt.byp_phase == BYPP_FOLLOW && a.scan_arg == BYP_FOLLOW_SIDE_DEG,
				"left pass needs less room (no sensor on that side): following, sensor right");
			mission_on_heading(&mt, 0);
			ot.kind = OBS_RANGE; ot.dist_mm = 212; mission_on_obs(&mt, &ot, &a);
			mission_on_forward(&mt, mt.byp_mark_mm + BYP_FOLLOW_MAX_MM + 1, &a);
			a = clear_act(); mission_on_tick(&mt, 5, MOT_DRIVING, sq, &a);
			CHECK_T("S5-H23p", mt.state == SYS_FAULT_RECOVERY && (a.evt_flags & EVT_STOP),
				"still beside it after 1.5 m: stop (fault)");
			/* the floor test with no line keeps the dog-leg */
			mission_init(&mt); mt.state = SYS_OBSTACLE_SCAN; mt.straight_only = 1; mt.track_enabled = 1;
			ot.kind = OBS_RESULT; a = clear_act(); mission_on_obs(&mt, &ot, &a);
			CHECK_T("S5-H23q", !mt.byp_track, "no line to come back to: fixed dog-leg");
		}
		/* S5-H25: straight leg of the dog-leg ended by the obstacle itself */
		{
			MISSION     mp;
			MSG_OBS     op;
			MSG_NAV     lp[8];
			UW          sq = 300;
			INT         i;

			memset(&op, 0, sizeof op);
			mission_init(&mp); mp.state = SYS_OBSTACLE_SCAN; mp.track_enabled = 0;
			mission_on_forward(&mp, 0, &a);
			op.kind = OBS_RESULT; op.side = OBS_SIDE_RIGHT; op.dist_mm = 250;
			a = clear_act(); mission_on_obs(&mp, &op, &a);
			bypass_legs(&mp, &sq, 3, lp, 8);	/* turn, diagonal, turn back */
			CHECK_T("S5-H25a", mp.byp_phase == BYPP_PASS && bypass_last.scan_cmd == SCANCMD_TRACK &&
				bypass_last.scan_arg == -BYP_SIDE_AIM_DEG && lp[2].type == NAV_MOVE && lp[2].arg == BYP_PASS_MAX_MM,
				"straight leg: up to 1 m, sensor turned fully sideways (left) at the obstacle");
			a = clear_act(); mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);	/* leg running */
			op.kind = OBS_RANGE; op.dist_mm = 180;
			mission_on_obs(&mp, &op, &a);
			mission_on_forward(&mp, mp.byp_mark_mm + 300, &a);
			a = clear_act(); mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			CHECK_T("S5-H25b", mp.byp_phase == BYPP_PASS && !(a.evt_flags & EVT_STOP),
				"obstacle still beside the car: keep going");
			op.dist_mm = 0;
			mission_on_obs(&mp, &op, &a); mission_on_obs(&mp, &op, &a);
			op.dist_mm = 900; mission_on_obs(&mp, &op, &a);
			CHECK_T("S5-H25c", mp.byp_end_fwd == mp.fwd_mm, "3 readings with nothing there: its far end");
			mission_on_forward(&mp, mp.fwd_mm + BYP_PASS_AFTER_MM - 1, &a);
			a = clear_act(); mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			CHECK_T("S5-H25d", mp.byp_phase == BYPP_PASS, "not yet a car length past it");
			mission_on_forward(&mp, mp.fwd_mm + 1, &a);
			a = clear_act(); mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			CHECK_T("S5-H25e", mp.byp_phase == BYPP_PASS_STOPPING && (a.evt_flags & EVT_STOP) &&
				a.scan_cmd == SCANCMD_DISARM, "a car length past its end: stop, sensor back to centre");
			a = clear_act(); mission_on_tick(&mp, 5, MOT_STOPPED, sq, &a);
			CHECK_T("S5-H25f", mp.byp_phase == BYPP_IN_TURN && a.n_nav == 1 && a.nav[0].type == NAV_TURN &&
				a.nav[0].arg == -BYP_TURN_CMD_DEG, "stopped: turn in towards the line");
			/* never seen beside the car: the planned length */
			mission_init(&mp); mp.state = SYS_OBSTACLE_SCAN; mp.track_enabled = 0;
			mission_on_forward(&mp, 0, &a);
			op.kind = OBS_RESULT; a = clear_act(); mission_on_obs(&mp, &op, &a);
			bypass_legs(&mp, &sq, 3, lp, 8);
			mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			mission_on_forward(&mp, mp.byp_mark_mm + mp.byp.pass_mm, &a);
			a = clear_act(); mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			CHECK_T("S5-H25g", mp.byp_phase == BYPP_PASS_STOPPING, "side never seen: stops at the planned length");
			/* an end found very early still drives the minimum */
			mission_init(&mp); mp.state = SYS_OBSTACLE_SCAN; mp.track_enabled = 0;
			mission_on_forward(&mp, 0, &a);
			op.kind = OBS_RESULT; a = clear_act(); mission_on_obs(&mp, &op, &a);
			bypass_legs(&mp, &sq, 3, lp, 8);
			mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			op.kind = OBS_RANGE; op.dist_mm = 150; mission_on_obs(&mp, &op, &a);
			op.dist_mm = 0;
			for (i = 0; i < 3; i++) { mission_on_obs(&mp, &op, &a); }
			mission_on_forward(&mp, mp.byp_mark_mm + BYP_PASS_MIN_MM - 1, &a);
			a = clear_act(); mission_on_tick(&mp, 5, MOT_TURNING, sq, &a);
			CHECK_T("S5-H25h", mp.byp_phase == BYPP_PASS, "never shorter than BYP_PASS_MIN_MM");
		}
		/* S5-H26: sideways position adds up at real speed (45 mm/s, 5 ms
		   ticks: under 1 mm per tick; it used to round to nothing) */
		{
			MISSION ml;
			MSG_OBS ol;
			INT     t;

			memset(&ol, 0, sizeof ol);
			mission_init(&ml); ml.state = SYS_OBSTACLE_SCAN; ml.track_enabled = 1;
			mission_on_forward(&ml, 0, &a); mission_on_heading(&ml, 0);
			ol.kind = OBS_RESULT; ol.side = OBS_SIDE_RIGHT; ol.dist_mm = 250;
			a = clear_act(); mission_on_obs(&ml, &ol, &a);
			mission_on_heading(&ml, 30);
			for (t = 1; t <= 1600; t++) {		/* 360 mm at 0.225 mm per tick */
				mission_on_forward(&ml, (t * 225) / 1000, &a);
				a = clear_act(); mission_on_tick(&ml, 5, MOT_DRIVING, 0, &a);
			}
			CHECK_T("S5-H26", ml.byp_lat_mm >= 170 && ml.byp_lat_mm <= 185,
				"360 mm at 30 deg in sub-mm steps: about 180 mm sideways (was 0)");
		}
		/* S5-H27: 7-angle sweep, 70 degree readings decide the side */
		{
			SCAN_RESULT sw;
			INT         d4[SCAN_ANGLES] = { 285, 278, 270, 285, 621 };	/* floor run 4 */
			INT         dc[SCAN_ANGLES] = { 290, 280, 270, 280, 300 };	/* square-on, both 45s hit */

			memset(&sw, 0, sizeof sw);
			CHECK_T("S5-H27a", scan_evaluate_wide(d4, 1500, 700, &sw) && sw.side == OBS_SIDE_RIGHT,
				"left 45 hits, right only 22.5: the side it reaches less far, even with left 70 open");
			CHECK_T("S5-H27b", scan_evaluate_wide(dc, 1500, 300, &sw) && sw.side == OBS_SIDE_LEFT &&
				sw.clear_left_mm >= SCAN_CAR_CLEAR_MM && sw.clear_right_mm == 0,
				"both 45s hit: the open 70 degree side gives room (was no way round)");
			CHECK_T("S5-H27c", !scan_evaluate_wide(dc, 500, 300, &sw) && sw.side == OBS_SIDE_LEFT,
				"both 70s hit too: no way round, less blocked side left");
			{
				INT dd[SCAN_ANGLES] = { 900, 400, 270, 400, 650 };

				CHECK_T("S5-H27d", scan_evaluate_wide(dd, 1800, 700, &sw) && sw.side == OBS_SIDE_LEFT,
					"both open: the side whose 45 + 70 readings add up to more");
				CHECK_T("S5-H27e", scan_evaluate_wide(dd, 0, 0, &sw) && sw.side == OBS_SIDE_LEFT,
					"no echo at 70 counts as far");
			}
		}
		/* S5-H28: never seen alongside: curve in once odometry says the car is
		   past; curve back where odometry puts the line if the IR misses it */
		{
			MISSION mr;
			MSG_OBS orr;
			INT     f, x_at = -1;

			memset(&orr, 0, sizeof orr);
			mission_init(&mr); mr.state = SYS_OBSTACLE_SCAN; mr.track_enabled = 1;
			mission_on_forward(&mr, 0, &a); mission_on_heading(&mr, 0);
			orr.kind = OBS_RESULT; orr.side = OBS_SIDE_RIGHT; orr.dist_mm = 250;
			a = clear_act(); mission_on_obs(&mr, &orr, &a);
			mr.byp_phase = BYPP_FOLLOW; mr.byp_mark_mm = 0; mr.byp_seen = 0;
			for (f = 2; f <= 700 && x_at < 0; f += 2) {
				mission_on_forward(&mr, f, &a);
				a = clear_act(); mission_on_tick(&mr, 5, MOT_DRIVING, 0, &a);
				if (mr.byp_phase == BYPP_RETURN) {
					x_at = mr.byp_x_mm;
				}
			}
			CHECK_T("S5-H28a", x_at >= 250 + BYP_OBS_DEPTH_MM + BYP_CAR_LEN_MM && x_at <= 570 &&
				a.scan_cmd == SCANCMD_DISARM,
				"not seen: RETURN as soon as the car is past its far end (550 mm), no extra 600 mm");
			mr.byp_lat_mm = 100;
			mission_on_heading(&mr, -BYP_RETURN_DEG);
			for (f = mr.fwd_mm + 2; f <= mr.fwd_mm + 300 && mr.byp_phase == BYPP_RETURN; f += 2) {
				mission_on_forward(&mr, f, &a);
				a = clear_act(); mission_on_tick(&mr, 5, MOT_DRIVING, 0, &a);
			}
			CHECK_T("S5-H28b", mr.byp_phase == BYPP_REJOIN && mr.byp_result == BYPR_ODOM &&
				mr.byp_lat_mm <= 0 && mr.fwd_mm - mr.byp_mark_mm < 10,
				"100 mm out at 45 deg: curves back after ~140 mm even with no black seen");
		}
		/* S5-H21: "no way round" three times: go round anyway */
		{
			MISSION     mf;
			MSG_OBS     of;
			BYPASS_PLAN pw;

			memset(&of, 0, sizeof of);
			mission_init(&mf); mf.state = SYS_LINE_FOLLOW;
			a = clear_act(); of.kind = OBS_DETECTED; of.dist_mm = 280; mission_on_obs(&mf, &of, &a);
			of.kind = OBS_FAIL; of.side = OBS_SIDE_LEFT; of.width_mm = 500; of.dist_mm = 260;
			a = clear_act(); mission_on_obs(&mf, &of, &a);
			a = clear_act(); mission_on_obs(&mf, &of, &a);
			CHECK_T("S5-H21a", mf.state == SYS_OBSTACLE_SCAN && a.scan_cmd == SCANCMD_PROFILE &&
				a.n_nav == 0 && !mf.byp_forced, "two no-way-round sweeps: still sweeping");
			a = clear_act(); mission_on_obs(&mf, &of, &a);
			mission_bypass_plan(OBS_SIDE_LEFT, 2 * BYP_HALF_MAX_MM, 260, TRUE, &pw);
			CHECK_T("S5-H21b", mf.state == SYS_OBSTACLE_BYPASS && mf.byp_forced &&
				mf.byp.turn_deg < 0 && mf.byp.diag_mm == pw.diag_mm && mf.obs_dist_mm == 260,
				"third: bypass anyway on the less blocked side (left), widest clearance");
			mission_init(&mf); mf.state = SYS_LINE_FOLLOW;
			a = clear_act(); of.kind = OBS_DETECTED; mission_on_obs(&mf, &of, &a);
			of.kind = OBS_FAIL; of.side = 0;
			mission_on_obs(&mf, &of, &a); mission_on_obs(&mf, &of, &a);
			a = clear_act(); mission_on_obs(&mf, &of, &a);
			CHECK_T("S5-H21c", mf.state == SYS_OBSTACLE_BYPASS && mf.byp.turn_deg == BYP_TURN_CMD_DEG * BYP_DEFAULT_SIDE,
				"no side known: the default side");
			mission_init(&mf); mf.state = SYS_LINE_FOLLOW;
			a = clear_act(); of.kind = OBS_DETECTED; mission_on_obs(&mf, &of, &a);
			of.kind = OBS_FAIL; mission_on_obs(&mf, &of, &a); mission_on_obs(&mf, &of, &a);
			a = clear_act(); of.kind = OBS_CLEAR; mission_on_obs(&mf, &of, &a);
			mission_on_obs(&mf, &of, &a);
			CHECK_T("S5-H21d", mf.state == SYS_LINE_FOLLOW && !mf.byp_forced,
				"the obstacle goes after two: drive on, no bypass");
			a = clear_act(); of.kind = OBS_DETECTED; mission_on_obs(&mf, &of, &a);
			of.kind = OBS_FAIL; a = clear_act(); mission_on_obs(&mf, &of, &a);
			CHECK_T("S5-H21e", mf.state == SYS_OBSTACLE_SCAN && a.scan_cmd == SCANCMD_PROFILE,
				"a new obstacle starts counting from zero");
		}
		{
			/* the sweep names the less blocked side even with no room */
			INT dd[SCAN_ANGLES] = { 300, 260, 250, 270, 420 };
			SCAN_RESULT sr;

			memset(&sr, 0, sizeof sr);
			CHECK_T("S5-H21f", !scan_evaluate(dd, &sr) && sr.side == OBS_SIDE_RIGHT,
				"no way round, but right is less blocked: side = right");
		}
		/* S5-H19: dashboard Start / Stop act like the GP20 button */
		{
			MISSION     md;
			MSG_MQTTCMD qc;

			mission_init(&md); md.state = SYS_IDLE;
			a = clear_act(); qc.cmd = MQTTCMD_START; mission_on_mqtt(&md, &qc, &a);
			CHECK_T("S5-H19a", md.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_START) &&
				a.scan_cmd == SCANCMD_ARM, "Start from IDLE: drives, detection armed");
			a = clear_act(); mission_on_mqtt(&md, &qc, &a);
			CHECK_T("S5-H19b", md.state == SYS_LINE_FOLLOW && a.evt_flags == 0 && a.n_nav == 0,
				"Start while running does nothing (never toggles)");
			md.state = SYS_OBSTACLE_BYPASS;
			a = clear_act(); qc.cmd = MQTTCMD_STOP; mission_on_mqtt(&md, &qc, &a);
			CHECK_T("S5-H19c", md.state == SYS_IDLE && (a.evt_flags & EVT_STOP) &&
				a.scan_cmd == SCANCMD_DISARM, "Stop from any running state: IDLE, motors off");
			a = clear_act(); mission_on_mqtt(&md, &qc, &a);
			CHECK_T("S5-H19d", md.state == SYS_IDLE && a.evt_flags == 0, "Stop while idle does nothing");
			md.state = SYS_FAULT_RECOVERY;
			qc.cmd = MQTTCMD_STOP; a = clear_act(); mission_on_mqtt(&md, &qc, &a);
			CHECK_T("S5-H19e", md.state == SYS_IDLE && (a.evt_flags & EVT_STOP),
				"Stop during FAULT_RECOVERY: IDLE");
			mission_init(&md);	/* still INIT */
			qc.cmd = MQTTCMD_START; a = clear_act(); mission_on_mqtt(&md, &qc, &a);
			CHECK_T("S5-H19f", md.state == SYS_INIT && a.evt_flags == 0, "Start during INIT ignored");
		}
		/* S5-H16: 45 degree dog-leg geometry (mission_bypass_plan) */
		{
			BYPASS_PLAN p;
			INT lat = 100 / 2 + BYP_MARGIN_MM + BYP_CAR_HALF_MM + BYP_RIGHT_EXTRA_MM;
			BYPASS_PLAN pl;

			mission_bypass_plan(OBS_SIDE_RIGHT, 100, 250, TRUE, &p);
			CHECK_T("S5-H16a", p.turn_deg == BYP_TURN_CMD_DEG &&
				p.diag_mm == (lat * 1414) / 1000 - BYP_DIAG_TRIM_MM &&
				p.pass_mm == 250 + BYP_OBS_DEPTH_MM + BYP_CAR_LEN_MM - (p.diag_mm * 707) / 1000
					     - BYP_PASS_TRIM_MM &&
				p.in_diag_mm == p.diag_mm + BYP_SEARCH_MM,
				"right: turn +45, diagonal 10 cm shorter than the clearance needs, pass covers the rest");
			mission_bypass_plan(OBS_SIDE_LEFT, 100, 250, FALSE, &p);
			CHECK_T("S5-H16b", p.turn_deg == -BYP_TURN_CMD_DEG && p.in_diag_mm == p.diag_mm,
				"left: turn -45; no line to look for: return diagonal = outward one");
			mission_bypass_plan(OBS_SIDE_RIGHT, 5000, 40, TRUE, &p);
			lat = BYP_WIDTH_MAX_MM / 2 + BYP_MARGIN_MM + BYP_CAR_HALF_MM + BYP_RIGHT_EXTRA_MM;
			CHECK_T("S5-H16c", p.diag_mm == (lat * 1414) / 1000 - BYP_DIAG_TRIM_MM &&
				p.pass_mm >= BYP_PASS_MIN_MM && p.diag_mm >= BYP_DIAG_MIN_MM,
				"absurd width clamped; legs never shorter than their minimums");
			mission_bypass_plan(OBS_SIDE_RIGHT, 250, 300, TRUE, &p);
			mission_bypass_plan(OBS_SIDE_LEFT, 250, 300, TRUE, &pl);
			CHECK_T("S5-H16d", p.diag_mm - pl.diag_mm == (BYP_RIGHT_EXTRA_MM * 1414) / 1000 ||
				p.diag_mm - pl.diag_mm == (BYP_RIGHT_EXTRA_MM * 1414) / 1000 + 1,
				"right pass: diagonal longer by the extra room for the left sensor");
		}
		/* S5-H22: the outward diagonal is sized from how far sideways the
		   sweep saw the obstacle on the chosen side */
		{
			MISSION     me;
			MSG_OBS     oe;
			INT         dg[4], x;
			static const INT reach[4] = { 0, 150, 200, 900 };
			BYPASS_PLAN pmin, pmax;

			memset(&oe, 0, sizeof oe);
			for (x = 0; x < 4; x++) {
				mission_init(&me); me.state = SYS_OBSTACLE_SCAN; me.track_enabled = 0;
				oe.kind = OBS_RESULT; oe.side = OBS_SIDE_LEFT; oe.dist_mm = 250;
				oe.reach_mm = reach[x];
				a = clear_act(); mission_on_obs(&me, &oe, &a);
				dg[x] = me.byp.diag_mm;
			}
			mission_bypass_plan(OBS_SIDE_LEFT, 2 * BYP_HALF_MIN_MM, 250, TRUE, &pmin);
			mission_bypass_plan(OBS_SIDE_LEFT, 2 * BYP_HALF_MAX_MM, 250, TRUE, &pmax);
			CHECK_T("S5-H22a", dg[0] == pmin.diag_mm && dg[1] > dg[0] && dg[2] > dg[1] &&
				dg[3] == pmax.diag_mm && dg[2] - dg[1] >= 60 && dg[2] - dg[1] <= 80,
				"diagonal grows with the measured reach (50 mm more reach: ~71 mm more diagonal), within limits");
		}
		{
			INT dn[SCAN_ANGLES] = { 2000, 2000, 250, 2000, 2000 };	/* narrow */
			INT dw[SCAN_ANGLES] = { 1000, 265, 250, 270, 354 };	/* right: far beam hits */
			SCAN_RESULT se;

			memset(&se, 0, sizeof se);
			CHECK_T("S5-H22b", scan_evaluate(dn, &se) && se.extent_left == SCAN_EXTENT_CENTRE &&
				se.extent_right == SCAN_EXTENT_CENTRE, "only the centre beam hit: extent centre");
			(void)scan_evaluate(dw, &se);
			CHECK_T("S5-H22c", se.extent_left == SCAN_EXTENT_NEAR && se.extent_right == SCAN_EXTENT_FAR &&
				se.reach_left_mm == (265 * 383) / 1000 && se.reach_right_mm == (354 * 707) / 1000,
				"22.5 deg beam hit on the left, 45 deg too on the right");
		}
		/* S5-H17: sweep until a side is clear, dog-leg, stop on the line */
		{
			MISSION   mo;
			MSG_OBS   oo;
			MSG_STEER so;
			MSG_NAV   legs[8];
			UW        sq = 20;
			int       n;
			BYPASS_PLAN p;

			mission_init(&mo); mo.track_enabled = 0; mo.state = SYS_LINE_FOLLOW;
			memset(&oo, 0, sizeof oo); memset(&so, 0, sizeof so);
			a = clear_act(); oo.kind = OBS_DETECTED; oo.dist_mm = 290; mission_on_obs(&mo, &oo, &a);
			a = clear_act(); mission_on_tick(&mo, 5, MOT_STOPPED, sq, &a);
			mission_on_tick(&mo, MS_OBSTACLE_SCAN - 100, MOT_STOPPED, sq, &a);
			a = clear_act(); oo.kind = OBS_FAIL; mission_on_obs(&mo, &oo, &a);
			CHECK_T("S5-H17a", mo.state == SYS_OBSTACLE_SCAN && a.scan_cmd == SCANCMD_PROFILE &&
				!(a.evt_flags & EVT_RESUME) && mo.obs_sweeps == 1,
				"no way round: stays stopped and sweeps again");
			a = clear_act(); mission_on_tick(&mo, MS_OBSTACLE_SCAN - 100, MOT_STOPPED, sq, &a);
			CHECK_T("S5-H17b", mo.state == SYS_OBSTACLE_SCAN, "the scan timeout restarts with each sweep");
			a = clear_act(); oo.kind = OBS_RESULT; oo.side = OBS_SIDE_RIGHT; oo.width_mm = 100;
			oo.dist_mm = 250; mission_on_obs(&mo, &oo, &a);	/* extent 0: narrow */
			mission_bypass_plan(OBS_SIDE_RIGHT, 2 * BYP_HALF_MIN_MM, 250, TRUE, &p);
			CHECK_T("S5-H17c", mo.state == SYS_OBSTACLE_BYPASS && a.n_nav == 1 && a.nav[0].type == NAV_TURN &&
				a.nav[0].arg == BYP_TURN_CMD_DEG && mo.obs_dist_mm == 250,
				"clear side: turn 45 right (sweep's own distance used)");
			n = bypass_legs(&mo, &sq, 5, legs, 8);
			CHECK_T("S5-H17d", n == 5 &&
				legs[0].type == NAV_MOVE && legs[0].arg == p.diag_mm && legs[0].arg2 == BYP_MMPS &&
				legs[1].type == NAV_TURN && legs[1].arg == -BYP_TURN_CMD_DEG &&
				legs[2].type == NAV_MOVE && legs[2].arg == BYP_PASS_MAX_MM &&
				legs[3].type == NAV_TURN && legs[3].arg == -BYP_TURN_CMD_DEG &&
				legs[4].type == NAV_MOVE && legs[4].arg == p.in_diag_mm,
				"out diagonal, turn back, past, turn in, return diagonal");
			a = clear_act(); mission_on_tick(&mo, 5, MOT_TURNING, sq, &a);	/* return leg running */
			so.flags = STEER_F_DARK_C; a = clear_act(); mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H17i", !(a.evt_flags & EVT_STOP) && mo.byp_phase == BYPP_IN_DIAG,
				"passed on the right: the centre (left) sensor reaches the line first, keep going");
			so.flags = STEER_F_DARK_C | STEER_F_DARK_R; a = clear_act(); mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H17e", (a.evt_flags & EVT_STOP) && mo.byp_phase == BYPP_STOPPING && a.n_nav == 0 &&
				mo.byp_result == BYPR_LINE,
				"the right sensor reaches the line too: line between the pair, stop");
			a = clear_act(); mission_on_tick(&mo, 5, MOT_TURNING, sq, &a);
			CHECK_T("S5-H17f", a.n_nav == 0, "no turn until the car has stopped");
			a = clear_act(); mission_on_tick(&mo, 5, MOT_STOPPED, sq, &a);
			CHECK_T("S5-H17g", a.n_nav == 1 && a.nav[0].type == NAV_MOVE && a.nav[0].arg == BYP_PAST_LINE_MM &&
				mo.byp_phase == BYPP_PAST_LINE, "stopped on the line: 5 cm further first");
			n = bypass_legs(&mo, &sq, 1, legs, 8);
			CHECK_T("S5-H17j", n == 1 && legs[0].type == NAV_TURN && legs[0].arg == BYP_TURN_CMD_DEG &&
				mo.byp_phase == BYPP_ALIGN_TURN, "then turn 45 back to face along the line");
			bypass_legs(&mo, &sq, 1, legs, 8);
			CHECK_T("S5-H17h", mo.state == SYS_LINE_REACQUIRE && (bypass_last.evt_flags & EVT_RESUME),
				"facing along the line: creep and reacquire");
		}
		/* S5-H18: black seen on an earlier leg is not the line; obstacle gone */
		{
			MISSION   mo;
			MSG_OBS   oo;
			MSG_STEER so;
			MSG_NAV   legs[8];
			UW        sq = 50;

			mission_init(&mo); mo.track_enabled = 0; mo.state = SYS_OBSTACLE_SCAN;
			memset(&oo, 0, sizeof oo); memset(&so, 0, sizeof so);
			a = clear_act(); oo.kind = OBS_RESULT; oo.side = OBS_SIDE_LEFT; oo.width_mm = 80;
			mission_on_obs(&mo, &oo, &a);
			so.flags = STEER_F_DARK_C | STEER_F_DARK_R; a = clear_act(); mission_on_steer(&mo, &so, &a);
			bypass_legs(&mo, &sq, 3, legs, 8);
			mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H18a", mo.byp_phase == BYPP_PASS && !(a.evt_flags & EVT_STOP),
				"black while turning out or passing (the line, the obstacle base) is ignored");
			bypass_legs(&mo, &sq, 3, legs, 8);	/* past, turn in, return diagonal */
			CHECK_T("S5-H18b", mo.byp_phase == BYPP_ALIGN_TURN && mo.byp_result == BYPR_NO_LINE &&
				legs[2].type == NAV_TURN &&
				legs[2].arg == -BYP_TURN_CMD_DEG,
				"return diagonal ran its full length without the line: turn back (left side) anyway");
			/* passed on the left: the right sensor is the near one */
			mission_init(&mo); mo.track_enabled = 0; mo.state = SYS_OBSTACLE_SCAN;
			a = clear_act(); oo.kind = OBS_RESULT; oo.side = OBS_SIDE_LEFT; oo.width_mm = 80;
			mission_on_obs(&mo, &oo, &a);
			bypass_legs(&mo, &sq, 5, legs, 8);	/* now on the return diagonal */
			so.flags = STEER_F_DARK_R; a = clear_act(); mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H18d", !(a.evt_flags & EVT_STOP) && mo.byp_phase == BYPP_IN_DIAG,
				"passed on the left: right sensor on the line first, keep going");
			so.flags = STEER_F_DARK_C; a = clear_act(); mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H18e", (a.evt_flags & EVT_STOP) && mo.byp_phase == BYPP_STOPPING,
				"centre (left) sensor on the line: stop, then turn back right");
			a = clear_act(); mission_on_tick(&mo, 5, MOT_STOPPED, sq, &a);
			CHECK_T("S5-H18f", a.n_nav == 1 && a.nav[0].type == NAV_MOVE && a.nav[0].arg == BYP_PAST_LINE_MM,
				"left pass: 5 cm past the line first");
			bypass_legs(&mo, &sq, 1, legs, 8);
			CHECK_T("S5-H18g", bypass_last.n_nav == 1 && bypass_last.nav[0].type == NAV_TURN &&
				bypass_last.nav[0].arg == -BYP_TURN_CMD_DEG,
				"then 45 left (came back heading right) to face along the line");
			/* whichever sensor meets the line first is ignored; the same
			   one again does not count (2026-11-18: the right pass turned
			   on the centre sensor) */
			mission_init(&mo); mo.track_enabled = 0; mo.state = SYS_OBSTACLE_SCAN;
			a = clear_act(); oo.kind = OBS_RESULT; oo.side = OBS_SIDE_RIGHT; oo.width_mm = 80;
			mission_on_obs(&mo, &oo, &a);
			bypass_legs(&mo, &sq, 5, legs, 8);
			so.flags = STEER_F_DARK_R; a = clear_act(); mission_on_steer(&mo, &so, &a);
			so.flags = 0; mission_on_steer(&mo, &so, &a);
			so.flags = STEER_F_DARK_R; mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H18h", !(a.evt_flags & EVT_STOP) && mo.byp_phase == BYPP_IN_DIAG,
				"the first sensor seeing black again is not the second");
			so.flags = STEER_F_DARK_C; a = clear_act(); mission_on_steer(&mo, &so, &a);
			CHECK_T("S5-H18i", (a.evt_flags & EVT_STOP) && mo.byp_phase == BYPP_STOPPING,
				"right pass, right sensor first: stops on the centre one (order, not side)");
			mission_init(&mo); mo.track_enabled = 0; mo.state = SYS_LINE_FOLLOW;
			a = clear_act(); oo.kind = OBS_DETECTED; oo.dist_mm = 200; mission_on_obs(&mo, &oo, &a);
			a = clear_act(); oo.kind = OBS_CLEAR; mission_on_obs(&mo, &oo, &a);
			CHECK_T("S5-H18c", mo.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_RESUME) &&
				a.scan_cmd == SCANCMD_ARM, "obstacle gone during the sweep: drive on");
		}
		/* every timed state leaves by itself */
		mission_init(&m); m.state = SYS_OBSTACLE_SCAN; a = clear_act();
		mission_on_tick(&m, MS_OBSTACLE_SCAN, MOT_DRIVING, 0, &a);
		CHECK_T("S5-H01", m.state == SYS_FAULT_RECOVERY, "OBSTACLE_SCAN times out");

		{
			/* far left, near left, centre, near right, far right (45 / 22.5 deg) */
			INT d[SCAN_ANGLES] = { 1200, 900, 250, 700, 800 };

			CHECK_T("S5-U10a", scan_evaluate(d, &r) && r.width_mm < 300, "narrow obstacle profiled");
			CHECK_T("S5-U10b", r.clear_left_mm > r.clear_right_mm ? r.side == OBS_SIDE_LEFT : r.side == OBS_SIDE_RIGHT,
				"side with more room chosen");
			d[0] = 354; d[1] = 271; d[2] = 250; d[3] = 271; d[4] = 354;	/* flat wall */
			CHECK_T("S5-U10c", !scan_evaluate(d, &r), "wall wider than sweep -> fault");
			d[2] = 0;
			CHECK_T("S5-U10d", !scan_evaluate(d, &r), "no centre echo -> fault");
			{
				/* box 250 mm ahead reaching past the far left beam (360 <= 354 + tol) */
				INT w[SCAN_ANGLES] = { 360, 280, 250, 1500, 1500 };

				CHECK_T("S5-H02", scan_evaluate(w, &r) && r.side == OBS_SIDE_RIGHT && r.clear_left_mm == 0,
					"box extends left -> go right");
				CHECK_T("S5-H11", scan_pulse_us(SCAN_FAR_LEFT) == SCAN_US_FAR_LEFT &&
					scan_pulse_us(SCAN_NEAR_LEFT) == SCAN_US_NEAR_LEFT &&
					scan_pulse_us(SCAN_CENTRE) == SCAN_US_CENTRE &&
					scan_pulse_us(SCAN_NEAR_RIGHT) == SCAN_US_NEAR_RIGHT &&
					scan_pulse_us(SCAN_FAR_RIGHT) == SCAN_US_FAR_RIGHT &&
					SCAN_US_FAR_LEFT > SCAN_US_NEAR_LEFT && SCAN_US_NEAR_LEFT > SCAN_US_CENTRE &&
					SCAN_US_CENTRE > SCAN_US_NEAR_RIGHT && SCAN_US_NEAR_RIGHT > SCAN_US_FAR_RIGHT,
					"5 servo positions use their pulse widths, in order, centre 1660 us");
				/* the edge between the near and far beams: half width between the two */
				{
					INT e[SCAN_ANGLES] = { 1200, 290, 250, 1200, 1200 };

					CHECK_T("S5-H14", scan_evaluate(e, &r) && r.side == OBS_SIDE_RIGHT &&
						r.clear_left_mm < r.clear_right_mm && r.width_mm > 51 * 2,
						"near beam still hits, far beam clear: wider on that side, go the other way");
				}
			}
		}

		/* ---- interrupt-driven echo capture (Tribunal 1, replaces the busy-wait) ---- */
		{
			ECHO e = {0};
			BOOL done;

			/* 1750 us pulse = 300 mm */
			echo_arm(&e);
			done = echo_edge(&e, 1000, ECHO_EDGE_RISE);
			CHECK_T("S5-H04a", !done && e.armed, "rising edge alone does not complete");
			done = echo_edge(&e, 2750, ECHO_EDGE_FALL);
			CHECK_T("S5-H04b", done && e.width_us == 1750 && echo_us_to_mm(e.width_us) == 300 && !e.armed,
				"falling edge completes: 1750 us -> 300 mm, disarmed");

			/* timer wrap between the edges */
			echo_arm(&e);
			echo_edge(&e, 0xFFFFFF00u, ECHO_EDGE_RISE);
			done = echo_edge(&e, 0x00000100u, ECHO_EDGE_FALL);
			CHECK_T("S5-H04c", done && e.width_us == 0x200, "timer wrap-around is harmless");

			/* a falling edge left over from the previous ping, before our rising edge */
			echo_arm(&e);
			done = echo_edge(&e, 500, ECHO_EDGE_FALL);
			CHECK_T("S5-H04d", !done && e.armed, "stale falling edge ignored");

			/* edges after a timeout (disarmed) are ignored */
			echo_arm(&e);
			echo_disarm(&e);
			echo_edge(&e, 100, ECHO_EDGE_RISE);
			done = echo_edge(&e, 900, ECHO_EDGE_FALL);
			CHECK_T("S5-H04e", !done && !e.have_rise, "edges after a timeout are ignored");

			/* conversion limits: no pulse, and a pulse longer than ~2 m, are 'no echo' */
			CHECK_T("S5-H04f", echo_us_to_mm(0) == SCAN_NO_ECHO && echo_us_to_mm(ECHO_MAX_US + 1) == SCAN_NO_ECHO &&
				echo_us_to_mm(ECHO_MAX_US) > 0 && echo_us_to_mm(583) == 99 &&
				echo_us_to_mm(152) == SCAN_NO_ECHO && echo_us_to_mm(234) == 40,
				"0 or > 12 ms = no echo; 583 us = 99 mm");
		}
	}

	printf("\nhost tests: %d passed, %d failed\n", pass_, fail_);
	return fail_ ? 1 : 0;
}
