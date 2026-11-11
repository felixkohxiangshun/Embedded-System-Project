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

static MISSION_ACT clear_act(void)
{
	MISSION_ACT a;

	mission_act_clear(&a);
	return a;
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
		IMU_LOGIC l;
		int i, ev;

		imu_logic_init(&l);
		for (i = 0; i < IMU_CAL_SAMPLES; i++) { imu_feed(&l, 1000); }
		CHECK_T("S3-U01", l.state == IMU_MONITORING && l.baseline_mg == 1000, "baseline measured");
		ev = imu_feed(&l, 1400);
		CHECK_T("S3-U02a", ev == IMU_EV_HUMP_START, "hump starts");
		imu_feed(&l, 1600);
		ev = imu_feed(&l, 1010);
		CHECK_T("S3-U02b", ev == IMU_EV_HUMP_DONE && l.peak_mg == 600 &&
			l.height_mm == (600 * HUMP_MM_PER_G) / 1000, "hump height from the peak");
		imu_feed(&l, 1000); imu_feed(&l, 1200); imu_feed(&l, 1005);
		CHECK_T("S3-U03", l.max_height_mm == (600 * HUMP_MM_PER_G) / 1000, "tallest hump kept");
		imu_logic_init(&l);
		for (i = 0; i < IMU_CAL_SAMPLES; i++) { imu_feed(&l, 1000); }
		imu_feed(&l, 1000 + SHOCK_MG + 500); imu_feed(&l, 1000 + SHOCK_MG + 500);
		CHECK_T("S3-U04", imu_feed(&l, 1000) == IMU_EV_SHOCK, "2-sample spike = SHOCK");
		imu_logic_init(&l);
		for (i = 0; i < IMU_CAL_SAMPLES; i++) { imu_feed(&l, 1000); }
		for (i = 0; i < 5; i++) { imu_feed(&l, 1000 + SHOCK_MG + 500); }
		CHECK_T("S3-U05", imu_feed(&l, 1000) != IMU_EV_SHOCK, "5-sample reading is not a SHOCK");
		imu_fault(&l);
		CHECK_T("S3-U06a", l.state == IMU_SENSOR_FAULT && imu_feed(&l, 5000) == IMU_EV_NONE, "fault ignores samples");
		imu_recovered(&l);
		CHECK_T("S3-U06b", l.state == IMU_CALIBRATING, "recovery recalibrates");
	}

	/* ---------------- subsystem 4: binary frames (TEAM.md 7) ---------------- */
	{
		MSG_TELEM t;
		DASH_DEBUG g;
		UB f[DASH_DEBUG_BYTES];
		uint32_t cur = 0, v[22];
		static const uint32_t tmax[12] = { 255, 15, 511, 65535, 359, 1, 255, 31, 31, 127, 1, 255 };
		static const uint32_t gmax[22] = { 255, 1023, 1023, 1023, 2047, 3, 255, 255, 255, 31, 4095, 7, 15, 63,
						   7, 4095, 4095, 4095, 4095, 4095, 3, 7 };
		int i, ok = 1;

		memset(&t, 0, sizeof t);
		t.sys_state = SYS_LINE_FOLLOW; t.speed_mmps = -40; t.dist_mm = 70000; t.heading_deg = -90;
		t.ir_line = 1; t.obs_dist_mm = 3000; t.last_barcode = 'Z'; t.hump_max_mm = 200; t.shock = 1;
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
		cur = 0; ok = 1;
		CHECK_T("S4-U12a", dash_pack_debug(&g, 200, f, DASH_DEBUG_BYTES) == DASH_DEBUG_BYTES,
			"debug frame packs into 22 bytes");
		for (i = 0; i < 22; i++) {
			ok &= protocol_bits_get(f, DASH_DEBUG_BYTES, &cur, gmax[i], &v[i]);
		}
		CHECK_T("S4-U12b", ok && cur == 173 && v[21] == 3 && v[14] == 3 && v[15] == 1200 && v[16] == 900 &&
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
		CHECK_T("S4-U04", dash_parse_cmd("E_STOP", 6) == MQTTCMD_ESTOP &&
			dash_parse_cmd("{\"cmd\":\"RESUME\"}", 16) == MQTTCMD_RESUME &&
			dash_parse_cmd("hello", 5) == 0, "commands parsed");

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
		mission_init(&m); m.net_online = 1;
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
		/* S5-U20: 'F' stored -> right turn at the next junction, then drive on */
		{
			MISSION   mj;
			MSG_BCCMD bj;

			mission_init(&mj); mj.state = SYS_LINE_FOLLOW; a = clear_act();
			bj.kind = BC_OK; bj.cmd = 'F'; mission_on_barcode(&mj, &bj, &a);
			CHECK_T("S5-U20a", mj.state == SYS_LINE_FOLLOW && a.n_nav == 0 && mj.stored_char == 'F',
				"barcode F: stored, no turn yet, keeps following the line");
			s.steer = 0; s.flags = STEER_F_JUNCTION; mission_on_steer(&mj, &s, &a);
			a = clear_act(); mission_on_tick(&mj, JUNCTION_CREEP_MS, MOT_DRIVING, 0, &a);
			CHECK_T("S5-U20b", mj.state == SYS_BARCODE_CMD && a.n_nav == 1 && a.nav[0].type == NAV_TURN &&
				a.nav[0].arg == 90 && mj.stored_char == 0,
				"1.5 s after the junction: right turn sent, stored letter cleared");
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
			CHECK_T("S5-U21a", mission_junction_deg(&mj, 'A') == 0 && mission_junction_deg(&mj, 0) == 0 &&
				mission_junction_deg(&mj, 'F') == 90, "only F is mapped (right turn); others go forward");
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
			a = clear_act(); bi.kind = BC_OK; bi.cmd = 'F'; mission_on_barcode(&mi, &bi, &a);
			CHECK_T("S5-H12d", mi.state == SYS_LINE_FOLLOW && mi.stored_char == 'F' && a.n_nav == 0,
				"F decoded: stored for the junction, no turn yet");
			a = clear_act(); memset(&ti, 0, sizeof ti); ti.kind = TERR_HUMP_START; mission_on_terrain(&mi, &ti, &a);
			mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12e", mi.state == SYS_HUMP_TRAVERSE && a.nav[0].type == NAV_SLOW &&
				a.nav[1].type == NAV_STEER && a.nav[1].arg == 250, "IMU hump: slow down, still follow the line");
			a = clear_act(); mission_on_tick(&mi, 8000, MOT_DRIVING, 0, &a);
			CHECK_T("S5-H12f", mi.state == SYS_HUMP_TRAVERSE, "a slow 8 s hump crossing is not cut short");
			ti.kind = TERR_HUMP_DONE; ti.height_mm = 20; ti.max_mm = 22; mission_on_terrain(&mi, &ti, &a);
			CHECK_T("S5-H12g", mi.state == SYS_LINE_FOLLOW && a.nav[0].type == NAV_RESUME &&
				mi.hump_max_mm == 22 && mi.stored_char == 'F', "hump done: normal speed, height kept, letter kept");
			a = clear_act(); ti.kind = TERR_FAULT; mission_on_terrain(&mi, &ti, &a);
			mission_on_steer(&mi, &s, &a);
			CHECK_T("S5-H12h", mi.state == SYS_LINE_FOLLOW && (mi.fault_flags & FAULT_IMU) &&
				!(a.evt_flags & EVT_STOP) && a.nav[0].type == NAV_STEER, "IMU fault: reported, car keeps driving");
			a = clear_act(); s.flags = STEER_F_JUNCTION; mission_on_steer(&mi, &s, &a);
			mission_on_tick(&mi, JUNCTION_CREEP_MS, MOT_DRIVING, 0, &a);
			CHECK_T("S5-H12i", mi.state == SYS_BARCODE_CMD && a.nav[a.n_nav - 1].type == NAV_TURN &&
				a.nav[a.n_nav - 1].arg == 90 && mi.stored_char == 0, "cross junction: right turn for the stored F");
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
				a.nav[0].type == NAV_BYPASS && a.nav[0].arg == OBS_SIDE_LEFT, "bypass on the clear side");
			a = clear_act(); mission_on_tick(&ms, 5, MOT_BYPASS, 3, &a);
			mission_on_tick(&ms, 5, MOT_STOPPED, 4, &a);
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
		CHECK_T("S5-U06d", m.state == SYS_OBSTACLE_BYPASS && a.nav[0].type == NAV_BYPASS, "-> OBSTACLE_BYPASS");
		mission_on_tick(&m, 5, MOT_BYPASS, 7, &a); a = clear_act();
		mission_on_tick(&m, 5, MOT_STOPPED, 8, &a);
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
		a = clear_act(); q.cmd = MQTTCMD_ESTOP; mission_on_mqtt(&m, &q, &a);
		CHECK_T("S5-U08a", m.state == SYS_E_STOP, "E_STOP");
		mission_on_button(&m, &a);
		CHECK_T("S5-U08b", m.state == SYS_E_STOP, "button cannot clear E_STOP");
		q.cmd = MQTTCMD_RESUME; mission_on_mqtt(&m, &q, &a);
		CHECK_T("S5-U08c", m.state == SYS_LINE_FOLLOW, "RESUME");
		a = clear_act(); mission_on_tick(&m, 5, MOT_ENCODER_FAULT, 0, &a);
		CHECK_T("S5-U09", m.state == SYS_FAULT_RECOVERY, "encoder fault -> FAULT_RECOVERY");
		/* NFR-10: a slow bypass is recorded, never aborted (review 8.2) */
		mission_init(&m); m.state = SYS_OBSTACLE_BYPASS; m.done_seq_base = 1; a = clear_act();
		mission_on_tick(&m, 8000, MOT_BYPASS, 1, &a);
		CHECK_T("S5-H03a", m.state == SYS_OBSTACLE_BYPASS, "bypass over 5 s is not a fault");
		mission_on_tick(&m, 5, MOT_STOPPED, 2, &a);
		CHECK_T("S5-H03b", m.state == SYS_LINE_REACQUIRE && m.bypass_late, "slow bypass finishes, flagged late");
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
