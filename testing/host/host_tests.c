/*
 *	host_tests.c
 *	Unit tests for the pure logic files, run on a PC.  The same checks (and
 *	test IDs) as the on-target tests in each subsystem's _test.c file, so
 *	the logic can be proven before it is flashed.   cd testing/host && make run
 */

#include <string.h>
#include <stdio.h>
#include "motor_control/motor_control.h"
#include "imu_terrain/imu_terrain.h"
#include "dashboard/dashboard.h"
#include "mission_obstacle/mission_obstacle.h"

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
	CHECK_T("S1-U03", mm_to_counts(WHEEL_MM_PER_REV) == ENC_COUNTS_PER_REV &&
		counts_to_mmps(ENC_COUNTS_PER_REV / 10) == WHEEL_MM_PER_REV * 10, "unit conversions agree");
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
	CHECK_T("S1-H02", deg_to_counts(90) > 0 && deg_to_counts(180) == 2 * deg_to_counts(90),
		"turn distance scales with the angle");

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
		SCAN_RESULT r; BOOL valid; int i;

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
		s.steer = 321; mission_on_steer(&m, &s, &a);
		CHECK_T("S5-U02a", a.n_nav == 1 && a.nav[0].type == NAV_STEER && a.nav[0].arg == 321, "steering forwarded");
		b.kind = BC_SEEN; b.cmd = 0; mission_on_barcode(&m, &b, &a);
		CHECK_T("S5-U02b", m.state == SYS_BARCODE_SCAN, "-> BARCODE_SCAN");
		CHECK_T("S5-U03", mission_barcode_turn('A', &valid) == -90 && mission_barcode_turn('B', &valid) == 90 &&
			mission_barcode_turn('C', &valid) == 0 && mission_barcode_turn('D', &valid) == 180 &&
			(mission_barcode_turn('Z', &valid), !valid), "A/B/C/D commands");
		a = clear_act(); b.kind = BC_OK; b.cmd = 'B'; mission_on_barcode(&m, &b, &a);
		CHECK_T("S5-U03b", m.state == SYS_BARCODE_CMD && a.nav[0].type == NAV_TURN && a.nav[0].arg == 90, "turn right 90 sent");
		a = clear_act();
		mission_on_tick(&m, 5, MOT_TURNING, 4, &a);
		mission_on_tick(&m, 5, MOT_STOPPED, 5, &a);
		CHECK_T("S5-U03c", m.state == SYS_LINE_FOLLOW && (a.evt_flags & EVT_RESUME), "turn done -> LINE_FOLLOW");
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
			INT d[SCAN_ANGLES] = { 1200, 900, 250, 700, 800 };

			CHECK_T("S5-U10a", scan_evaluate(d, &r) && r.width_mm < 300, "narrow obstacle profiled");
			CHECK_T("S5-U10b", r.clear_left_mm > r.clear_right_mm ? r.side == OBS_SIDE_LEFT : r.side == OBS_SIDE_RIGHT,
				"side with more room chosen");
			d[0] = 500; d[1] = 289; d[2] = 250; d[3] = 289; d[4] = 500;
			CHECK_T("S5-U10c", !scan_evaluate(d, &r), "wall wider than sweep -> fault");
			d[2] = 0;
			CHECK_T("S5-U10d", !scan_evaluate(d, &r), "no centre echo -> fault");
			{
				/* 20 cm wide box 250 mm ahead: beams at +/-30 deg still hit (lat 144 mm > 100) */
				INT w[SCAN_ANGLES] = { 1200, 288, 250, 1500, 1500 };

				CHECK_T("S5-H02", scan_evaluate(w, &r) && r.side == OBS_SIDE_RIGHT, "box extends left -> go right");
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
				echo_us_to_mm(ECHO_MAX_US) > 0 && echo_us_to_mm(583) == 99,
				"0 or > 12 ms = no echo; 583 us = 99 mm");
		}
	}

	printf("\nhost tests: %d passed, %d failed\n", pass_, fail_);
	return fail_ ? 1 : 0;
}
