/*
 *	dashboard_test.c
 *	On-target tests for the Dashboard subsystem (Dashboard).
 *	Built only with: make TEST=1.  Results: testing/dashboard/TEST_LOG.md
 */

#include "dashboard.h"
#include "../test_report.h"

#if APP_TEST

/* Does s contain sub? */
LOCAL BOOL has(const char *s, const char *sub)
{
	INT i, k;

	for (i = 0; s[i]; i++) {
		for (k = 0; sub[k] && s[i + k] == sub[k]; k++) {
			;
		}
		if (sub[k] == '\0') {
			return TRUE;
		}
	}
	return FALSE;
}

LOCAL DASH_OUT step(DASH *d, BOOL wifi, UB ev, BOOL telem, INT dt)
{
	DASH_IN  in;
	DASH_OUT out;

	in.wifi_up = wifi;
	in.mqtt_event = ev;
	in.got_telem = telem;
	in.dt_ms = dt;
	dash_step(d, &in, &out);
	return out;
}

EXPORT void dashboard_test(void)
{
	MSG_TELEM t = {0};
	char      buf[DASH_JSON_MAX];
	DASH      d;
	DASH_OUT  o;
	UW        before;
	INT       len;

	TEST_INIT();

	/* S4-U01: a normal frame is valid JSON-looking text with the FR-16 fields */
	t.sys_state = SYS_LINE_FOLLOW;
	t.speed_mmps = 120;
	t.last_barcode = 'B';
	len = dash_format_telem(&t, buf, (INT)sizeof(buf));
	TEST_CHECK("S4-U01", len > 0 && buf[0] == '{' && buf[len - 1] == '}' &&
		   has(buf, "\"state\":\"LINE_FOLLOW\"") && has(buf, "\"barcode\":\"B\"") &&
		   has(buf, "\"speed_mmps\":120"), "telemetry frame has the expected fields");

	/* S4-U02: the largest possible frame still fits in 512 bytes (NFR-13) */
	t.speed_mmps = -2147483647 - 1;
	t.dist_mm = 0xFFFFFFFF;
	t.enc_left = -2147483647 - 1;
	t.enc_right = 2147483647;
	t.heading_deg = -2147483647 - 1;
	t.obs_dist_mm = 2147483647;
	t.hump_max_mm = -2147483647 - 1;
	t.uptime_ms = 0xFFFFFFFF;
	len = dash_format_telem(&t, buf, (INT)sizeof(buf));
	TEST_CHECK("S4-U02", len > 0 && len <= DASH_JSON_MAX, "worst-case frame fits in 512 bytes");

	/* S4-U03: a too-small buffer is refused, not overrun */
	TEST_CHECK("S4-U03", dash_format_telem(&t, buf, 20) == -1, "overflow is detected");

	/* S4-U04: dashboard commands */
	TEST_CHECK("S4-U04", dash_parse_cmd("E_STOP", 6) == MQTTCMD_ESTOP &&
		   dash_parse_cmd("{\"cmd\":\"RESUME\"}", 16) == MQTTCMD_RESUME &&
		   dash_parse_cmd("hello", 5) == 0, "E_STOP / RESUME parsed, junk ignored");

	/* S4-U05: connection statechart, section 6.5 */
	dash_init(&d);
	step(&d, FALSE, 0, FALSE, 100);
	TEST_CHECK("S4-U05a", d.state == DASH_WIFI_CONNECTING, "stays in WIFI_CONNECTING without WiFi");
	step(&d, TRUE, 0, FALSE, 100);
	TEST_CHECK("S4-U05b", d.state == DASH_MQTT_CONNECTING, "WiFi up -> MQTT_CONNECTING");
	step(&d, TRUE, MQTTM_CONNECTED, FALSE, 100);
	TEST_CHECK("S4-U05c", d.state == DASH_ONLINE, "broker connected -> ONLINE");
	o = step(&d, TRUE, 0, TRUE, 100);
	TEST_CHECK("S4-U05d", o.publish_telem, "ONLINE publishes a new frame");
	step(&d, TRUE, 0, FALSE, DASH_HEARTBEAT_AFTER_MS);
	TEST_CHECK("S4-U05e", d.state == DASH_HEARTBEAT, "no telemetry -> HEARTBEAT");
	o = step(&d, TRUE, 0, FALSE, DASH_HEARTBEAT_EVERY_MS);
	TEST_CHECK("S4-U05f", o.publish_heartbeat, "HEARTBEAT publishes a heartbeat (FR-18)");
	step(&d, TRUE, 0, TRUE, 100);
	TEST_CHECK("S4-U05g", d.state == DASH_ONLINE, "telemetry again -> ONLINE");
	step(&d, FALSE, 0, FALSE, 100);
	TEST_CHECK("S4-U05h", d.state == DASH_RECONNECTING, "link lost -> RECONNECTING");
	step(&d, FALSE, 0, FALSE, DASH_RECONNECT_MS);
	TEST_CHECK("S4-U05i", d.state == DASH_WIFI_CONNECTING, "after the pause -> WIFI_CONNECTING");

	/* S4-I01: the Telemetry task sends a frame every 500 ms (FR-16, IF-09 source) */
	before = s4_telem_sent;
	tk_dly_tsk(1200);
	TEST_CHECK("S4-I01", s4_telem_sent >= before + 2, "Telemetry sends a frame every 500 ms");

	/* INT-02 (IF-09): the Telemetry task's frames reach the Dashboard */
	before = s4_frames;
	tk_dly_tsk(1300);
	TEST_CHECK("INT-02", s4_frames >= before + 2, "telemetry frames flow from Telemetry to Dashboard");

	TEST_SUMMARY("dashboard");
}

#endif /* APP_TEST */
