#include "dashboard.h"
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT volatile UB s4_state;
EXPORT volatile UW s4_frames;
EXPORT volatile UW s4_console_lines;

#define DASH_TICK_MS	100

#if APP_TEST && TM_WIFI_CYW43 && TM_NET_LWIP

LOCAL void report_wifi_test_status(void)
{
	T_CYW43_UTK_STATUS st;
	UW ip;
	cyw43_utk_get_status(&st);
	if (st.link_status == 3 && st.ip_addr != 0) {
		ip = st.ip_addr;
		tm_printf((UB*)"[wifi-test] UP ip=%d.%d.%d.%d\n",
			(INT)(ip & 0xff), (INT)((ip >> 8) & 0xff),
			(INT)((ip >> 16) & 0xff), (INT)((ip >> 24) & 0xff));
	} else {
		tm_printf((UB*)"[wifi-test] NOT_UP link=%d ready=%d\n",
			(INT)st.link_status, (INT)st.ready);
	}
}
#endif

LOCAL BOOL wifi_is_up(void)
{
#if TM_WIFI_CYW43
	T_CYW43_UTK_STATUS st;

	cyw43_utk_get_status(&st);
	return (st.ready && st.link_status > 0 && st.ip_addr != 0) ? TRUE : FALSE;
#else
	return FALSE;
#endif
}

LOCAL void send_out(const char *json, INT len)
{
	static MQTT_MSG m;
	INT i;

	m.kind = MQTTM_PUBLISH;
	m.len = (UH)len;
	for (i = 0; i < len; i++) {
		m.data[i] = json[i];
	}
	MBF_SEND(mbf_mqtt_out, m);
}

EXPORT void dashboard_task(INT stacd, void *exinf)
{
	static MQTT_MSG in;
	static char     json[DASH_JSON_MAX];
	DASH            d;
	DASH_IN         din;
	DASH_OUT        dout;
	MSG_TELEM       t, latest;
	MSG_MQTTCMD     c;
	SYSTIM          now;
	INT             len;
	UB              cmd;
#if APP_TEST && TM_WIFI_CYW43 && TM_NET_LWIP
	INT             wifi_report_ms = 0;
#endif

	(void)stacd; (void)exinf;
	dash_init(&d);
	latest.sys_state = SYS_INIT;

	while (1) {
		tk_dly_tsk(DASH_TICK_MS);
#if APP_TEST && TM_WIFI_CYW43 && TM_NET_LWIP
		wifi_report_ms += DASH_TICK_MS;
		if (wifi_report_ms >= 5000) {
			wifi_report_ms = 0;
			report_wifi_test_status();
		}
#endif

		din.dt_ms = DASH_TICK_MS;
		din.wifi_up = wifi_is_up();
		din.mqtt_event = 0;
		din.got_telem = FALSE;

		while (MBF_RECV(mbf_mqtt_in, in, TMO_POL) > 0) {
			if (in.kind == MQTTM_CONNECTED || in.kind == MQTTM_DISCONNECTED) {
				din.mqtt_event = in.kind;
			} else if (in.kind == MQTTM_COMMAND) {
				cmd = dash_parse_cmd(in.data, in.len);
				if (cmd) {
					c.cmd = cmd;
					MBF_SEND(mbf_mqttcmd, c);
				}
			}
		}

		while (MBF_RECV(mbf_telem, t, TMO_POL) > 0) {
			latest = t;
			din.got_telem = TRUE;
		}

		dash_step(&d, &din, &dout);
		s4_state = d.state;
		if (state_lock(TMO_POL) == E_OK) {
			app_state.net_online = (d.state == DASH_ONLINE || d.state == DASH_HEARTBEAT) ? 1 : 0;
			state_unlock();
		}

		if (din.got_telem) {
			len = dash_format_telem(&latest, json, (INT)sizeof(json));
			if (len > 0) {
				s4_frames++;
				if (dout.publish_telem) {
					send_out(json, len);
				} else {

					tm_printf((UB*)"[telemetry] %s\n", json);
					s4_console_lines++;
				}
			}
		}
		if (dout.publish_heartbeat) {
			tk_get_otm(&now);
			len = dash_format_heartbeat(now.lo, latest.sys_state,
						    json, (INT)sizeof(json));
			if (len > 0) {
				send_out(json, len);
			}
		}
	}
}
