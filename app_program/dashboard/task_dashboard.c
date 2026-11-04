/*
 *	task_dashboard.c
 *	Dashboard -- the Dashboard / WiFi task (design review 6.5).
 *
 *	States  WIFI_CONNECTING, MQTT_CONNECTING, ONLINE, HEARTBEAT, RECONNECTING
 *	        (the logic is in dash_logic.c)
 *
 *	Every 100 ms: read the WiFi state, drain mbf_mqtt_in (broker events and
 *	commands), take the newest telemetry frame from mbf_telem, step the
 *	statechart, and send what it asks for to mbf_mqtt_out.  Dashboard
 *	commands (E_STOP, RESUME) are forwarded to Mission on mbf_mqttcmd.
 *
 *	It never calls cyw43/lwIP/MQTT functions.  A slow or failed network can
 *	therefore never delay driving: this is the lowest-priority task.
 */

#include "dashboard.h"
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT volatile UB s4_state;
EXPORT volatile UW s4_frames;
EXPORT volatile UW s4_console_lines;

#define DASH_TICK_MS	100

/* 1 prints each telemetry frame on the console while offline.  0 keeps the
   console free for bench tests. */
#ifndef DASH_CONSOLE_TELEM
#define DASH_CONSOLE_TELEM	0
#endif

LOCAL BOOL wifi_is_up(void)
{
#if TM_WIFI_CYW43
	T_CYW43_UTK_STATUS st;

	cyw43_utk_get_status(&st);
	return (st.ready && st.link_status > 0 && st.ip_addr != 0) ? TRUE : FALSE;
#else
	return FALSE;			/* built with WIFI=none */
#endif
}

LOCAL void send_out(const char *json, INT len)
{
	static MQTT_MSG m;		/* too big for a 2 KB task stack */
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

	(void)stacd; (void)exinf;
	dash_init(&d);
	latest.sys_state = SYS_INIT;

	while (1) {
		tk_dly_tsk(DASH_TICK_MS);

		din.dt_ms = DASH_TICK_MS;
		din.wifi_up = wifi_is_up();
		din.mqtt_event = 0;
		din.got_telem = FALSE;

		/* events and commands from the MQTT service task */
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

		/* keep only the newest telemetry frame */
		while (MBF_RECV(mbf_telem, t, TMO_POL) > 0) {
			latest = t;
			din.got_telem = TRUE;
		}

		dash_step(&d, &din, &dout);
		s4_state = d.state;
		if (state_lock(TMO_POL) == E_OK) {	/* tell Mission whether we are online */
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
					/* not online yet: show the frame on the console */
#if DASH_CONSOLE_TELEM
					tm_printf((UB*)"[telemetry] %s\n", json);
					s4_console_lines++;
#endif
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
