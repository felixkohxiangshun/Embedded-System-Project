/*
 *	dash_logic.h
 *	Dashboard -- pure dashboard logic: the connection statechart, the
 *	telemetry JSON and the command parser.  No kernel calls, no hardware, so
 *	it is unit-tested on a PC as well as on the target.
 */

#ifndef DASH_LOGIC_H
#define DASH_LOGIC_H

/* Connection statechart (design review 6.5) */
#define DASH_WIFI_CONNECTING	0
#define DASH_MQTT_CONNECTING	1
#define DASH_ONLINE		2
#define DASH_HEARTBEAT		3
#define DASH_RECONNECTING	4

#define DASH_HEARTBEAT_AFTER_MS	1500	/* no telemetry this long -> HEARTBEAT */
#define DASH_HEARTBEAT_EVERY_MS	1000	/* FR-18 */
#define DASH_RECONNECT_MS	2000

#define DASH_JSON_MAX		512	/* NFR-13: a message never exceeds 512 bytes */

typedef struct {
	UB	state;
	INT	t_ms;			/* time in this state */
	INT	since_telem_ms;		/* time since the last telemetry frame */
	INT	since_hb_ms;
} DASH;

/* Inputs for one step.  mqtt_event is 0 or one of the MQTTM_ values. */
typedef struct {
	BOOL	wifi_up;
	UB	mqtt_event;		/* MQTTM_CONNECTED / MQTTM_DISCONNECTED / 0 */
	BOOL	got_telem;		/* a telemetry frame arrived this step */
	INT	dt_ms;
} DASH_IN;

/* What to do after a step */
typedef struct {
	BOOL	publish_telem;		/* publish the frame just received */
	BOOL	publish_heartbeat;
} DASH_OUT;

IMPORT void dash_init(DASH *d);
IMPORT void dash_step(DASH *d, const DASH_IN *in, DASH_OUT *out);

/* Format into buf (size bytes).  Return the length, or -1 if it would not
   fit.  The output is a single JSON object, no trailing newline. */
IMPORT INT  dash_format_telem(const MSG_TELEM *t, char *buf, INT size);
IMPORT INT  dash_format_heartbeat(UW uptime_ms, UB sys_state, char *buf, INT size);

/* Command text from the dashboard -> MQTTCMD_ESTOP / MQTTCMD_RESUME / 0 */
IMPORT UB   dash_parse_cmd(const char *text, INT len);

#endif /* DASH_LOGIC_H */
