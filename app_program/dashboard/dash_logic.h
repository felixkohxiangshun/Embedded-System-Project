#ifndef DASH_LOGIC_H
#define DASH_LOGIC_H

#define DASH_WIFI_CONNECTING	0
#define DASH_MQTT_CONNECTING	1
#define DASH_ONLINE		2
#define DASH_HEARTBEAT		3
#define DASH_RECONNECTING	4

#define DASH_HEARTBEAT_AFTER_MS	1500
#define DASH_HEARTBEAT_EVERY_MS	1000
#define DASH_RECONNECT_MS	2000

#define DASH_JSON_MAX		512

typedef struct {
	UB	state;
	INT	t_ms;
	INT	since_telem_ms;
	INT	since_hb_ms;
} DASH;

typedef struct {
	BOOL	wifi_up;
	UB	mqtt_event;
	BOOL	got_telem;
	INT	dt_ms;
} DASH_IN;

typedef struct {
	BOOL	publish_telem;
	BOOL	publish_heartbeat;
} DASH_OUT;

IMPORT void dash_init(DASH *d);
IMPORT void dash_step(DASH *d, const DASH_IN *in, DASH_OUT *out);

IMPORT INT  dash_format_telem(const MSG_TELEM *t, char *buf, INT size);
IMPORT INT  dash_format_heartbeat(UW uptime_ms, UB sys_state, char *buf, INT size);

IMPORT UB   dash_parse_cmd(const char *text, INT len);

#endif
