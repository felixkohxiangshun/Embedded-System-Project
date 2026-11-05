#include "dashboard.h"

EXPORT void dash_init(DASH *d)
{
	d->state = DASH_WIFI_CONNECTING;
	d->t_ms = 0;
	d->since_telem_ms = 0;
	d->since_hb_ms = 0;
}

LOCAL void go(DASH *d, UB state)
{
	d->state = state;
	d->t_ms = 0;
	d->since_telem_ms = 0;
	d->since_hb_ms = 0;
}

EXPORT void dash_step(DASH *d, const DASH_IN *in, DASH_OUT *out)
{
	out->publish_telem = FALSE;
	out->publish_heartbeat = FALSE;
	d->t_ms += in->dt_ms;

	if (d->state == DASH_MQTT_CONNECTING || d->state == DASH_ONLINE ||
	    d->state == DASH_HEARTBEAT) {
		if (!in->wifi_up || in->mqtt_event == MQTTM_DISCONNECTED) {
			go(d, DASH_RECONNECTING);
			return;
		}
	}

	switch (d->state) {
	case DASH_WIFI_CONNECTING:
		if (in->wifi_up) {
			go(d, DASH_MQTT_CONNECTING);
		}
		break;

	case DASH_MQTT_CONNECTING:
		if (in->mqtt_event == MQTTM_CONNECTED) {
			go(d, DASH_ONLINE);
		}
		break;

	case DASH_ONLINE:
		d->since_telem_ms += in->dt_ms;
		if (in->got_telem) {
			d->since_telem_ms = 0;
			out->publish_telem = TRUE;
		} else if (d->since_telem_ms >= DASH_HEARTBEAT_AFTER_MS) {
			go(d, DASH_HEARTBEAT);
		}
		break;

	case DASH_HEARTBEAT:
		d->since_hb_ms += in->dt_ms;
		if (in->got_telem) {
			go(d, DASH_ONLINE);
			out->publish_telem = TRUE;
		} else if (d->since_hb_ms >= DASH_HEARTBEAT_EVERY_MS) {
			d->since_hb_ms = 0;
			out->publish_heartbeat = TRUE;
		}
		break;

	case DASH_RECONNECTING:
		if (d->t_ms >= DASH_RECONNECT_MS) {
			go(d, DASH_WIFI_CONNECTING);
		}
		break;

	default:
		go(d, DASH_WIFI_CONNECTING);
		break;
	}
}

typedef struct {
	char	*p;
	INT	size;
	INT	n;
	BOOL	over;
} JB;

LOCAL void jb_ch(JB *j, char c)
{
	if (j->n + 1 < j->size) {
		j->p[j->n] = c;
	} else {
		j->over = TRUE;
	}
	j->n++;
}

LOCAL void jb_str(JB *j, const char *s)
{
	while (*s) {
		jb_ch(j, *s++);
	}
}

LOCAL void jb_int(JB *j, INT v)
{
	char tmp[12];
	INT  i = 0;
	UW   u;

	if (v < 0) {
		jb_ch(j, '-');
		u = (UW)(-(v + 1)) + 1u;
	} else {
		u = (UW)v;
	}
	do {
		tmp[i++] = (char)('0' + (u % 10));
		u /= 10;
	} while (u);
	while (i) {
		jb_ch(j, tmp[--i]);
	}
}

LOCAL void jb_field(JB *j, const char *name, INT v, BOOL first)
{
	if (!first) {
		jb_ch(j, ',');
	}
	jb_ch(j, '"');
	jb_str(j, name);
	jb_str(j, "\":");
	jb_int(j, v);
}

LOCAL INT jb_end(JB *j)
{
	jb_ch(j, '}');
	if (j->over) {
		return -1;
	}
	j->p[j->n] = '\0';
	return j->n;
}

LOCAL const char *state_name(UB s)
{
	switch (s) {
	case SYS_INIT:            return "INIT";
	case SYS_IDLE:            return "IDLE";
	case SYS_LINE_FOLLOW:     return "LINE_FOLLOW";
	case SYS_BARCODE_SCAN:    return "BARCODE_SCAN";
	case SYS_BARCODE_CMD:     return "BARCODE_CMD";
	case SYS_HUMP_TRAVERSE:   return "HUMP_TRAVERSE";
	case SYS_OBSTACLE_SCAN:   return "OBSTACLE_SCAN";
	case SYS_OBSTACLE_BYPASS: return "OBSTACLE_BYPASS";
	case SYS_LINE_REACQUIRE:  return "LINE_REACQUIRE";
	case SYS_FAULT_RECOVERY:  return "FAULT_RECOVERY";
	case SYS_E_STOP:          return "E_STOP";
	default:                  return "UNKNOWN";
	}
}

EXPORT INT dash_format_telem(const MSG_TELEM *t, char *buf, INT size)
{
	JB j = { buf, size, 0, FALSE };
	char bc[2] = { 0, 0 };

	if (size < 2) {
		return -1;
	}
	bc[0] = t->last_barcode ? (char)t->last_barcode : '-';
	jb_ch(&j, '{');
	jb_str(&j, "\"state\":\"");
	jb_str(&j, state_name(t->sys_state));
	jb_ch(&j, '"');
	jb_field(&j, "speed_mmps", t->speed_mmps, FALSE);
	jb_field(&j, "dist_mm", (INT)t->dist_mm, FALSE);
	jb_field(&j, "enc_l", t->enc_left, FALSE);
	jb_field(&j, "enc_r", t->enc_right, FALSE);
	jb_field(&j, "heading", t->heading_deg, FALSE);
	jb_field(&j, "line", t->ir_line, FALSE);
	jb_field(&j, "obs_mm", t->obs_dist_mm, FALSE);
	jb_str(&j, ",\"barcode\":\"");
	jb_str(&j, bc);
	jb_ch(&j, '"');
	jb_field(&j, "hump_max_mm", t->hump_max_mm, FALSE);
	jb_field(&j, "shock", t->shock, FALSE);
	jb_field(&j, "faults", t->fault_flags, FALSE);
	jb_field(&j, "t_ms", (INT)t->uptime_ms, FALSE);
	return jb_end(&j);
}

EXPORT INT dash_format_heartbeat(UW uptime_ms, UB sys_state, char *buf, INT size)
{
	JB j = { buf, size, 0, FALSE };

	if (size < 2) {
		return -1;
	}
	jb_ch(&j, '{');
	jb_str(&j, "\"heartbeat\":1,\"state\":\"");
	jb_str(&j, state_name(sys_state));
	jb_ch(&j, '"');
	jb_field(&j, "t_ms", (INT)uptime_ms, FALSE);
	return jb_end(&j);
}

LOCAL BOOL contains(const char *s, INT len, const char *word)
{
	INT i, k;

	for (i = 0; i < len; i++) {
		for (k = 0; word[k] && i + k < len && s[i + k] == word[k]; k++) {
			;
		}
		if (word[k] == '\0') {
			return TRUE;
		}
	}
	return FALSE;
}

EXPORT UB dash_parse_cmd(const char *text, INT len)
{
	if (contains(text, len, "E_STOP")) {
		return MQTTCMD_ESTOP;
	}
	if (contains(text, len, "RESUME")) {
		return MQTTCMD_RESUME;
	}
	return 0;
}
