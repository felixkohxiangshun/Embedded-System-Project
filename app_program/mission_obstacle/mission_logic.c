/*
 *	mission_logic.c
 *	Mission/obstacle -- the system statechart as pure logic.  See mission_logic.h.
 */

#include "mission_obstacle.h"

LOCAL BOOL is_running(UB s)
{
	return (s >= SYS_LINE_FOLLOW && s <= SYS_LINE_REACQUIRE) ? TRUE : FALSE;
}


/* States in which the Line task's steering is passed on to the Motor */
LOCAL BOOL forwards_steer(UB s)
{
	return (s == SYS_LINE_FOLLOW || s == SYS_BARCODE_SCAN ||
		s == SYS_HUMP_TRAVERSE) ? TRUE : FALSE;
}

LOCAL void nav(MISSION_ACT *a, UB type, INT arg, INT arg2)
{
	if (a->n_nav < MAX_NAV_ACTIONS) {
		a->nav[a->n_nav].type = type;
		a->nav[a->n_nav].arg = arg;
		a->nav[a->n_nav].arg2 = arg2;
		a->n_nav++;
	}
}

LOCAL void go(MISSION *m, UB state)
{
	m->state = state;
	m->t_ms = 0;
	m->wait_stop = 0;
	m->line_ok = 0;
	m->creeping = 0;
}

EXPORT void mission_act_clear(MISSION_ACT *a)
{
	a->evt_flags = 0;
	a->scan_cmd = 0;
	a->n_nav = 0;
}

EXPORT void mission_init(MISSION *m)
{
	m->done_seq_base = 0;
	m->recoveries = 0;
	m->last_barcode = 0;
	m->stored_char = 0;
	m->hump_mm = 0;
	m->hump_max_mm = 0;
	m->shock = 0;
	m->obs_dist_mm = 0;
	m->obs_width_mm = 0;
	m->obs_side = 0;
	m->fault_flags = 0;
	m->ir_line = 0;
	m->net_online = 0;
	m->bypass_ms = 0;
	m->bypass_late = 0;
	m->stop_at_barcode = 0;
	m->move_map[JMOVE_FORWARD] = 0;
	m->move_map[JMOVE_UTURN] = JMAP_DEFAULT_UTURN;
	m->move_map[JMOVE_LEFT] = 0;
	m->move_map[JMOVE_RIGHT] = 0;
	go(m, SYS_INIT);
}

EXPORT INT mission_junction_deg(const MISSION *m, UB ch)
{
	if (ch == 0) {
		return 0;			/* nothing stored: forward */
	}
	if (ch == m->move_map[JMOVE_UTURN]) {
		return 180;
	}
	if (ch == m->move_map[JMOVE_LEFT]) {
		return -90;
	}
	if (ch == m->move_map[JMOVE_RIGHT]) {
		return 90;
	}
	return 0;				/* forward, or not mapped */
}

/* Halt everything and stop pinging; used by IDLE, E_STOP, FAULT_RECOVERY */
LOCAL void halt(MISSION_ACT *a)
{
	a->evt_flags |= EVT_STOP;
	a->scan_cmd = SCANCMD_DISARM;
}

LOCAL void start_driving(MISSION_ACT *a)
{
	a->evt_flags |= EVT_RESUME;
	a->scan_cmd = SCANCMD_ARM;
	nav(a, NAV_RESUME, 0, 0);
	nav(a, NAV_STEER, 0, 0);
}

LOCAL void fault(MISSION *m, MISSION_ACT *a, UB flag)
{
	m->fault_flags |= flag;
	halt(a);
	go(m, SYS_FAULT_RECOVERY);
}

/* The start / stop button toggles between IDLE and RUNNING */
EXPORT void mission_on_button(MISSION *m, MISSION_ACT *a)
{
	if (m->state == SYS_IDLE) {
		a->evt_flags |= EVT_START;
		a->scan_cmd = SCANCMD_ARM;
		nav(a, NAV_RESUME, 0, 0);
		go(m, SYS_LINE_FOLLOW);
	} else if (is_running(m->state) || m->state == SYS_FAULT_RECOVERY) {
		halt(a);
		go(m, SYS_IDLE);
	}
	/* INIT and E_STOP ignore the button: E_STOP is cleared remotely */
}

EXPORT void mission_on_mqtt(MISSION *m, const MSG_MQTTCMD *c, MISSION_ACT *a)
{
	if (m->state == SYS_INIT) {
		return;
	}
	if (c->cmd == MQTTCMD_ESTOP) {
		halt(a);
		go(m, SYS_E_STOP);		/* telemetry keeps running */
	} else if (c->cmd == MQTTCMD_RESUME && m->state == SYS_E_STOP) {
		start_driving(a);
		go(m, SYS_LINE_FOLLOW);
	}
}

EXPORT void mission_on_steer(MISSION *m, const MSG_STEER *s, MISSION_ACT *a)
{
	m->ir_line = (s->flags & STEER_F_LOST) ? 0 : 1;

	if (forwards_steer(m->state)) {
		if ((s->flags & STEER_F_JUNCTION) && !m->creeping) {
			m->creeping = 1;
			m->creep_primed = 0;
		}
		if (m->creeping) {
			/* Straight on: the branch would pull the steering, and
			   a T has no line ahead, so ignore the line for now. */
			nav(a, NAV_STEER, 0, 0);
			return;
		}
		if (s->flags & STEER_F_TIMEOUT) {
			fault(m, a, FAULT_LINE);	/* line lost for too long */
			return;
		}
		nav(a, NAV_STEER, s->steer, 0);
	} else if (m->state == SYS_LINE_REACQUIRE) {
		if (s->flags & STEER_F_LOST) {
			m->line_ok = 0;
		} else if (++m->line_ok >= LINE_FOUND_TICKS) {
			nav(a, NAV_STEER, s->steer, 0);
			nav(a, NAV_RESUME, 0, 0);
			a->scan_cmd = SCANCMD_ARM;
			go(m, SYS_LINE_FOLLOW);
		} else {
			nav(a, NAV_STEER, s->steer, 0);
		}
	}
}

EXPORT void mission_on_distance(MISSION *m, UW dist_mm, MISSION_ACT *a)
{
	if (!m->creeping) {
		return;
	}
	if (!m->creep_primed) {
		m->creep_base_mm = dist_mm;
		m->creep_primed = 1;
	} else if ((UW)(dist_mm - m->creep_base_mm) >= (UW)JUNCTION_CREEP_MM) {
		/* The axle is on the junction: carry out the stored letter's move
		   and forget the letter (TEAM.md 7, steps 4 and 5). */
		INT deg = mission_junction_deg(m, m->stored_char);

		m->stored_char = 0;
		if (deg != 0) {
			nav(a, NAV_TURN, deg, 0);
			go(m, SYS_BARCODE_CMD);
			m->done_seq_base = SEQ_UNSET;	/* captured on the next tick */
		} else {
			m->creeping = 0;	/* forward: follow the line again */
		}
	}
}

EXPORT void mission_on_barcode(MISSION *m, const MSG_BCCMD *b, MISSION_ACT *a)
{
	/* Every letter read is recorded and kept for the next cross junction,
	   whatever the state: a code swiped by hand while IDLE counts too. */
	if (b->kind == BC_OK && b->cmd >= 'A' && b->cmd <= 'Z') {
		m->last_barcode = b->cmd;
		m->stored_char = b->cmd;
	}
	if (m->state != SYS_LINE_FOLLOW && m->state != SYS_BARCODE_SCAN) {
		return;				/* no action during hump, obstacle, ... */
	}
	if (m->stop_at_barcode && (b->kind == BC_OK || b->kind == BC_FAIL)) {
		halt(a);			/* barcode read (or failed): stop here */
		go(m, SYS_IDLE);
		return;
	}
	switch (b->kind) {
	case BC_SEEN:
		if (m->state == SYS_LINE_FOLLOW) {
			go(m, SYS_BARCODE_SCAN);
		}
		break;
	case BC_OK:
		/* Stored above; the move waits for the next cross junction. */
		go(m, SYS_LINE_FOLLOW);
		break;
	case BC_FAIL:
	default:
		go(m, SYS_LINE_FOLLOW);		/* no turn on a failed decode */
		break;
	}
}

EXPORT void mission_on_terrain(MISSION *m, const MSG_TERRAIN *t, MISSION_ACT *a)
{
	switch (t->kind) {
	case TERR_HUMP_START:
		if (m->state == SYS_LINE_FOLLOW) {
			nav(a, NAV_SLOW, 0, 0);
			go(m, SYS_HUMP_TRAVERSE);
		}
		break;
	case TERR_HUMP_DONE:
		m->hump_mm = t->height_mm;
		m->hump_max_mm = t->max_mm;
		if (m->state == SYS_HUMP_TRAVERSE) {
			nav(a, NAV_RESUME, 0, 0);
			go(m, SYS_LINE_FOLLOW);
		}
		break;
	case TERR_SHOCK:
		m->shock = 1;
		break;
	case TERR_FAULT:
		m->fault_flags |= FAULT_IMU;	/* FR-19: the car keeps driving */
		break;
	case TERR_RECOVERED:
		m->fault_flags &= (UB)~FAULT_IMU;
		break;
	default:
		break;
	}
}

EXPORT void mission_on_obs(MISSION *m, const MSG_OBS *o, MISSION_ACT *a)
{
	switch (o->kind) {
	case OBS_DETECTED:
		if (m->state == SYS_LINE_FOLLOW || m->state == SYS_BARCODE_SCAN ||
		    m->state == SYS_HUMP_TRAVERSE) {
			m->obs_dist_mm = o->dist_mm;
			a->evt_flags |= EVT_STOP;	/* stop first, then profile */
			go(m, SYS_OBSTACLE_SCAN);
			m->wait_stop = 1;
		}
		break;
	case OBS_RESULT:
		if (m->state == SYS_OBSTACLE_SCAN) {
			m->obs_side = o->side;
			m->obs_width_mm = o->width_mm;
			nav(a, NAV_BYPASS, o->side, o->width_mm);
			go(m, SYS_OBSTACLE_BYPASS);
			m->done_seq_base = SEQ_UNSET;	/* captured on the next tick */
		}
		break;
	case OBS_FAIL:
		if (m->state == SYS_OBSTACLE_SCAN) {
			fault(m, a, FAULT_SCAN);
		}
		break;
	default:
		break;
	}
}

EXPORT void mission_on_tick(MISSION *m, INT dt_ms, UB motor_state,
			    UW motor_done_seq, MISSION_ACT *a)
{
	m->t_ms += dt_ms;

	/* A dead encoder while driving is a fault: retry after a pause */
	if (motor_state == MOT_ENCODER_FAULT && is_running(m->state)) {
		fault(m, a, 0);
		return;
	}

	switch (m->state) {
	case SYS_INIT:
		/* design review 5: INIT connects WiFi and MQTT.  Wait for them, but
		   not for ever: the car must still be usable without a dashboard. */
		if ((m->t_ms >= MS_INIT && m->net_online) || m->t_ms >= MS_INIT_NET) {
			go(m, SYS_IDLE);
		}
		break;

	case SYS_BARCODE_SCAN:
		if (m->t_ms >= MS_BARCODE_SCAN) {
			go(m, SYS_LINE_FOLLOW);
		}
		break;

	case SYS_BARCODE_CMD:
		/* the turn is a Motor sequence; it ends in STOPPED, so drive on */
		if (m->done_seq_base == SEQ_UNSET) {
			m->done_seq_base = motor_done_seq;	/* remember the counter */
		} else if (motor_done_seq != m->done_seq_base) {
			start_driving(a);
			go(m, SYS_LINE_FOLLOW);
		} else if (m->t_ms >= MS_BARCODE_CMD) {
			fault(m, a, FAULT_LINE);
		}
		break;

	case SYS_HUMP_TRAVERSE:
		if (m->t_ms >= MS_HUMP) {
			nav(a, NAV_RESUME, 0, 0);
			go(m, SYS_LINE_FOLLOW);
		}
		break;

	case SYS_OBSTACLE_SCAN:
		if (m->wait_stop && motor_state == MOT_STOPPED) {
			a->scan_cmd = SCANCMD_PROFILE;	/* car is still: sweep now */
			m->wait_stop = 0;
		}
		if (m->t_ms >= MS_OBSTACLE_SCAN) {
			fault(m, a, FAULT_SCAN);
		}
		break;

	case SYS_OBSTACLE_BYPASS:
		if (m->done_seq_base == SEQ_UNSET) {
			m->done_seq_base = motor_done_seq;	/* remember the counter */
		} else if (motor_done_seq != m->done_seq_base) {
			/* bypass finished: creep forward until the line is found */
			m->bypass_ms = m->t_ms;
			m->bypass_late = (m->t_ms > BYPASS_TARGET_MS) ? 1 : 0;
			a->evt_flags |= EVT_RESUME;
			nav(a, NAV_SLOW, 0, 0);
			nav(a, NAV_STEER, 0, 0);
			go(m, SYS_LINE_REACQUIRE);
		}
		/* No timeout: NFR-10 (5 s) is a performance grade, not a safety
		   limit, and aborting half way would leave the car off the line
		   (design review 8.2).  The Motor sequence always ends, and a dead
		   encoder is caught separately above. */
		break;

	case SYS_LINE_REACQUIRE:
		if (m->t_ms >= MS_LINE_REACQUIRE) {
			fault(m, a, FAULT_LINE);
		}
		break;

	case SYS_FAULT_RECOVERY:
		if (m->t_ms >= MS_FAULT_RECOVERY) {
			m->recoveries++;
			/* design review 8.2: recovery returns to IDLE, never to
			   RUNNING, so the car cannot restart without BTN_START */
			m->fault_flags &= (UB)~(FAULT_LINE | FAULT_SCAN);
			go(m, SYS_IDLE);
		}
		break;

	default:				/* IDLE, E_STOP, LINE_FOLLOW */
		break;
	}
}
