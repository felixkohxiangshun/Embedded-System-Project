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

LOCAL INT  track_steer(const MISSION *m);	/* tracking bypass, below */
LOCAL void enter_out(MISSION *m, MISSION_ACT *a);	/* tracking bypass, below */

LOCAL void go(MISSION *m, UB state)
{
	if (m->state == SYS_BARCODE_SCAN && state != SYS_BARCODE_SCAN &&
	    m->bc_exit == BCX_NONE) {
		m->bc_exit = BCX_STOPPED;	/* no reason set by the caller */
	}
	m->state = state;
	m->t_ms = 0;
	m->line_hint = 0;		/* set again after go() where it applies */
	m->wait_stop = 0;
	m->line_ok = 0;
	m->creeping = 0;
	m->bc_phase = BCP_READ;
	m->bc_slow = 0;
	m->bc_read_primed = 0;
	if (state == SYS_IDLE) {
		m->bc_skip = 0;			/* a new run reads every code */
	}
}

EXPORT void mission_act_clear(MISSION_ACT *a)
{
	a->evt_flags = 0;
	a->scan_cmd = 0;
	a->scan_arg = 0;
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
	m->lifted = 0;
	m->obs_dist_mm = 0;
	m->obs_width_mm = 0;
	m->obs_side = 0;
	m->fault_flags = 0;
	m->ir_line = 0;
	m->net_online = 0;
	m->bypass_ms = 0;
	m->bypass_late = 0;
	m->byp_phase = BYPP_OUT_TURN;
	m->track_enabled = BYP_TRACK;
	m->byp_track = 0;
	m->heading_deg = 0;
	m->byp_h0 = 0;
	m->line_hint = 0;
	m->byp_lat_mm = 0;
	m->byp_x_mm = 0;
	m->byp_last_fwd = 0;
	m->byp_edge_mm = 0;
	m->byp_front = 0;
	m->byp_aim = 0;
	m->byp_end_fwd = -1;
	m->byp_side_sense = 0;
	m->byp_result = BYPR_NONE;
	m->obs_sweeps = 0;
	m->byp_forced = 0;
	m->stop_at_barcode = 0;
	m->straight_only = 0;
	m->bc_backup = 1;
	m->bc_exit = BCX_NONE;
	m->fwd_mm = 0;
	m->bc_retried = 0;
	m->bc_rev_start_mm = 0;
	m->bc_skip = 0;
	m->bc_skip_until_mm = 0;
	m->move_map[JMOVE_FORWARD] = 0;
	m->move_map[JMOVE_UTURN] = JMAP_DEFAULT_UTURN;
	m->move_map[JMOVE_LEFT] = 0;
	m->move_map[JMOVE_RIGHT] = JMAP_DEFAULT_RIGHT;
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

/* Halt everything and stop pinging; used by IDLE and FAULT_RECOVERY */
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
		m->lifted = 0;			/* a new run */
		a->evt_flags |= EVT_START;
		a->scan_cmd = SCANCMD_ARM;
		nav(a, NAV_RESUME, 0, 0);
		go(m, SYS_LINE_FOLLOW);
	} else if (is_running(m->state) || m->state == SYS_FAULT_RECOVERY) {
		halt(a);
		go(m, SYS_IDLE);
	}
	/* INIT ignores the button */
}

EXPORT void mission_on_mqtt(MISSION *m, const MSG_MQTTCMD *c, MISSION_ACT *a)
{
	if (m->state == SYS_INIT) {
		return;
	}
	if (c->cmd == MQTTCMD_START && m->state == SYS_IDLE) {
		mission_on_button(m, a);	/* exactly what GP20 does */
	} else if (c->cmd == MQTTCMD_STOP &&
		   (is_running(m->state) || m->state == SYS_FAULT_RECOVERY)) {
		mission_on_button(m, a);	/* running -> IDLE */
	}
	/* START while running and STOP while idle do nothing, so a repeated
	   or late message cannot toggle the car the wrong way. */
}

EXPORT void mission_on_steer(MISSION *m, const MSG_STEER *s, MISSION_ACT *a)
{
	m->ir_line = (s->flags & STEER_F_LOST) ? 0 : 1;

	if (m->state == SYS_OBSTACLE_BYPASS) {
		/* Return diagonal: stop when the line reaches the SECOND sensor
		   of the pair, so the line lies between the two when the car
		   turns back along it.  Whichever sensor sees black first is
		   remembered and ignored after that, so this does not depend on
		   which way round the car came (2026-11-18: choosing the sensor
		   from the side picked the wrong one passing on the right). */
		UB dark = s->flags & (STEER_F_DARK_C | STEER_F_DARK_R);
		BOOL second = FALSE;

		if ((m->byp_phase == BYPP_IN_DIAG || m->byp_phase == BYPP_RETURN) &&
		    dark != 0 && !m->straight_only) {
			if (dark == (STEER_F_DARK_C | STEER_F_DARK_R)) {
				second = TRUE;		/* both at once */
			} else if (m->byp_dark_first == 0) {
				m->byp_dark_first = dark;	/* the first one */
			} else if (dark != m->byp_dark_first) {
				second = TRUE;		/* the other one now */
			}
		}
		if (second && m->byp_phase == BYPP_RETURN) {
			/* tracking bypass: on the line, curve back along it
			   while still moving (no fixed turn) */
			m->byp_phase = BYPP_REJOIN;
			m->byp_mark_mm = m->fwd_mm;
			m->byp_result = BYPR_LINE;
		} else if (second) {
			a->evt_flags |= EVT_STOP;
			m->byp_phase = BYPP_STOPPING;
			m->byp_result = BYPR_LINE;
		}
		if (m->byp_phase >= BYPP_FOLLOW && m->byp_phase <= BYPP_OUT_CLEAR) {
			nav(a, NAV_STEER, track_steer(m), 0);	/* Mission steers */
		}
		return;			/* else Motor runs the legs itself */
	}
	if (m->state == SYS_BARCODE_SCAN &&
	    (m->bc_phase == BCP_BACKUP || m->bc_phase == BCP_REVERSE)) {
		return;			/* backing up: Motor runs the move itself */
	}
	if (forwards_steer(m->state) && m->straight_only) {
		/* Obstacle floor test: no line to follow.  Still one message per
		   line cycle, so Motor's watchdog stays fed. */
		nav(a, NAV_STEER, 0, 0);
		return;
	}
	if (forwards_steer(m->state)) {
		if ((s->flags & STEER_F_JUNCTION) && !m->creeping) {
			m->creeping = 1;
			m->creep_ms = 0;
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

/* Forward travel, not path length: steering corrections at the slow speed
   would otherwise end the pass part way through the code. */
EXPORT void mission_on_heading(MISSION *m, INT heading_deg)
{
	m->heading_deg = heading_deg;
}

EXPORT void mission_on_forward(MISSION *m, INT fwd_mm, MISSION_ACT *a)
{
	m->fwd_mm = fwd_mm;
	if (m->bc_skip && fwd_mm >= m->bc_skip_until_mm) {
		m->bc_skip = 0;			/* past the code: read codes again */
	}
	if (m->state != SYS_BARCODE_SCAN || !m->bc_slow || m->bc_phase != BCP_READ) {
		return;
	}
	/* slow pass: never crawl further than the code needs */
	if (!m->bc_read_primed) {
		m->bc_read_base_mm = fwd_mm;
		m->bc_read_primed = 1;
	} else if (fwd_mm - m->bc_read_base_mm >= BC_READ_MAX_MM) {
		nav(a, NAV_RESUME, 0, 0);
		m->bc_exit = BCX_DISTANCE;
		go(m, SYS_LINE_FOLLOW);
	}
}

/* JUNCTION_CREEP_MS after a cross junction was seen: carry out the stored
   letter's move and forget the letter (TEAM.md 7, steps 4 and 5). */
LOCAL void junction_move(MISSION *m, MISSION_ACT *a)
{
	INT deg = mission_junction_deg(m, m->stored_char);

	m->stored_char = 0;
	if (deg != 0) {
		nav(a, NAV_TURN, deg, 0);
		go(m, SYS_BARCODE_CMD);
		m->done_seq_base = SEQ_UNSET;	/* captured on the next tick */
	} else {
		m->creeping = 0;		/* forward: follow the line again */
	}
}

EXPORT void mission_on_barcode(MISSION *m, const MSG_BCCMD *b, MISSION_ACT *a)
{
	if (m->bc_skip) {
		return;		/* driving over the code just read in reverse */
	}
	/* Every letter read is recorded and kept for the next cross junction,
	   whatever the state: a code swiped by hand while IDLE counts too. */
	if (b->kind == BC_OK && b->cmd >= 'A' && b->cmd <= 'Z') {
		m->last_barcode = b->cmd;
		m->stored_char = b->cmd;
	}
	if (b->kind == BC_FIRST) {
		/* A code starts at the normal speed: back up with the reader on hold,
		   then read the code slowly (on_tick).  Only from plain line
		   following; IDLE (a hand swipe) and every other state just let
		   the capture run on. */
		if (m->state == SYS_LINE_FOLLOW && m->bc_backup && !m->creeping &&
		    !m->straight_only) {
			a->evt_flags |= EVT_BC_HOLD;
			nav(a, NAV_MOVE, -BC_BACKUP_MM, 0);
			go(m, SYS_BARCODE_SCAN);
			m->bc_exit = BCX_NONE;		/* a new pass */
			m->bc_retried = 0;
			m->bc_phase = BCP_BACKUP;
			m->done_seq_base = SEQ_UNSET;	/* captured on the next tick */
		}
		return;
	}
	if (m->state != SYS_LINE_FOLLOW && m->state != SYS_BARCODE_SCAN) {
		return;				/* no action during hump, obstacle, ... */
	}
	if (m->state == SYS_BARCODE_SCAN && m->bc_phase == BCP_BACKUP) {
		return;				/* left over from the fast pass */
	}
	if (m->state == SYS_BARCODE_SCAN && m->bc_phase == BCP_REVERSE) {
		/* the reverse read: note the result, keep reversing (on_tick) */
		if (b->kind == BC_OK) {
			m->bc_exit = BCX_DECODED_REV;	/* letter stored above */
		} else if (b->kind == BC_FAIL && m->bc_exit == BCX_NONE) {
			m->bc_exit = BCX_FAILED;
		}
		return;
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
			m->bc_exit = BCX_NONE;
		}
		break;
	case BC_OK:
		/* Stored above; the move waits for the next cross junction. */
		if (m->bc_slow) {
			nav(a, NAV_RESUME, 0, 0);	/* read done: normal speed */
		}
		if (m->state == SYS_BARCODE_SCAN) {
			m->bc_exit = BCX_DECODED;
		}
		go(m, SYS_LINE_FOLLOW);
		break;
	case BC_FAIL:
	default:
		if (m->state == SYS_BARCODE_SCAN && m->bc_slow && !m->bc_retried) {
			/* Try again backwards: reverse to where the slow pass began
			   with a fresh capture. */
			INT rev = m->bc_read_primed ? (m->fwd_mm - m->bc_read_base_mm)
						    : BC_BACKUP_MM;

			if (rev < BC_BACKUP_MM) {
				rev = BC_BACKUP_MM;
			} else if (rev > BC_READ_MAX_MM) {
				rev = BC_READ_MAX_MM;
			}
			a->evt_flags |= EVT_BC_ARM;
			nav(a, NAV_MOVE, -rev, BC_REVERSE_MMPS);
			m->bc_phase = BCP_REVERSE;
			m->bc_retried = 1;
			m->bc_rev_start_mm = m->fwd_mm;
			m->done_seq_base = SEQ_UNSET;	/* captured on the next tick */
			m->t_ms = 0;
			break;
		}
		if (m->bc_slow) {
			nav(a, NAV_RESUME, 0, 0);
		}
		if (m->state == SYS_BARCODE_SCAN) {
			m->bc_exit = BCX_FAILED;
		}
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
	case TERR_LIFTED:
		/* End of the demo: the car is picked up and turned over.  Stop
		   every motor now, whatever the car was doing (a bypass, a turn,
		   a barcode back-up), and wait in IDLE for a new start.  INIT
		   and IDLE just get the stop as well. */
		m->lifted = 1;
		halt(a);
		if (is_running(m->state) || m->state == SYS_FAULT_RECOVERY) {
			go(m, SYS_IDLE);
		}
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

/* ---- tracking bypass (BYP_TRACK) -------------------------------------- */

/* +1 when going round on the right, -1 on the left */
LOCAL INT byp_side(const MISSION *m)
{
	return (m->byp.turn_deg < 0) ? -1 : 1;
}

/* Heading since the stop, + = turned towards the bypass side, degrees */
LOCAL INT byp_rel(const MISSION *m)
{
	return (m->heading_deg - m->byp_h0) * byp_side(m);
}

LOCAL INT clampi(INT v, INT lim)
{
	return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

/* Steering that turns the car to rel = target degrees, at most lim */
LOCAL INT steer_to(const MISSION *m, INT target, INT lim)
{
	return byp_side(m) * clampi(BYP_KH * (target - byp_rel(m)), lim);
}

/* sin of 0..90 degrees x 1000 (10-degree table, linear in between) */
LOCAL INT sin_pm(INT deg)
{
	static const INT t[10] = { 0, 174, 342, 500, 643, 766, 866, 940, 985, 1000 };
	INT i;

	if (deg <= 0) {
		return 0;
	}
	if (deg >= 90) {
		return 1000;
	}
	i = deg / 10;
	return t[i] + ((t[i + 1] - t[i]) * (deg % 10)) / 10;
}

/* sin x 1000 for -90..+90 degrees */
LOCAL INT ssin_pm(INT deg)
{
	return (deg < 0) ? -sin_pm(-deg) : sin_pm(deg);
}

/* Ask Scan to aim at deg (- left, + right) unless it is nearly there */
LOCAL void aim(MISSION *m, MISSION_ACT *a, INT deg, BOOL force)
{
	INT diff = deg - m->byp_aim;

	if (force || diff >= BYP_AIM_STEP_DEG || diff <= -BYP_AIM_STEP_DEG) {
		a->scan_cmd = SCANCMD_TRACK;
		a->scan_arg = deg;
		m->byp_aim = deg;
	}
}

/* Aim along the track: turned rel away, the obstacle's front is rel
   degrees round towards the obstacle side (at most the 45 deg position) */
LOCAL INT front_aim(const MISSION *m)
{
	INT rel = byp_rel(m);

	return -byp_side(m) * ((rel < 0) ? 0 : ((rel > SCAN_MAX_AIM_DEG) ? SCAN_MAX_AIM_DEG : rel));
}

LOCAL void enter_out(MISSION *m, MISSION_ACT *a)
{
	m->byp_phase = BYPP_OUT;
	m->byp_mark_mm = m->fwd_mm;
	m->byp_last_fwd = m->fwd_mm;
	m->byp_lat_mm = 0;
	m->byp_x_mm = 0;
	m->byp_seen = 0;
	m->byp_miss = 0;
	m->byp_front = 0;
	a->evt_flags |= EVT_RESUME;		/* drive, steered by Mission */
	nav(a, NAV_SPEED, BYP_OUT_MMPS, 0);
	nav(a, NAV_STEER, 0, 0);
	aim(m, a, 0, TRUE);			/* straight at it */
}

LOCAL void enter_out_clear(MISSION *m)
{
	m->byp_phase = BYPP_OUT_CLEAR;
	m->byp_mark_mm = m->fwd_mm;
	m->byp_edge_mm = m->byp_lat_mm;		/* the sensor is past its edge here */
}

/* A reading straight down the track at the obstacle's front (OUT) */
LOCAL void out_range(MISSION *m, INT dist_mm)
{
	if (dist_mm == SCAN_NO_ECHO || dist_mm > BYP_SEEN_MAX_MM) {
		if (m->byp_seen && ++m->byp_miss >= BYP_LOST_PINGS) {
			enter_out_clear(m);	/* the beam now misses its edge */
		}
		return;
	}
	m->byp_seen = 1;
	m->byp_miss = 0;
	m->byp_front = dist_mm;
}

/* Sideways room the car needs beyond the obstacle's edge */
LOCAL INT side_room(const MISSION *m)
{
	return BYP_CAR_HALF_MM + BYP_MARGIN_MM +
	       ((byp_side(m) > 0) ? BYP_RIGHT_EXTRA_MM : 0);
}

LOCAL void enter_follow(MISSION *m, MISSION_ACT *a)
{
	m->byp_phase = BYPP_FOLLOW;
	m->byp_mark_mm = m->fwd_mm;
	m->byp_seen = 0;
	m->byp_miss = 0;
	m->byp_corr = 0;
	m->byp_have_err = 0;
	m->byp_too_close = 0;
	a->evt_flags |= EVT_RESUME;		/* (still) driving, steered by Mission */
	nav(a, NAV_SPEED, BYP_MMPS, 0);
	aim(m, a, -byp_side(m) * BYP_FOLLOW_SIDE_DEG, TRUE);	/* facing its side */
}

LOCAL void enter_clear(MISSION *m, MISSION_ACT *a)
{
	m->byp_phase = BYPP_CLEAR;
	m->byp_mark_mm = m->fwd_mm;
	a->scan_cmd = SCANCMD_DISARM;		/* sensor back to centre */
}

/* A reading at the held angle while following (OBS_RANGE) */
LOCAL void follow_range(MISSION *m, INT dist_mm, MISSION_ACT *a)
{
	INT lat, err;

	if (dist_mm == SCAN_NO_ECHO || dist_mm > BYP_SEEN_MAX_MM) {
		if (m->byp_seen && ++m->byp_miss >= BYP_LOST_PINGS) {
			enter_clear(m, a);	/* the sensor has passed its end */
		}
		return;
	}
	m->byp_seen = 1;
	m->byp_miss = 0;
	if (byp_rel(m) > BYP_FOLLOW_AIM_DEG) {
		m->byp_corr = 0;		/* still turning back to parallel: */
		m->byp_too_close = 0;		/* the beam runs along its side */
		m->byp_have_err = 0;
		return;
	}
	/* sideways distance: the beam is BYP_FOLLOW_SIDE_DEG off the car's
	   heading towards the obstacle; turned byp_rel() away from it, the
	   beam meets the obstacle's side at that minus rel degrees */
	lat = (dist_mm * sin_pm(BYP_FOLLOW_SIDE_DEG - byp_rel(m))) / 1000;
	m->byp_too_close = (lat < BYP_TOO_CLOSE_MM) ? 1 : 0;
	err = lat - BYP_GAP_MM;			/* + = too far out */
	m->byp_corr = (BYP_KP_X100 * err +
		       (m->byp_have_err ? BYP_KD_X100 * (err - m->byp_last_err) : 0)) / 100;
	m->byp_last_err = err;
	m->byp_have_err = 1;
}

/* Steering for the tracking phases, sent with every line message */
LOCAL INT track_steer(const MISSION *m)
{
	INT mag;

	switch (m->byp_phase) {
	case BYPP_OUT:
		if (byp_rel(m) >= BYP_OUT_MAX_DEG) {
			return steer_to(m, BYP_OUT_MAX_DEG, BYP_STEER_MAX);	/* no further */
		}
		/* away from it, harder the nearer its front */
		mag = BYP_OUT_STEER;
		if (m->byp_front > 0 && m->byp_front < BYP_FRONT_REF_MM) {
			mag += (BYP_OUT_KF_X100 * (BYP_FRONT_REF_MM - m->byp_front)) / 100;
		}
		return byp_side(m) * ((mag > BYP_STEER_MAX) ? BYP_STEER_MAX : mag);
	case BYPP_OUT_CLEAR: {
		/* keep moving out: hold the heading, at least BYP_OUT_MIN_DEG */
		INT rel = byp_rel(m);

		rel = (rel < BYP_OUT_MIN_DEG) ? BYP_OUT_MIN_DEG :
		      ((rel > BYP_OUT_MAX_DEG) ? BYP_OUT_MAX_DEG : rel);
		return steer_to(m, rel, BYP_STEER_MAX);
	}
	case BYPP_FOLLOW:
		if (m->byp_too_close) {
			return byp_side(m) * BYP_STEER_MAX;	/* away from it */
		}
		/* hold parallel, pulled in (too far) or out (too near) */
		return byp_side(m) * clampi(BYP_KH * -byp_rel(m) - m->byp_corr, BYP_STEER_MAX);
	case BYPP_CLEAR:
		return steer_to(m, 0, BYP_STEER_MAX);
	case BYPP_RETURN:
		return steer_to(m, -BYP_RETURN_DEG, BYP_RETURN_STEER);
	default:				/* BYPP_REJOIN */
		return steer_to(m, 0, BYP_RETURN_STEER);
	}
}

/* Done: follow the line again, told which side it is on */
LOCAL void track_finish(MISSION *m, MISSION_ACT *a)
{
	INT side = byp_side(m);

	m->bypass_ms = m->t_ms;
	m->bypass_late = (m->t_ms > BYPASS_TARGET_MS) ? 1 : 0;
	nav(a, NAV_RESUME, 0, 0);
	a->scan_cmd = SCANCMD_ARM;
	go(m, SYS_LINE_FOLLOW);
	/* the car came back from the bypass side; if it is off the line now,
	   the line is back towards that side */
	m->line_hint = side;
}

LOCAL void enter_return(MISSION *m)
{
	m->byp_phase = BYPP_RETURN;
	m->byp_mark_mm = m->fwd_mm;
	m->byp_dark_first = 0;
}

/* Every tick in a tracking phase: distances and headings that end it */
LOCAL void track_tick(MISSION *m, MISSION_ACT *a)
{
	INT d = m->fwd_mm - m->byp_mark_mm;
	INT rel = byp_rel(m);

	/* sideways position from the line: forward travel x sin(heading).
	   Added up per 10 mm travelled, not per tick: at 45 mm/s a 5 ms tick
	   is under 1 mm, and x sin / 1000 rounded every step to 0 (the car
	   never knew it had moved out; floor run 2026-11-22). */
	if (m->fwd_mm - m->byp_last_fwd >= BYP_LAT_STEP_MM ||
	    m->fwd_mm - m->byp_last_fwd <= -BYP_LAT_STEP_MM) {
		INT step = m->fwd_mm - m->byp_last_fwd;

		m->byp_lat_mm += (step * ssin_pm(rel)) / 1000;
		m->byp_x_mm += (step * sin_pm(90 - ((rel < 0) ? -rel : rel))) / 1000;
		m->byp_last_fwd = m->fwd_mm;
	}

	switch (m->byp_phase) {
	case BYPP_OUT:
		aim(m, a, front_aim(m), FALSE);		/* keep looking at its front */
		if (d > BYP_OUT_MAX_MM) {
			fault(m, a, FAULT_SCAN);	/* cannot get round it */
		} else if (!m->byp_seen && d > BYP_NOSEE_MM) {
			enter_out_clear(m);		/* never saw it: treat as past */
		}
		break;
	case BYPP_OUT_CLEAR:
		if (m->byp_lat_mm >= m->byp_edge_mm + side_room(m)) {
			enter_follow(m, a);		/* body clear: follow its side */
		} else if (d > BYP_OUT_MAX_MM) {
			fault(m, a, FAULT_SCAN);
		}
		break;
	case BYPP_FOLLOW:
		if (d > BYP_FOLLOW_MAX_MM) {
			fault(m, a, FAULT_SCAN);	/* never found its end */
		} else if (!m->byp_seen &&
			   (d > BYP_FOLLOW_MAX_MM / 2 ||
			    m->byp_x_mm >= m->obs_dist_mm + BYP_OBS_DEPTH_MM + BYP_CAR_LEN_MM)) {
			/* never seen alongside: by odometry the whole car is
			   past where its far end can be; curve in now (2026-11-22:
			   NOSEE + CLEAR ran 600 mm straight first) */
			a->scan_cmd = SCANCMD_DISARM;
			enter_return(m);
		}
		break;
	case BYPP_CLEAR:
		if (d >= BYP_CLEAR_MM) {
			enter_return(m);
		}
		break;
	case BYPP_RETURN:
		if (m->byp_lat_mm <= BYP_ONLINE_MM) {
			/* back over the line by odometry, the IR missed it */
			m->byp_phase = BYPP_REJOIN;
			m->byp_mark_mm = m->fwd_mm;
			m->byp_result = BYPR_ODOM;
		} else if (d > BYP_RETURN_MAX_MM) {
			m->byp_result = BYPR_NO_LINE;
			fault(m, a, FAULT_LINE);	/* no line: stop */
		}
		break;
	default:				/* BYPP_REJOIN */
		if ((rel <= BYP_ALIGN_DEG && rel >= -BYP_ALIGN_DEG) || d > BYP_REJOIN_MAX_MM) {
			track_finish(m, a);
		}
		break;
	}
}

/* A sideways reading during the straight leg (dog-leg, OBS_RANGE) */
LOCAL void pass_range(MISSION *m, INT dist_mm)
{
	if (dist_mm != SCAN_NO_ECHO && dist_mm <= BYP_SIDE_SEEN_MM) {
		m->byp_seen = 1;		/* the obstacle is beside the car */
		m->byp_miss = 0;
		return;
	}
	if (m->byp_seen && m->byp_end_fwd < 0 && ++m->byp_miss >= BYP_LOST_PINGS) {
		m->byp_end_fwd = m->fwd_mm;	/* the sensor has passed its far end */
	}
}

/* Every tick of a side-watched straight leg: TRUE when it should end */
LOCAL BOOL pass_done(const MISSION *m)
{
	INT d = m->fwd_mm - m->byp_mark_mm;

	if (d < BYP_PASS_MIN_MM) {
		return FALSE;
	}
	if (m->byp_end_fwd >= 0) {
		return (m->fwd_mm - m->byp_end_fwd >= BYP_PASS_AFTER_MM) ? TRUE : FALSE;
	}
	/* never seen beside the car: the planned length */
	return (!m->byp_seen && d >= m->byp.pass_mm) ? TRUE : FALSE;
}

EXPORT void mission_bypass_plan(INT side, INT width_mm, INT dist_mm,
				BOOL search, BYPASS_PLAN *p)
{
	INT w = width_mm, lateral, gain;

	if (w < BYP_WIDTH_MIN_MM) {
		w = BYP_WIDTH_MIN_MM;
	} else if (w > BYP_WIDTH_MAX_MM) {
		w = BYP_WIDTH_MAX_MM;
	}
	/* sideways offset of the car's centre from the line */
	lateral = w / 2 + BYP_MARGIN_MM + BYP_CAR_HALF_MM;
	if (side == OBS_SIDE_RIGHT) {
		lateral += BYP_RIGHT_EXTRA_MM;	/* the left sensor sticks out */
	}
	p->turn_deg = (side == OBS_SIDE_LEFT) ? -BYP_TURN_CMD_DEG : BYP_TURN_CMD_DEG;
	p->diag_mm = (lateral * 1414) / 1000 - BYP_DIAG_TRIM_MM;	/* / sin 45 */
	if (p->diag_mm < BYP_DIAG_MIN_MM) {
		p->diag_mm = BYP_DIAG_MIN_MM;
	}
	/* a 45 degree diagonal gains diag x sin 45 sideways AND along the
	   track; the straight leg covers the rest of the way past */
	gain = (p->diag_mm * 707) / 1000;
	p->pass_mm = dist_mm + BYP_OBS_DEPTH_MM + BYP_CAR_LEN_MM - gain
		     - BYP_PASS_TRIM_MM;
	if (p->pass_mm < BYP_PASS_MIN_MM) {
		p->pass_mm = BYP_PASS_MIN_MM;
	}
	p->in_diag_mm = p->diag_mm + (search ? BYP_SEARCH_MM : 0);
}

/* Send the Motor command of bypass phase m->byp_phase */
LOCAL void bypass_step(MISSION *m, MISSION_ACT *a)
{
	INT t = m->byp.turn_deg;

	switch (m->byp_phase) {
	case BYPP_OUT_TURN:	nav(a, NAV_TURN, t, 0);				break;
	case BYPP_OUT_DIAG:	nav(a, NAV_MOVE, m->byp.diag_mm, BYP_MMPS);	break;
	case BYPP_PAR_TURN:	nav(a, NAV_TURN, -t, 0);			break;
	case BYPP_PASS:
		m->byp_side_sense = (BYP_SIDE_SENSE && !m->straight_only) ? 1 : 0;
		if (m->byp_side_sense) {
			/* drive on until the obstacle's far end has gone by,
			   watching it fully sideways */
			m->byp_mark_mm = m->fwd_mm;
			m->byp_end_fwd = -1;
			m->byp_seen = 0;
			m->byp_miss = 0;
			nav(a, NAV_MOVE, BYP_PASS_MAX_MM, BYP_MMPS);
			a->scan_cmd = SCANCMD_TRACK;
			a->scan_arg = -byp_side(m) * BYP_SIDE_AIM_DEG;
		} else {
			nav(a, NAV_MOVE, m->byp.pass_mm, BYP_MMPS);
		}
		break;
	case BYPP_IN_TURN:	nav(a, NAV_TURN, -t, 0);			break;
	case BYPP_IN_DIAG:	nav(a, NAV_MOVE, m->byp.in_diag_mm, BYP_MMPS);	break;
	case BYPP_ALIGN_TURN:	nav(a, NAV_TURN, t, 0);				break;
	case BYPP_PAST_LINE:	nav(a, NAV_MOVE, BYP_PAST_LINE_MM, BYP_MMPS);	break;
	default:								break;
	}
	m->done_seq_base = SEQ_UNSET;	/* captured on the next tick */
}

/* Width to plan the bypass for: twice how far sideways the sweep saw the
   obstacle on the chosen side (MSG_OBS.reach_mm), within the limits */
LOCAL INT reach_width(INT reach_mm)
{
	INT half = reach_mm;

	if (half < BYP_HALF_MIN_MM) {
		half = BYP_HALF_MIN_MM;
	} else if (half > BYP_HALF_MAX_MM) {
		half = BYP_HALF_MAX_MM;
	}
	return 2 * half;
}

/* Leave OBSTACLE_SCAN on a 45 degree dog-leg round the obstacle */
LOCAL void start_bypass(MISSION *m, MISSION_ACT *a, INT side, INT width_mm,
			INT dist_mm)
{
	m->obs_side = side;
	m->obs_width_mm = width_mm;
	if (dist_mm > 0) {
		m->obs_dist_mm = dist_mm;	/* from the sweep, car stopped */
	}
	mission_bypass_plan(side, width_mm, m->obs_dist_mm,
			    m->straight_only ? FALSE : TRUE, &m->byp);
	go(m, SYS_OBSTACLE_BYPASS);
	m->byp_phase = BYPP_OUT_TURN;
	m->byp_dark_first = 0;
	/* track the obstacle, unless there is no line to come back to */
	m->byp_track = (m->track_enabled && !m->straight_only) ? 1 : 0;
	m->byp_h0 = m->heading_deg;	/* stopped square to the line */
	if (m->byp_track) {
		enter_out(m, a);		/* steered by the sensor from here */
	} else {
		bypass_step(m, a);		/* fixed dog-leg */
	}
}

EXPORT void mission_on_obs(MISSION *m, const MSG_OBS *o, MISSION_ACT *a)
{
	switch (o->kind) {
	case OBS_DETECTED:
		if (m->state == SYS_LINE_FOLLOW || m->state == SYS_BARCODE_SCAN ||
		    m->state == SYS_HUMP_TRAVERSE) {
			m->obs_dist_mm = o->dist_mm;
			m->obs_sweeps = 0;
			a->evt_flags |= EVT_STOP;	/* stop first, then profile */
			go(m, SYS_OBSTACLE_SCAN);
			m->wait_stop = 1;
		}
		break;
	case OBS_RESULT:
		if (m->state == SYS_OBSTACLE_SCAN) {
			m->obs_sweeps++;
			m->byp_forced = 0;
			start_bypass(m, a, o->side, reach_width(o->reach_mm), o->dist_mm);
		}
		break;
	case OBS_FAIL:
		if (m->state == SYS_OBSTACLE_SCAN) {
			m->obs_sweeps++;
			if (m->obs_sweeps >= BYP_MAX_FAILS) {
				/* still "no way round": go round anyway, on the
				   less blocked side, as wide as allowed */
				m->byp_forced = 1;
				start_bypass(m, a,
					     (o->side != 0) ? o->side : BYP_DEFAULT_SIDE,
					     2 * BYP_HALF_MAX_MM, o->dist_mm);
			} else {
				/* sweep again: a side may open (or it goes) */
				m->t_ms = 0;	/* MS_OBSTACLE_SCAN is per sweep */
				a->scan_cmd = SCANCMD_PROFILE;
			}
		}
		break;
	case OBS_RANGE:
		if (m->state == SYS_OBSTACLE_BYPASS && m->byp_phase == BYPP_FOLLOW) {
			follow_range(m, o->dist_mm, a);
		} else if (m->state == SYS_OBSTACLE_BYPASS && m->byp_phase == BYPP_OUT) {
			out_range(m, o->dist_mm);
		} else if (m->state == SYS_OBSTACLE_BYPASS && m->byp_phase == BYPP_PASS &&
			   m->byp_side_sense) {
			pass_range(m, o->dist_mm);
		}
		break;
	case OBS_CLEAR:
		if (m->state == SYS_OBSTACLE_SCAN) {
			start_driving(a);	/* nothing ahead any more */
			go(m, SYS_LINE_FOLLOW);
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
		if (m->bc_phase == BCP_BACKUP) {
			/* the back-up is a Motor sequence; it ends in STOPPED */
			if (m->done_seq_base == SEQ_UNSET) {
				m->done_seq_base = motor_done_seq;
			} else if (motor_done_seq != m->done_seq_base) {
				/* now cross the code slowly with a fresh capture */
				a->evt_flags |= EVT_RESUME | EVT_BC_ARM;
				nav(a, NAV_SLOW, 0, 0);
				nav(a, NAV_STEER, 0, 0);
				m->bc_phase = BCP_READ;
				m->bc_slow = 1;
				m->t_ms = 0;
			} else if (m->t_ms >= MS_BC_BACKUP) {
				m->bc_exit = BCX_BACKUP_FAULT;
				fault(m, a, FAULT_LINE);
			}
		} else if (m->bc_phase == BCP_REVERSE) {
			/* the reverse read is a Motor sequence too */
			if (m->done_seq_base == SEQ_UNSET) {
				m->done_seq_base = motor_done_seq;
			} else if (motor_done_seq != m->done_seq_base) {
				INT skip_to = m->bc_rev_start_mm + BC_SKIP_MARGIN_MM;

				/* back before the code: drive on over it at the
				   normal speed without reading it a third time */
				if (m->bc_exit == BCX_NONE) {
					m->bc_exit = BCX_FAILED;
				}
				start_driving(a);
				go(m, SYS_LINE_FOLLOW);
				m->bc_skip = 1;
				m->bc_skip_until_mm = skip_to;
			} else if (m->t_ms >= MS_BC_REVERSE) {
				m->bc_exit = BCX_BACKUP_FAULT;
				fault(m, a, FAULT_LINE);
			}
		} else if (m->t_ms >= MS_BARCODE_SCAN) {
			if (m->bc_slow) {
				nav(a, NAV_RESUME, 0, 0);
			}
			m->bc_exit = BCX_TIMEOUT;
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
		if (m->byp_phase >= BYPP_FOLLOW && m->byp_phase <= BYPP_OUT_CLEAR) {
			track_tick(m, a);		/* steered, not Motor legs */
			break;
		}
		if (m->byp_phase == BYPP_PASS_STOPPING) {
			if (motor_state == MOT_STOPPED) {
				m->byp_phase = BYPP_IN_TURN;	/* past it: turn in */
				bypass_step(m, a);
			}
			break;
		}
		if (m->byp_phase == BYPP_PASS && m->byp_side_sense && pass_done(m)) {
			a->evt_flags |= EVT_STOP;		/* far end + car length passed */
			a->scan_cmd = SCANCMD_DISARM;		/* sensor back to centre */
			m->byp_phase = BYPP_PASS_STOPPING;
			break;
		}
		if (m->byp_phase == BYPP_STOPPING) {
			if (motor_state == MOT_STOPPED) {
				m->byp_phase = BYPP_PAST_LINE;	/* a little further on */
				bypass_step(m, a);
			}
			break;
		}
		if (m->done_seq_base == SEQ_UNSET) {
			m->done_seq_base = motor_done_seq;	/* remember the counter */
			break;
		}
		if (motor_done_seq == m->done_seq_base) {
			break;				/* this leg is still running */
		}
		if (m->byp_phase == BYPP_PAST_LINE) {
			m->byp_phase = BYPP_ALIGN_TURN;	/* on the line: face along it */
			bypass_step(m, a);
			break;
		}
		if (m->byp_track && m->byp_phase == BYPP_PAR_TURN) {
			enter_follow(m, a);		/* now follow it at 15 cm */
			break;
		}
		if (m->byp_phase < BYPP_ALIGN_TURN) {
			if (m->byp_phase == BYPP_PASS && m->byp_side_sense) {
				a->scan_cmd = SCANCMD_DISARM;	/* ran its full length */
			}
			if (m->byp_phase == BYPP_IN_DIAG) {
				m->byp_result = BYPR_NO_LINE;	/* full length, no black */
			}
			m->byp_phase++;			/* next leg */
			bypass_step(m, a);
			break;
		}
		{
			/* bypass finished: creep forward until the line is found */
			m->bypass_ms = m->t_ms;
			m->bypass_late = (m->t_ms > BYPASS_TARGET_MS) ? 1 : 0;
			if (m->straight_only) {
				/* no line to find: carry on straight */
				start_driving(a);
				go(m, SYS_LINE_FOLLOW);
				break;
			}
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

	case SYS_LINE_FOLLOW:
		if (m->line_hint != 0 && m->t_ms >= BYP_HINT_MS) {
			m->line_hint = 0;	/* the line follower has taken it */
		}
		break;

	default:				/* IDLE */
		break;
	}

	/* Junction creep: straight on for JUNCTION_CREEP_MS, then the move */
	if (m->creeping && forwards_steer(m->state)) {
		m->creep_ms += dt_ms;
		if (m->creep_ms >= JUNCTION_CREEP_MS) {
			junction_move(m, a);
		}
	}
}
