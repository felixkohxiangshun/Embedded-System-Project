/*
 *	mission_logic.h
 *	Mission/obstacle -- the system statechart (design review section 5) as pure
 *	logic: events in, actions out.  No kernel calls, no hardware, so every
 *	transition can be tested on a PC and on the target.
 *
 *	task_mission.c feeds it the messages it receives and carries out the
 *	actions it returns (sets event-flag bits, sends Motor/Scan commands).
 *
 *	States are SYS_xxx in app_if.h:
 *	  INIT -> IDLE -> RUNNING{ LINE_FOLLOW, BARCODE_SCAN, BARCODE_CMD,
 *	  HUMP_TRAVERSE, OBSTACLE_SCAN, OBSTACLE_BYPASS, LINE_REACQUIRE }
 *	  with FAULT_RECOVERY and E_STOP reachable from RUNNING.
 *
 *	Per the final design review: INIT waits for WiFi + MQTT (but gives up
 *	after MS_INIT_NET so the car can still run without a dashboard);
 *	FAULT_RECOVERY returns to IDLE (the car never restarts without
 *	BTN_START); a bypass longer than 5 s is recorded, never aborted.
 */

#ifndef MISSION_LOGIC_H
#define MISSION_LOGIC_H

/* Timeouts, ms.  Every waiting state has one, so nothing can hang. */
#define MS_INIT			500	/* let sensors settle (IMU calibrates) */
#define MS_INIT_NET		20000	/* INIT waits this long for WiFi + MQTT */
#define MS_BARCODE_SCAN		15000	/* a whole code at 24 mm/s takes about 7 s */
#define MS_BARCODE_CMD		5000	/* a 90 or 180 degree turn */
#define MS_HUMP			3000
#define MS_OBSTACLE_SCAN	10000	/* stop, sweep five angles, evaluate */
#define BYPASS_TARGET_MS	5000	/* NFR-10: a grade, not a limit */
#define MS_LINE_REACQUIRE	5000
#define MS_FAULT_RECOVERY	1000
#define SEQ_UNSET		0xFFFFFFFFu	/* done_seq_base not captured yet */
#define LINE_FOUND_TICKS	5	/* consecutive steer messages with the line */
#define JUNCTION_CREEP_MM	150	/* detection to axle on the junction [CAL] */

/* Junction moves (TEAM.md 7): which letter means which move.  The mapping
   is meant to come from the PC over MQTT; until that command exists only
   U-turn = 'F' is set.  0 = no letter mapped to that move. */
#define JMOVE_FORWARD		0
#define JMOVE_UTURN		1
#define JMOVE_LEFT		2
#define JMOVE_RIGHT		3
#define JMOVE_COUNT		4
#define JMAP_DEFAULT_UTURN	'F'

/* What the caller must do after an event.  Actions accumulate; the caller
   clears the struct before feeding a batch of events. */
#define MAX_NAV_ACTIONS		4
typedef struct {
	UB	evt_flags;		/* EVT_START | EVT_STOP | EVT_RESUME to set */
	UB	scan_cmd;		/* SCANCMD_xxx, 0 = none */
	INT	n_nav;
	MSG_NAV	nav[MAX_NAV_ACTIONS];
} MISSION_ACT;

typedef struct {
	UB	state;			/* SYS_xxx */
	INT	t_ms;			/* time spent in this state */
	UW	done_seq_base;		/* Motor completion counter when cmd sent */
	UB	wait_stop;		/* OBSTACLE_SCAN: waiting for Motor to stop */
	INT	line_ok;		/* LINE_REACQUIRE: consecutive good steers */
	UW	recoveries;		/* FAULT_RECOVERY count (telemetry) */
	/* results, copied to app_state by the task */
	UB	last_barcode;
	UB	stored_char;		/* letter kept for the next cross junction, 0 = none */
	INT	hump_mm;
	INT	hump_max_mm;
	UB	shock;
	INT	obs_dist_mm;
	INT	obs_width_mm;
	INT	obs_side;
	UB	fault_flags;		/* FAULT_IMU / FAULT_LINE / FAULT_SCAN */
	UB	ir_line;
	UB	net_online;		/* set by the task from app_state */
	INT	bypass_ms;		/* duration of the last bypass */
	UB	bypass_late;		/* 1 = last bypass took over BYPASS_TARGET_MS */
	UB	creeping;		/* junction seen: driving straight to it */
	UB	creep_primed;		/* creep_base_mm holds the start distance */
	UW	creep_base_mm;		/* distance when the creep began */
	UB	move_map[JMOVE_COUNT];	/* letter for each JMOVE_xxx, 0 = none */
	UB	stop_at_barcode;	/* floor test: stop once a barcode is read, set by the task */
} MISSION;

IMPORT void mission_init(MISSION *m);
IMPORT void mission_act_clear(MISSION_ACT *a);

IMPORT void mission_on_button(MISSION *m, MISSION_ACT *a);
IMPORT void mission_on_mqtt(MISSION *m, const MSG_MQTTCMD *c, MISSION_ACT *a);
IMPORT void mission_on_steer(MISSION *m, const MSG_STEER *s, MISSION_ACT *a);
IMPORT void mission_on_barcode(MISSION *m, const MSG_BCCMD *b, MISSION_ACT *a);
IMPORT void mission_on_terrain(MISSION *m, const MSG_TERRAIN *t, MISSION_ACT *a);
IMPORT void mission_on_obs(MISSION *m, const MSG_OBS *o, MISSION_ACT *a);

/* Called every tick with the Motor state and completion counter read from
   app_state.  dt_ms is the time since the previous tick. */
IMPORT void mission_on_tick(MISSION *m, INT dt_ms, UB motor_state,
			    UW motor_done_seq, MISSION_ACT *a);

/* Called every tick with the distance read from app_state.  Ends a junction
   creep once the car has driven JUNCTION_CREEP_MM. */
IMPORT void mission_on_distance(MISSION *m, UW dist_mm, MISSION_ACT *a);

/* TEAM.md 7: the turn for a stored letter at a cross junction, in degrees
   (+ right): forward 0, U-turn 180, left -90, right +90.  A letter that is
   not mapped (or 0, nothing stored) goes forward. */
IMPORT INT  mission_junction_deg(const MISSION *m, UB ch);

#endif /* MISSION_LOGIC_H */
