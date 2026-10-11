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
#define MS_BARCODE_SCAN		3000
#define MS_BARCODE_CMD		5000	/* a 90 or 180 degree turn */
#define MS_HUMP			3000
#define MS_OBSTACLE_SCAN	10000	/* stop, sweep five angles, evaluate */
#define BYPASS_TARGET_MS	5000	/* NFR-10: a grade, not a limit */
#define MS_LINE_REACQUIRE	5000
#define MS_FAULT_RECOVERY	1000
#define SEQ_UNSET		0xFFFFFFFFu	/* done_seq_base not captured yet */
#define LINE_FOUND_TICKS	5	/* consecutive steer messages with the line */

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

/* FR-03: 'A' left 90, 'B' right 90, 'C' straight, 'D' u-turn.
   Degrees, + right.  *valid is FALSE for any other character. */
IMPORT INT  mission_barcode_turn(UB ch, BOOL *valid);

#endif /* MISSION_LOGIC_H */
