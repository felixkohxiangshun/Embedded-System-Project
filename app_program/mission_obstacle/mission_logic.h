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
#define MS_BARCODE_SCAN		40000	/* backstop for the slow pass (BC_READ_MAX_MM is 33 s) */
#define MS_BC_BACKUP		5000	/* backing up BC_BACKUP_MM at 50 mm/s takes 2 s */
#define BC_BACKUP_MM		100	/* back up this far when a code starts
					   (was 50; the barcode sensor sits further
					   back on the car) [CAL] */
/* Reverse re-read: when the slow forward read fails, the car reverses at
   this speed back over the code to where the slow pass began, reading it
   backwards (the decoder accepts a reversed code). */
#define BC_REVERSE_MMPS		18
#define MS_BC_REVERSE		40000	/* up to BC_READ_MAX_MM at 18 mm/s */
#define BC_SKIP_MARGIN_MM	100	/* after the reverse: ignore barcodes until
					   this far past where it began */
/* Slow pass backstop: only for a code the reader never reports on.  A code
   still being read ends on its own result (BC_OK, or BC_FAIL once the
   reader has seen 32 mm of white).  The first limit, back-up + 141 mm code
   + 40 mm = 231 mm, cut a real read off on 2026-11-09: the reader saw a bar
   or space every 10.5 mm, about twice what 3 mm narrow bars give, so the
   printed code (or the odometry) is about twice as long. [CAL] */
#define BC_READ_MAX_MM		600
#define MS_BARCODE_CMD		5000	/* a 90 or 180 degree turn */
#define MS_HUMP			10000	/* a hump at 18 mm/s takes several seconds */
#define MS_OBSTACLE_SCAN	10000	/* stop, sweep five positions (about 2.2 s), evaluate */
#define BYPASS_TARGET_MS	5000	/* NFR-10: a grade, not a limit */
#define MS_LINE_REACQUIRE	5000
#define MS_FAULT_RECOVERY	1000
#define SEQ_UNSET		0xFFFFFFFFu	/* done_seq_base not captured yet */
#define LINE_FOUND_TICKS	5	/* consecutive steer messages with the line */
#define JUNCTION_CREEP_MS	1500	/* junction seen -> carry out the move (was 150 mm) [CAL] */

/* Junction moves (TEAM.md 7): which letter means which move.  The mapping
   is meant to come from the PC over MQTT; until that command exists only
   right turn = 'F' is set (was U-turn until 2026-11-09).  0 = no letter
   mapped to that move. */
#define JMOVE_FORWARD		0
#define JMOVE_UTURN		1
#define JMOVE_LEFT		2
#define JMOVE_RIGHT		3
#define JMOVE_COUNT		4
#define JMAP_DEFAULT_RIGHT	'F'

/* BARCODE_SCAN phases.  When a code starts (BC_FIRST: two bars close
   together) the car backs up BC_BACKUP_MM with the reader on hold, then
   crosses the code at the slow speed with a fresh capture, then returns to
   the normal speed on the result, or after BC_READ_MAX_MM at the latest.
   A failed forward read is tried once more in reverse (BCP_REVERSE). */
#define BCP_READ		0	/* following the line over the code */
#define BCP_BACKUP		1	/* backing up, reader on hold */
#define BCP_REVERSE		2	/* forward read failed: reversing, reading again */

/* Why the last BARCODE_SCAN ended (shown on the dashboard) */
#define BCX_NONE		0	/* no pass yet */
#define BCX_DECODED		1	/* BC_OK from the reader */
#define BCX_FAILED		2	/* BC_FAIL from the reader */
#define BCX_DISTANCE		3	/* Mission: slow pass longer than BC_READ_MAX_MM */
#define BCX_TIMEOUT		4	/* Mission: MS_BARCODE_SCAN */
#define BCX_BACKUP_FAULT	5	/* Mission: the back-up never finished */
#define BCX_STOPPED		6	/* button, E_STOP, fault: left the state */
#define BCX_DECODED_REV		7	/* forward read failed, reverse read decoded */

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
	INT	creep_ms;		/* time since the junction was seen */
	UB	move_map[JMOVE_COUNT];	/* letter for each JMOVE_xxx, 0 = none */
	UB	stop_at_barcode;	/* floor test: stop once a barcode is read, set by the task */
	UB	straight_only;		/* obstacle floor test: ignore the line, drive straight, set by the task */
	UB	bc_backup;		/* 1 = back up and read slowly on the first bar */
	UB	bc_phase;		/* BARCODE_SCAN: BCP_xxx */
	UB	bc_slow;		/* this read slowed the car: resume after it */
	UB	bc_read_primed;		/* bc_read_base_mm holds the slow-pass start */
	UB	bc_exit;		/* why the last barcode pass ended: BCX_xxx */
	INT	bc_read_base_mm;	/* forward travel (fwd_mm) at that point */
	INT	fwd_mm;			/* latest forward travel (mission_on_forward) */
	UB	bc_retried;		/* this pass already reversed once */
	INT	bc_rev_start_mm;	/* fwd_mm where the reverse began */
	UB	bc_skip;		/* ignore barcodes until bc_skip_until_mm */
	INT	bc_skip_until_mm;
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

/* Signed forward travel (app_state.fwd_mm): ends a slow barcode pass */
IMPORT void mission_on_forward(MISSION *m, INT fwd_mm, MISSION_ACT *a);

/* TEAM.md 7: the turn for a stored letter at a cross junction, in degrees
   (+ right): forward 0, U-turn 180, left -90, right +90.  A letter that is
   not mapped (or 0, nothing stored) goes forward. */
IMPORT INT  mission_junction_deg(const MISSION *m, UB ch);

#endif /* MISSION_LOGIC_H */
