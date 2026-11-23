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
 *	  with FAULT_RECOVERY reachable from RUNNING.
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
#define BC_BACKUP_MM		50	/* back up this far when a code starts
					   (halved from 100 on 2026-11-17) [CAL] */
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
#define MS_OBSTACLE_SCAN	10000	/* one sweep (seven positions, about 6 s) must
					   answer within this; a "no way round"
					   answer starts another sweep */
#define BYPASS_TARGET_MS	30000	/* NFR-10: a grade, not a limit */
#define MS_LINE_REACQUIRE	5000
#define MS_FAULT_RECOVERY	1000
#define SEQ_UNSET		0xFFFFFFFFu	/* done_seq_base not captured yet */
#define LINE_FOUND_TICKS	5	/* consecutive steer messages with the line */
#define JUNCTION_CREEP_MS	1500	/* junction seen -> carry out the move (was 150 mm) [CAL] */

/* Junction moves (TEAM.md 7): which letter means which move.  The mapping
   is meant to come from the PC over MQTT; until that command exists the
   defaults are U-turn = 'Z' and right turn = 'Q' (2026-11-18; 'F', then
   'A', were the U-turn letter before; both now go forward).  0 = no letter mapped
   to that move. */
#define JMOVE_FORWARD		0
#define JMOVE_UTURN		1
#define JMOVE_LEFT		2
#define JMOVE_RIGHT		3
#define JMOVE_COUNT		4
#define JMAP_DEFAULT_UTURN	'Z'
#define JMAP_DEFAULT_RIGHT	'Q'

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
#define BCX_STOPPED		6	/* button, Stop, fault: left the state */
#define BCX_DECODED_REV		7	/* forward read failed, reverse read decoded */

/* OBSTACLE_BYPASS: a fixed 45 degree dog-leg, run by Mission one Motor
   command at a time (NAV_TURN / NAV_MOVE), so the return leg can stop on
   the line:

        line  |                       side = the side the sweep chose
              |  <- 7. turn back 45: straight along the line again
              |\     (after BYP_PAST_LINE_MM more, so the car is on it)
              | \  6. diagonal back IN, stops when the SECOND IR sensor
              |  |    sees black (STEER_F_DARK_C / _R): the line is then
              |  |    between the two, whichever way the car came
     obstacle |  |  5. turn 45 towards the line
       [##]   |  |  4. straight PAST the obstacle
              |  /  3. turn back 45: parallel to the line
              | /   2. diagonal OUT
              |/    1. turn 45 to the side
             car

   The diagonal would put the car's side BYP_MARGIN_MM clear of the
   obstacle's estimated half width; it is then shortened by BYP_DIAG_TRIM_MM
   (2026-11-18 on the floor: 10 cm too long).  The straight leg brings the
   car's rear past the obstacle (whose depth the sweep cannot see). [CAL] */
#define BYP_TURN_DEG		45	/* the angle the legs are worked out for */
/* The angle actually asked of Motor for each of the four turns.  Motor
   cuts every pivot TURN_COAST_COUNTS early (tuned on 90 degree turns), so a
   45 degree request turned visibly less on the floor (2026-11-18: not far
   enough away from the obstacle).  Asking a bit more lands nearer 45.  All
   four turns use it, so the car still ends up parallel / along the line. [CAL] */
#define BYP_TURN_CMD_DEG	55
#define BYP_MMPS		65	/* speed on every straight leg (50 + 30 %,
					   2026-11-18) [CAL] */
#define BYP_CAR_HALF_MM		75	/* half the car's width [CAL] */
#define BYP_MARGIN_MM		40	/* gap kept beside the obstacle (was 60) [CAL] */
/* Passing on the RIGHT puts the obstacle beside the car's LEFT side, where
   the left IR / barcode sensor pokes out further than the body: keep this
   much more sideways room then (2026-11-18: the right pass was too close). [CAL] */
#define BYP_RIGHT_EXTRA_MM	50
/* Widths passed to mission_bypass_plan() are clamped to this range (the
   bypass is sized from BYP_HALF_xxx below since 2026-11-18). */
#define BYP_WIDTH_MIN_MM	60
#define BYP_WIDTH_MAX_MM	500
#define BYP_OBS_DEPTH_MM	150	/* obstacle length along the track [CAL] */
#define BYP_CAR_LEN_MM		150	/* the rear must pass the obstacle too [CAL] */
#define BYP_PASS_MIN_MM		100	/* shortest straight leg */
#define BYP_PASS_TRIM_MM	100	/* straight leg this much shorter
					   (2026-11-18: 10 cm too long) [CAL] */

/* The outward 45 degree diagonal is sized from the sweep readings: how far
   sideways the obstacle was seen on the chosen side (MSG_OBS.reach_mm =
   reading x sin(angle) of the widest beam that still hit it) is taken as
   its half width there, so the car's body clears that point by
   BYP_MARGIN_MM.  Never less than BYP_HALF_MIN_MM (the diagonal of about
   24 cm that worked on the floor, used when only the centre beam saw it)
   and never more than BYP_HALF_MAX_MM (also used for the forced bypass
   after "no way round").  (2026-11-22, replaces the three fixed sizes.)
   [CAL] */
#define BYP_HALF_MIN_MM		125
#define BYP_HALF_MAX_MM		250
#define BYP_SEARCH_MM		150	/* the return diagonal may run this much
					   further while looking for the line */
#define BYP_DIAG_TRIM_MM	100	/* outward diagonal this much shorter [CAL] */
#define BYP_DIAG_MIN_MM		100	/* ... but never shorter than this */
#define BYP_PAST_LINE_MM	100	/* line found: drive on this far before
					   turning back along it (2026-11-18: 50
					   then 100; the turn came too early) [CAL] */

/* How the last return diagonal ended (dashboard) */
#define BYPR_NONE		0	/* no bypass yet */
#define BYPR_LINE		1	/* stopped on the line (far sensor black) */
#define BYPR_NO_LINE		2	/* ran its full length, no black seen */
#define BYPR_ODOM		3	/* curved back where odometry put the line */

/* "No way round" (OBS_FAIL): the car sweeps again, but after BYP_MAX_FAILS
   in a row (an obstacle dead centre can look blocked on both sides for
   ever) it goes round anyway, on the side the sweep found LESS blocked
   (BYP_DEFAULT_SIDE if it could not tell), assuming the widest obstacle
   (BYP_WIDTH_MAX_MM) so the car keeps well clear. [CAL] */
#define BYP_MAX_FAILS		3
#define BYP_DEFAULT_SIDE	OBS_SIDE_RIGHT

/* OBSTACLE_BYPASS phases, in order */
#define BYPP_OUT_TURN		0
#define BYPP_OUT_DIAG		1
#define BYPP_PAR_TURN		2
#define BYPP_PASS		3
#define BYPP_IN_TURN		4
#define BYPP_IN_DIAG		5
#define BYPP_ALIGN_TURN		6
#define BYPP_STOPPING		7	/* line seen on the return: waiting to stop */
#define BYPP_PAST_LINE		8	/* then BYP_PAST_LINE_MM on, then ALIGN_TURN */
/* Tracking bypass (BYP_TRACK), after OUT_TURN, OUT_DIAG and PAR_TURN: */
#define BYPP_FOLLOW		9	/* drive beside it, BYP_GAP_MM from it */
#define BYPP_CLEAR		10	/* lost it: drive on until the car is past */
#define BYPP_RETURN		11	/* curve in towards the line */
#define BYPP_REJOIN		12	/* line reached: curve back along it */
#define BYPP_OUT		13	/* curve away, sensor on the obstacle's front */
#define BYPP_OUT_CLEAR		14	/* sensor past its edge: gain side margin */
#define BYPP_PASS_STOPPING	15	/* straight leg ended by the side reading */

/* Straight leg of the dog-leg, ended by the obstacle itself (2026-11-22):
   the car drives straight past with the sensor turned as far sideways as
   it goes (BYP_SIDE_AIM_DEG) at the obstacle.  Seen beside the car (within BYP_SIDE_SEEN_MM),
   then BYP_LOST_PINGS readings in a row with nothing there = the sensor has
   passed its far end; the car drives BYP_PASS_AFTER_MM more so its rear
   (and the protruding left sensor) is past too, then turns in.  Never
   under BYP_PASS_MIN_MM nor over BYP_PASS_MAX_MM.  If the side is never
   seen, the planned length (mission_bypass_plan) is used. [CAL] */
#ifndef BYP_SIDE_SENSE
#define BYP_SIDE_SENSE		1
#endif
#define BYP_SIDE_SEEN_MM	350	/* sideways reading that counts as the obstacle */
#define BYP_PASS_AFTER_MM	250	/* sensor (car front) to rear + margin, + 5 cm
					   because at 70 deg the beam loses the end a
					   little early [CAL] */
#define BYP_SIDE_AIM_DEG	70	/* as far sideways as the servo turns */
#define BYP_PASS_MAX_MM		1000

/* Tracking bypass (2026-11-21, no fixed turn since 2026-11-22).  From the
   stop the car curves away while moving (OUT), the servo turning with it so
   the beam keeps looking straight down the track at the obstacle's front:
   the nearer the front, the harder it steers away (no fixed angle; at most
   BYP_OUT_MAX_DEG).  When the beam stops hitting the obstacle the sensor
   is past its edge; the sideways position there (from encoder distance and
   heading) is the edge, and the car carries on until its body is
   BYP_CAR_HALF_MM + BYP_MARGIN_MM (+ BYP_RIGHT_EXTRA_MM on the right)
   beyond it (OUT_CLEAR).  It then follows the obstacle's side with the
   sensor at the calibrated 45 degree position facing it, keeping
   BYP_GAP_MM sideways (reading x sin 45); several pings in a row with
   nothing there means the sensor has passed its far end, then
   BYP_CLEAR_MM more brings the whole car past.  It then curves in to the line at
   BYP_RETURN_DEG, and once the second line IR sensor is on the line it
   curves back gently (no fixed turn that could overshoot onto white),
   telling the line follower which side the line is on.  Every curve is
   steered by the heading from the encoders, measured from the heading at
   the stop (the line's direction).  0 = the fixed dog-leg above. [CAL] */
#ifndef BYP_TRACK
#define BYP_TRACK		0	/* dog-leg chosen for the final design
					   2026-11-22; 1 = tracking bypass */
#endif
#define BYP_GAP_MM		150	/* sensor to obstacle, sideways [CAL] */
#define BYP_OUT_MMPS		40	/* speed while curving away [CAL] */
#define BYP_OUT_STEER		120	/* gentlest curve away (was 250: turned 55
					   deg in 12 cm and lost the obstacle) [CAL] */
#define BYP_OUT_KF_X100		100	/* + steer per mm the front is nearer than
					   BYP_FRONT_REF_MM, x 100 [CAL] */
#define BYP_FRONT_REF_MM	300
#define BYP_OUT_MAX_DEG		35	/* never turned further out than this */
#define BYP_OUT_MIN_DEG		20	/* past the edge: at least this, to move out */
#define BYP_OUT_MAX_MM		800	/* still not round its edge: fault, stop */
#define BYP_AIM_STEP_DEG	5	/* re-aim the servo when off by this much */
#define BYP_LAT_STEP_MM		10	/* sideways position updated per this travel */
#define BYP_FOLLOW_SIDE_DEG	70	/* following: sensor this far round towards
					   it (nearly square-on to its side) */
#define BYP_FOLLOW_AIM_DEG	20	/* following: gap only counted once turned
					   back within this of parallel */
#define BYP_SEEN_MAX_MM		400	/* a reading beyond this (or none) is "nothing
					   there" (sideways 283 mm) */
#define BYP_LOST_PINGS		3	/* that many in a row: passed its end */
#define BYP_NOSEE_MM		300	/* never saw it while following this far:
					   short obstacle, already past */
#define BYP_CLEAR_MM		200	/* then this far on: sensor to the car's
					   rear plus margin [CAL] */
#define BYP_FOLLOW_MAX_MM	1500	/* longer than this: fault, stop */
#define BYP_TOO_CLOSE_MM	70	/* sideways: steer fully away */
#define BYP_KP_X100		300	/* steer per mm of gap error x 100 [CAL] */
#define BYP_KD_X100		600	/* steer per mm of change per ping x 100 [CAL] */
#define BYP_KH			12	/* steer per degree of heading error [CAL] */
#define BYP_STEER_MAX		500	/* while following */
#define BYP_RETURN_DEG		45	/* heading in towards the line [CAL] */
#define BYP_RETURN_STEER	450	/* curve in / back [CAL] */
#define BYP_ONLINE_MM		0	/* RETURN: odometry says back on the line
					   here even if the IR missed it: curve back */
#define BYP_RETURN_MAX_MM	1000	/* no line by then: fault, stop */
#define BYP_ALIGN_DEG		8	/* back within this of the line: follow it */
#define BYP_REJOIN_MAX_MM	400	/* curving back longer than this: follow anyway */
#define BYP_HINT_MS		500	/* line_hint kept this long in LINE_FOLLOW */

typedef struct {
	INT	turn_deg;		/* first turn, + right (= side * BYP_TURN_CMD_DEG) */
	INT	diag_mm;		/* outward diagonal */
	INT	pass_mm;		/* straight leg past the obstacle */
	INT	in_diag_mm;		/* return diagonal, longest it may run */
} BYPASS_PLAN;

/* The dog-leg for an obstacle dist_mm ahead and width_mm wide, passed on
   side (OBS_SIDE_xxx).  search = FALSE: no line to look for, so the return
   diagonal is exactly as long as the outward one. */
IMPORT void mission_bypass_plan(INT side, INT width_mm, INT dist_mm,
				BOOL search, BYPASS_PLAN *p);

/* What the caller must do after an event.  Actions accumulate; the caller
   clears the struct before feeding a batch of events. */
#define MAX_NAV_ACTIONS		4
typedef struct {
	UB	evt_flags;		/* EVT_START | EVT_STOP | EVT_RESUME to set */
	UB	scan_cmd;		/* SCANCMD_xxx, 0 = none */
	INT	scan_arg;		/* SCANCMD_TRACK: servo side */
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
	UB	lifted;			/* stopped because lifted upside down */
	INT	obs_dist_mm;
	INT	obs_width_mm;
	INT	obs_side;
	UB	fault_flags;		/* FAULT_IMU / FAULT_LINE / FAULT_SCAN */
	UB	ir_line;
	UB	net_online;		/* set by the task from app_state */
	INT	bypass_ms;		/* duration of the last bypass */
	UB	byp_phase;		/* OBSTACLE_BYPASS: BYPP_xxx */
	UB	byp_result;		/* last return diagonal: BYPR_xxx */
	UB	track_enabled;		/* BYP_TRACK at start-up; tests set 0 for the dog-leg */
	UB	byp_track;		/* this bypass tracks the obstacle */
	INT	heading_deg;		/* latest heading from the encoders, + right */
	INT	byp_h0;			/* heading at the stop: the line's direction */
	INT	byp_mark_mm;		/* fwd_mm where the current phase began */
	UB	byp_seen;		/* FOLLOW: the obstacle has been seen */
	INT	byp_miss;		/* FOLLOW: pings in a row with nothing there */
	INT	byp_corr;		/* FOLLOW: steer correction from the gap */
	INT	byp_last_err;		/* FOLLOW: previous gap error, mm */
	UB	byp_have_err;
	UB	byp_too_close;
	INT	line_hint;		/* -1 / +1 to the line follower after a bypass */
	INT	byp_lat_mm;		/* sideways from the line, + = bypass side */
	INT	byp_x_mm;		/* along the line from the stop */
	INT	byp_last_fwd;		/* fwd_mm at the previous tick */
	INT	byp_edge_mm;		/* sideways position where the edge was passed */
	INT	byp_front;		/* OUT: latest reading at the front, 0 = none */
	INT	byp_aim;		/* servo angle last asked for, deg - left + right */
	INT	byp_end_fwd;		/* PASS: fwd_mm where the side went clear, -1 = not yet */
	UB	byp_side_sense;		/* this straight leg is ended by the side reading */
	UB	byp_dark_first;		/* return diagonal: STEER_F_DARK_x of the
					   sensor that met the line first, 0 = none */
	BYPASS_PLAN byp;		/* legs of the bypass under way */
	UW	obs_sweeps;		/* sweeps for the current obstacle */
	UB	byp_forced;		/* last bypass started after BYP_MAX_FAILS
					   "no way round" sweeps */
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

/* Heading from the encoders, degrees, + right (counts_to_deg of the wheel
   difference); every tick before mission_on_tick */
IMPORT void mission_on_heading(MISSION *m, INT heading_deg);

/* TEAM.md 7: the turn for a stored letter at a cross junction, in degrees
   (+ right): forward 0, U-turn 180, left -90, right +90.  A letter that is
   not mapped (or 0, nothing stored) goes forward. */
IMPORT INT  mission_junction_deg(const MISSION *m, UB ch);

#endif /* MISSION_LOGIC_H */
