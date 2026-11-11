/*
 *	app_if.h
 *	Interfaces between subsystems: message formats, event-flag bits and the
 *	IDs of the kernel objects (design review section 4, task interaction
 *	diagram).  Included by app.h.  Changed only by team agreement; every
 *	row is mirrored in TEAM.md section 5.
 *
 *	Rule: the subsystem that RECEIVES or OWNS the data creates the object
 *	(<subsystem>_init) and the ID is declared here.  Everyone else only
 *	sends to it or waits on it.  Tasks never call each other directly.
 *
 *	Why message buffers and not queues: micro T-Kernel has no data queue.
 *	A message buffer copies the message in and out (no memory ownership to
 *	manage) and its bounded size gives back-pressure.  A mailbox only passes
 *	a pointer, which would force a memory pool on every path.
 */

#ifndef APP_IF_H
#define APP_IF_H

/* =====================================================================
 *  Messages (all fixed-size structs, copied by the message buffer)
 * ===================================================================== */

/* IF-01  Line -> Mission : MBF_STEER ------------------------------------ */
#define STEER_MAX		1000	/* steer is -1000 (hard left) .. +1000 */
#define STEER_F_LOST		0x01	/* both sensors are off the line */
#define STEER_F_TIMEOUT		0x02	/* line lost longer than the timeout */
#define STEER_F_JUNCTION	0x04	/* side branch found, sent once per branch */
typedef struct {
	INT	steer;			/* -1000..+1000, positive = steer right */
	UB	flags;			/* STEER_F_xxx */
} MSG_STEER;

/* IF-02  1 kHz handler -> Barcode : MBF_EDGE ---------------------------- */
typedef struct {
	UW	t_us;			/* APP_NOW_US() at the edge */
	UB	dark;			/* 1 = sensor now sees a dark bar */
} MSG_EDGE;

/* IF-03  Barcode -> Mission : MBF_BCCMD --------------------------------- */
#define BC_SEEN			1	/* start bar seen, scanning */
#define BC_OK			2	/* decoded, cmd is valid */
#define BC_FAIL			3	/* bad width or timeout */
#define BC_FIRST		4	/* a code starts: second dark bar close behind the first */
typedef struct {
	UB	kind;			/* BC_xxx */
	UB	cmd;			/* 'A'..'D' when kind == BC_OK */
} MSG_BCCMD;

/* IF-04  IMU -> Mission : MBF_TERRAIN ----------------------------------- */
#define TERR_HUMP_START		1	/* Z rose above the hump threshold */
#define TERR_HUMP_DONE		2	/* Z back at baseline, height valid */
#define TERR_SHOCK		3	/* > 3 g impulse shorter than 3 samples */
#define TERR_FAULT		4	/* I2C read failed (terrain data lost) */
#define TERR_RECOVERED		5	/* sensor answers again */
typedef struct {
	UB	kind;			/* TERR_xxx */
	INT	height_mm;		/* TERR_HUMP_DONE: this hump */
	INT	max_mm;			/* TERR_HUMP_DONE: tallest of the run */
} MSG_TERRAIN;

/* IF-05  Scan -> Mission : MBF_OBS -------------------------------------- */
#define OBS_DETECTED		1	/* object within 30 cm, waiting for stop */
#define OBS_RESULT		2	/* profile done, side and sizes valid */
#define OBS_FAIL		3	/* no echo or no side with room */
#define OBS_SIDE_LEFT		(-1)
#define OBS_SIDE_RIGHT		(+1)
typedef struct {
	UB	kind;			/* OBS_xxx */
	INT	side;			/* OBS_SIDE_xxx, chosen bypass side */
	INT	dist_mm;		/* distance when first seen */
	INT	width_mm;		/* estimated obstacle width */
	INT	clear_mm;		/* clearance on the chosen side */
} MSG_OBS;

/* IF-06  Mission -> Scan : MBF_SCANCMD ---------------------------------- */
#define SCANCMD_ARM		1	/* car is driving: ping ahead */
#define SCANCMD_DISARM		2	/* stop pinging (idle, e-stop, ...) */
#define SCANCMD_PROFILE		3	/* car has stopped: sweep and measure */
typedef struct {
	UB	cmd;			/* SCANCMD_xxx */
} MSG_SCANCMD;

/* IF-07  Dashboard -> Mission : MBF_MQTTCMD ----------------------------- */
#define MQTTCMD_ESTOP		1
#define MQTTCMD_RESUME		2
typedef struct {
	UB	cmd;			/* MQTTCMD_xxx */
} MSG_MQTTCMD;

/* IF-08  Mission -> Motor : MBF_NAV ------------------------------------- */
#define NAV_STEER		1	/* arg = steer -1000..+1000 (only while
					   Mission is in a line-following state) */
#define NAV_SLOW		2	/* drive at the slow speed */
#define NAV_TURN		3	/* arg = degrees, + = right, - = left */
#define NAV_BYPASS		4	/* arg = side (OBS_SIDE_xxx), arg2 = width mm */
#define NAV_RESUME		5	/* back to normal speed */
#define NAV_MOVE		6	/* arg = mm straight, - = backwards; arg2 =
					   speed mm/s (0 = SPEED_MOVE_MMPS); ends
					   in STOPPED like a turn */
typedef struct {
	UB	type;			/* NAV_xxx */
	INT	arg;
	INT	arg2;
} MSG_NAV;

/* IF-09  Telemetry -> Dashboard : MBF_TELEM ----------------------------- */
typedef struct {
	UB	sys_state;		/* SYS_xxx */
	INT	speed_mmps;
	UW	dist_mm;
	INT	enc_left;
	INT	enc_right;
	INT	heading_deg;		/* dead-reckoned from the encoders, + right */
	UB	ir_line;		/* 1 = line seen */
	INT	obs_dist_mm;
	UB	last_barcode;		/* 'A'..'Z' or 0 */
	INT	hump_max_mm;
	UB	shock;
	UB	fault_flags;		/* FAULT_xxx */
	UW	uptime_ms;
} MSG_TELEM;

/* =====================================================================
 *  Event flag FLG_EVT  (IF-10)
 *  Mission sets START / STOP / RESUME and the Motor task waits on them.
 *  The button ISR sets BTN for Mission.  Waiters use TWF_BITCLR, so each
 *  clears only the bits it waited for.
 * ===================================================================== */
#define EVT_START		0x01
#define EVT_STOP		0x02
#define EVT_RESUME		0x04
#define EVT_BTN			0x08	/* BTN_START pressed (debounced in ISR) */
#define EVT_BC_HOLD		0x10	/* Mission -> Barcode: drop the capture and
					   ignore edges (the car backs up) */
#define EVT_BC_ARM		0x20	/* Mission -> Barcode: start a fresh capture */

/* =====================================================================
 *  System state  (system statechart, design review section 5)
 * ===================================================================== */
#define SYS_INIT		0
#define SYS_IDLE		1
#define SYS_LINE_FOLLOW		2	/* RUNNING (initial sub-state) */
#define SYS_BARCODE_SCAN	3	/* RUNNING */
#define SYS_BARCODE_CMD		4	/* RUNNING */
#define SYS_HUMP_TRAVERSE	5	/* RUNNING */
#define SYS_OBSTACLE_SCAN	6	/* RUNNING */
#define SYS_OBSTACLE_BYPASS	7	/* RUNNING */
#define SYS_LINE_REACQUIRE	8	/* RUNNING */
#define SYS_FAULT_RECOVERY	9
#define SYS_E_STOP		10

/* Motor statechart (section 6.3) */
#define MOT_STOPPED		0
#define MOT_DRIVING		1
#define MOT_TURNING		2
#define MOT_BYPASS		3
#define MOT_ENCODER_FAULT	4

#define FAULT_IMU		0x01
#define FAULT_ENCODER		0x02
#define FAULT_LINE		0x04
#define FAULT_SCAN		0x08
#define FAULT_MOTOR_STALL	0x10	/* Dave: wheel driven but not turning */
#define FAULT_WIFI		0x20	/* Jun Kai: WiFi or broker link lost */
#define FAULT_HUMP_RANGE	0x40	/* Sook Yee: hump above the measurable range */
#define FAULT_MISSION_TMO	0x80	/* Dave: no MSG_NAV from Mission in time */

/* One writer per bit.  Each writer replaces only its own bits. */
#define FAULT_OWN_MOTOR		(FAULT_ENCODER | FAULT_MOTOR_STALL | FAULT_MISSION_TMO)
#define FAULT_OWN_DASH		(FAULT_WIFI)
#define FAULT_OWN_MISSION	(FAULT_IMU | FAULT_LINE | FAULT_SCAN | FAULT_HUMP_RANGE)

/* =====================================================================
 *  Shared state  MTX_STATE  (IF-11)
 *  Written by Mission (state and sensor results) and by Motor (speed,
 *  distance, motor state, completion counter).  Read by Telemetry, and by
 *  Mission to see that a Motor command has finished.  Always access it
 *  through state_lock()/state_unlock() (mission_obstacle.h).
 * ===================================================================== */
typedef struct {
	/* written by Mission */
	UB	sys_state;		/* SYS_xxx */
	UB	last_barcode;		/* last letter read, 'A'..'Z' or 0 */
	UB	stored_char;		/* letter kept for the next junction, or 0 */
	INT	hump_mm;
	INT	hump_max_mm;
	UB	shock;
	INT	obs_dist_mm;
	INT	obs_width_mm;
	INT	obs_side;
	UB	fault_flags;		/* FAULT_xxx */
	UB	ir_line;
	/* written by Dashboard */
	UB	net_online;		/* 1 = WiFi and broker connected */
	/* written by Motor */
	UB	motor_state;		/* MOT_xxx */
	INT	speed_mmps;		/* mean of both wheels */
	UW	dist_mm;		/* path of both wheels, either direction */
	INT	fwd_mm;			/* signed forward travel: mean of the two
					   wheels, so steering and pivots add 0 */
	INT	enc_left;
	INT	enc_right;
	UW	cmd_done_seq;		/* +1 each time TURN/BYPASS completes */
} APP_STATE;

/* =====================================================================
 *  Kernel object IDs.  Declared here, created by the owner (TEAM.md 5).
 * ===================================================================== */
IMPORT ID mbf_nav;		/* S1 owns: Mission -> Motor            */
IMPORT ID flg_evt;		/* S1 owns: Mission/ISR -> Motor/Mission */
IMPORT ID mbf_edge;		/* S2 owns: 1 kHz handler -> Barcode     */
IMPORT ID mbf_steer;		/* S5 owns: Line -> Mission              */
IMPORT ID mbf_bccmd;		/* S5 owns: Barcode -> Mission           */
IMPORT ID mbf_terrain;		/* S5 owns: IMU -> Mission               */
IMPORT ID mbf_obs;		/* S5 owns: Scan -> Mission              */
IMPORT ID mbf_scancmd;		/* S5 owns: Mission -> Scan              */
IMPORT ID mbf_mqttcmd;		/* S5 owns: Dashboard -> Mission         */
IMPORT ID mbf_telem;		/* S4 owns: Telemetry -> Dashboard       */
IMPORT ID mtx_state;		/* S5 owns: protects app_state           */
IMPORT APP_STATE app_state;

/* Lock/unlock app_state (owned by the Mission/obstacle subsystem).  Use TMO_POL in control
   loops that must not wait: if the lock is busy, skip this update. */
IMPORT ER state_lock(TMO tmout);
IMPORT void state_unlock(void);

/* Replace the caller's own fault bits (own_mask) with new_bits and leave every
   other owner's bits untouched.  The caller must hold state_lock(): the
   read-modify-write is then never interleaved with another writer. */
static inline void fault_write_locked(UB own_mask, UB new_bits)
{
	app_state.fault_flags = (UB)((app_state.fault_flags & (UB)~own_mask) |
				      (new_bits & own_mask));
}

#endif /* APP_IF_H */
