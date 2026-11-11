/*
 *	scan_logic.h
 *	Mission/obstacle -- pure obstacle profiling (no hardware, no kernel calls).
 *
 *	The servo turns the ultrasonic sensor through five positions (FR-08):
 *	far left, near left, centre, near right, far right, about 45 and 22.5
 *	degrees each side.  Each position is a servo pulse width, not a formula,
 *	so a non-linear servo still points right.  Treating the front of the
 *	obstacle as a flat face at the distance D measured straight ahead, a
 *	beam at angle a would read D / cos(a) if it still hit the obstacle.  A
 *	reading no longer than that (plus a tolerance) is a "hit".  The outermost
 *	hit on each side gives that side's half width; the far beam's reading
 *	beyond the edge gives the clearance.
 *
 *	An estimate from five samples.  NFR-09 asks for width within 20 %;
 *	check the real error on known boxes and tune the constants [CAL].
 */

#ifndef SCAN_LOGIC_H
#define SCAN_LOGIC_H

#define SCAN_ANGLES		5
#define SCAN_FAR_LEFT		0	/* reading index */
#define SCAN_NEAR_LEFT		1
#define SCAN_CENTRE		2
#define SCAN_NEAR_RIGHT		3
#define SCAN_FAR_RIGHT		4

/* Servo pulse width (us) at each position.  This servo is mounted so that
   left is the LARGER pulse.  Far left / centre / far right were measured on
   the car with S5_TEST_SERVO_CAL (2026-11-09); near left / near right are
   the midpoints until they are measured too [CAL]. */
#define SCAN_US_FAR_LEFT	1970
#define SCAN_US_NEAR_LEFT	1815
#define SCAN_US_CENTRE		1660
#define SCAN_US_NEAR_RIGHT	1500
#define SCAN_US_FAR_RIGHT	1340

/* cos / sin / tan of each position's angle in thousandths (45 and 22.5
   degrees).  Change them with the pulse widths if other angles are used. */
#define SCAN_FAR_COS_PM		707
#define SCAN_FAR_SIN_PM		707
#define SCAN_FAR_TAN_PM		1000
#define SCAN_NEAR_COS_PM	924
#define SCAN_NEAR_TAN_PM	414

#define SCAN_NO_ECHO		0	/* reading value meaning "no valid echo" */
#define SCAN_MAX_RANGE_MM	2000	/* an unanswered beam counts as this far */
#define SCAN_HIT_TOL_MM		40	/* tolerance on the flat-face model [CAL] */
#define SCAN_CAR_CLEAR_MM	180	/* room the car needs beside the obstacle [CAL] */

typedef struct {
	INT	side;			/* OBS_SIDE_LEFT / OBS_SIDE_RIGHT */
	INT	width_mm;		/* estimated obstacle width */
	INT	clear_left_mm;
	INT	clear_right_mm;
} SCAN_RESULT;

/* d_mm[SCAN_FAR_LEFT .. SCAN_FAR_RIGHT] are the readings (0 = no echo).
   Returns FALSE when the centre reading is missing, or neither side has the
   SCAN_CAR_CLEAR_MM of free space the car needs. */
IMPORT BOOL scan_evaluate(const INT d_mm[SCAN_ANGLES], SCAN_RESULT *r);

/* Servo pulse width (us) for reading index SCAN_FAR_LEFT..SCAN_FAR_RIGHT */
IMPORT INT  scan_pulse_us(INT step);

/* ---- Echo pulse capture (HC-SR04 ECHO, filled by the GPIO interrupt) -------
 * Written by the interrupt (echo_edge) and read by scan_task once the event
 * flag says the pulse is complete, so the 32-bit fields are never read half
 * written.  scan_task arms it before the trigger pulse; a rising edge stores
 * the time, the next falling edge completes the width and disarms, so late or
 * stale edges from an earlier ping are ignored.  No kernel calls here. */
#define ECHO_MAX_US		12000	/* about 2 m; a longer pulse means "no object" */
/* Shorter readings are not trusted.  The HC-SR04 cannot measure below about
   20-30 mm, and on the car (2026-11-11) it read 13-38 mm with nothing in
   front: echoes off its own mount or the car.  Obstacles are detected at
   300 mm, so nothing real is ever this close. [CAL] */
#define SCAN_MIN_MM		40

typedef struct {
	volatile UB	armed;		/* 1 = accepting edges */
	volatile UB	have_rise;	/* 1 = rising edge seen, waiting for the fall */
	volatile UW	t_rise;		/* APP_NOW_US() at the rising edge */
	volatile UW	width_us;	/* valid once echo_edge() returned TRUE */
} ECHO;

IMPORT void echo_arm(ECHO *e);
IMPORT void echo_disarm(ECHO *e);
/* Feed one interrupt: now_us from the 1 MHz timer, edges = ECHO_EDGE_xxx bits.
   Returns TRUE when the pulse is complete (width_us is valid). */
IMPORT BOOL echo_edge(ECHO *e, UW now_us, UW edges);
/* Pulse width -> mm, or SCAN_NO_ECHO for 0, longer than ECHO_MAX_US, or a
   distance under SCAN_MIN_MM */
IMPORT INT  echo_us_to_mm(UW width_us);

#endif /* SCAN_LOGIC_H */
