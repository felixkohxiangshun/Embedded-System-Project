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

/* Servo pulse width (us) at each position, measured on the car with
   S5_TEST_SERVO_CAL on 2026-11-22 after the sensor was moved out in front
   of the car.  The car's LEFT is the LARGER pulse.  The servo is not
   symmetric about the centre, and reaches at most 70 degrees each way. */
#define SCAN_US_FAR_LEFT	1910	/* 45 deg left */
#define SCAN_US_NEAR_LEFT	1750	/* 22.5 deg left */
#define SCAN_US_CENTRE		1580
#define SCAN_US_NEAR_RIGHT	1380	/* 22.5 deg right */
#define SCAN_US_FAR_RIGHT	1200	/* 45 deg right */
#define SCAN_US_60_LEFT		2040
#define SCAN_US_60_RIGHT	1080
#define SCAN_US_SIDE_LEFT	2120	/* 70 deg left, the most it turns */
#define SCAN_US_SIDE_RIGHT	980	/* 70 deg right, the most it turns */
#define SCAN_MAX_AIM_DEG	70

/* cos / sin / tan of each position's angle in thousandths (45 and 22.5
   degrees).  Change them with the pulse widths if other angles are used. */
#define SCAN_FAR_COS_PM		707
#define SCAN_FAR_SIN_PM		707
#define SCAN_FAR_TAN_PM		1000
#define SCAN_NEAR_COS_PM	924
#define SCAN_NEAR_TAN_PM	414
#define SCAN_NEAR_SIN_PM	383
#define SCAN_WIDE_COS_PM	342	/* 70 degree beams */
#define SCAN_WIDE_SIN_PM	940
#define SCAN_WIDE_DEG		70	/* the extra sweep readings, each side */

#define SCAN_NO_ECHO		0	/* reading value meaning "no valid echo" */
#define SCAN_NOT_MEASURED	(-1)	/* scan_evaluate_wide: no 70 degree reading */
#define SCAN_SIDE_MARGIN_MM	100	/* openness this much bigger picks that side [CAL] */
#define SCAN_MAX_RANGE_MM	2000	/* an unanswered beam counts as this far */
#define SCAN_HIT_TOL_MM		40	/* tolerance on the flat-face model [CAL] */
#define SCAN_CAR_CLEAR_MM	180	/* room the car needs beside the obstacle [CAL] */

/* How far out to one side the obstacle was still seen (MSG_OBS.extent) */
#define SCAN_EXTENT_CENTRE	0	/* only by the centre beam: narrow */
#define SCAN_EXTENT_NEAR	1	/* the 22.5 degree beam too */
#define SCAN_EXTENT_FAR		2	/* the 45 degree beam too: wide */

typedef struct {
	INT	side;			/* OBS_SIDE_LEFT / OBS_SIDE_RIGHT */
	INT	width_mm;		/* estimated obstacle width */
	INT	clear_left_mm;
	INT	clear_right_mm;
	INT	extent_left;		/* SCAN_EXTENT_xxx on each side */
	INT	extent_right;
	INT	reach_left_mm;		/* how far sideways the obstacle was seen on
					   each side: reading x sin(angle) of the
					   widest beam that still hit it, 0 = only the
					   centre beam */
	INT	reach_right_mm;
} SCAN_RESULT;

/* d_mm[SCAN_FAR_LEFT .. SCAN_FAR_RIGHT] are the readings (0 = no echo).
   Returns FALSE when the centre reading is missing, or neither side has the
   SCAN_CAR_CLEAR_MM of free space the car needs.  In the second case r is
   still filled in, side = the less blocked one. */
IMPORT BOOL scan_evaluate(const INT d_mm[SCAN_ANGLES], SCAN_RESULT *r);
/* The 7-angle sweep: as scan_evaluate, plus the 70 degree readings each side
   (0 = no echo, SCAN_NOT_MEASURED = not taken).  The side is the only one with
   room; with room both sides, the one the obstacle reaches less far; then
   the one whose 45 + 70 degree readings add up to more; then right. */
IMPORT BOOL scan_evaluate_wide(const INT d_mm[SCAN_ANGLES], INT wide_l, INT wide_r,
			       SCAN_RESULT *r);

/* Servo pulse width (us) for reading index SCAN_FAR_LEFT..SCAN_FAR_RIGHT */
IMPORT INT  scan_pulse_us(INT step);

/* Servo pulse width (us) for an angle in degrees, - left / + right, clamped
   to +/-SCAN_MAX_AIM_DEG; linear between the calibrated 0, 22.5, 45, 60
   and 70 degree positions */
IMPORT INT  scan_angle_us(INT deg);

/* ---- Echo pulse capture (HC-SR04 ECHO, filled by the GPIO interrupt) -------
 * Written by the interrupt (echo_edge) and read by scan_task once the event
 * flag says the pulse is complete, so the 32-bit fields are never read half
 * written.  scan_task arms it before the trigger pulse; a rising edge stores
 * the time, the next falling edge completes the width and disarms, so late or
 * stale edges from an earlier ping are ignored.  No kernel calls here. */
#define ECHO_MAX_US		12000	/* about 2 m; a longer pulse means "no object" */

/* APP_NOW_US() is NOT a 1 MHz count on this board.  Nothing starts the
   RP2040 watchdog tick that clocks the timer (the Pico SDK does it in
   clocks_init; this kernel does not), so it counts clk_ref 12 MHz / 512 =
   23.4375 kHz: one count is 512 / 12 = 128 / 3 = 42.67 us.  Measured with
   the debug probe 2026-11-17: 23646 counts in 1 s.  The echo is timed in
   counts and converted here, so a distance is right to about 7 mm (one
   count).  Writing WATCHDOG_TICK = enable | 12 at start-up would make it
   1 MHz, but Line, Barcode and the button debounce were tuned on the slow
   count, so that is a team change. */
#define TIMER_COUNT_US_NUM	128u
#define TIMER_COUNT_US_DEN	3u
#define TIMER_COUNTS_TO_US(c)	(((UW)(c) * TIMER_COUNT_US_NUM) / TIMER_COUNT_US_DEN)
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
