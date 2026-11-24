/*
 *	scan_logic.h
 *	Mission/obstacle -- pure obstacle profiling (no hardware, no kernel calls).
 *
 *	The servo turns the ultrasonic sensor from 55 degrees left to 55 degrees
 *	right in 5 degree steps (FR-08); scan_profile() turns the readings into
 *	the obstacle's width and the side to go round (see below).  The servo
 *	is aimed through calibrated pulse widths (scan_angle_us), so a
 *	non-linear servo still points right.
 */

#ifndef SCAN_LOGIC_H
#define SCAN_LOGIC_H

/* Servo pulse width (us) at each calibrated angle, measured on the car with
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
#define SCAN_US_SIDE_LEFT	2180	/* 70 deg left (2026-11-25: was 2120, only
					   +540 us against the right's -600; the
					   bypass's left side reading saw nothing) [CAL] */
#define SCAN_US_SIDE_RIGHT	980	/* 70 deg right, the most it turns */
#define SCAN_MAX_AIM_DEG	70

#define SCAN_NO_ECHO		0	/* reading value meaning "no valid echo" */
#define SCAN_MAX_RANGE_MM	2000	/* an unanswered beam counts as this far */

/* ---- The sweep and the obstacle profile --------------------------------
 * (2026-11-24: the calculation of the INF2004 group LordPercivalXII,
 * app_program/subsystem5/obstacle_geometry.c, on this car's sweep.)
 * The servo reads straight ahead, steps out to SCAN_SWEEP_DEG left, returns
 * to the middle, then steps out to SCAN_SWEEP_DEG right, in SCAN_STEP_DEG10
 * steps (each reading a median of three).  Then:
 *  - the obstacle is the nearest reading plus every neighbouring reading
 *    within SCAN_BAND_MM of it; its edges are where that run ends;
 *  - each edge angle is moved half a step out and half the beam
 *    (SCAN_BEAM_DEG) in: the beam starts to see an edge half a beam
 *    before pointing at it;
 *  - the face is taken square to the direction of its centre, at the
 *    closest reading's distance corrected to that centre; width = face
 *    distance x (tan right edge - tan left edge), both from the centre;
 *  - the clearance on each side is the nearest reading beyond that edge;
 *    an obstacle running past the sweep's end on a side is "unseen, not
 *    blocked" there (edge out);
 *  - the sideways move to clear each edge is SCAN_PATH_HALF_MM (half the
 *    car plus 4 cm) beyond the edge's sideways position; a side has room
 *    if its clearance is more than that move (kept between
 *    SCAN_OFFSET_MIN_MM and SCAN_OFFSET_MAX_MM) plus SCAN_CLEAR_MARGIN_MM,
 *    or if its edge is out;
 *  - side: the one with room; with room on both, the smaller move. */
/* 2026-11-25: 5 deg steps over +-55 deg (was +-70 deg; nothing useful
   was heard beyond about 55 deg).  2.5 deg steps were tried and dropped. */
#define SCAN_STEP_DEG10		50	/* the step, tenths of a degree */
#define SCAN_SWEEP_DEG		55	/* the sweep: +-55 deg */
#define SCAN_STEPS		23	/* -55 .. +55 every 5 deg */
#define SCAN_STEP_CENTRE	11	/* index of 0 deg; index i = (i - 11) x 5 deg */
#define SCAN_BAND_MM		30	/* readings this close to the nearest: the obstacle */
#define SCAN_BEAM_DEG		15	/* the beam's width */
#define SCAN_PATH_HALF_MM	115	/* half the car (7.5 cm) + 4 cm clearance */
#define SCAN_OFFSET_MIN_MM	100
#define SCAN_OFFSET_MAX_MM	300	/* stays on the track mat */
#define SCAN_CLEAR_MARGIN_MM	40
#define SCAN_WIDTH_MAX_MM	1023	/* sent in 10 bits */

/* Calibration of the width on this car (2026-11-25).  The sensor hears an
   obstacle far past its edges, so the method above reads it too wide, but
   in step with its real width.  Centred boxes at the first sweep
   (29-30 cm): 8 cm read 436, 20 cm 521 (twice), 40 cm 637; a straight line
   through them, real = 1.59 x read - 614, is within 15 mm of all three;
   it still read high on the car, so 100 mm more was taken off, then 50 mm
   given back (- 664).
   Measured on a +-70 deg sweep, now +-55 deg: recheck on known boxes.
   It only holds at that distance: outside SCAN_CAL_D_MIN_MM ..
   SCAN_CAL_D_MAX_MM (e.g. after creeping closer) the width is left as
   read.  The half widths are scaled with it. [CAL] */
#define SCAN_WIDTH_GAIN_PM	1590	/* x 1000 */
#define SCAN_WIDTH_OFFSET_MM	664
#define SCAN_CAL_D_MIN_MM	250
#define SCAN_CAL_D_MAX_MM	330

typedef struct {
	INT	side;			/* OBS_SIDE_LEFT / OBS_SIDE_RIGHT */
	INT	width_mm;		/* face distance x (tan right - tan left) */
	INT	half_left_mm;		/* sideways from the centre line to each edge
					   (MSG_OBS.reach_mm on the chosen side) */
	INT	half_right_mm;
	BOOL	out_left;		/* the obstacle runs past the sweep's end */
	BOOL	out_right;
	INT	clear_left_mm;		/* nearest reading beyond each edge, mm */
	INT	clear_right_mm;
} SCAN_RESULT;

/* Angle of sweep step i, degrees (- left, + right) */
#define SCAN_STEP_DEG10_OF(i)	(((i) - SCAN_STEP_CENTRE) * SCAN_STEP_DEG10)	/* 0.1 deg */

/* mm[0 .. SCAN_STEPS-1]: the sweep, 55 left to 55 right (0 = no echo or
   not read).  Returns FALSE when the centre
   reading is missing, or neither side has room; r is filled in either way
   (side = the one needing the smaller move). */
IMPORT BOOL scan_profile(const INT mm[SCAN_STEPS], SCAN_RESULT *r);

/* Correct r's width (and its half widths, in proportion) with the
   calibration above when the obstacle is D_mm ahead, D within the
   calibrated range.  Returns TRUE if it was corrected. */
IMPORT BOOL scan_width_calibrated(SCAN_RESULT *r, INT D_mm);

/* tan, cos and sin of 0..90 degrees (tenths), x 1000 (scan_stan_pm:
   signed, -90..90) */
IMPORT INT  scan_tan_pm(INT deg10);
IMPORT INT  scan_cos_pm(INT deg10);
IMPORT INT  scan_sin_pm(INT deg10);
IMPORT INT  scan_stan_pm(INT deg10);

/* Servo pulse width (us) for an angle in degrees, - left / + right, clamped
   to +/-SCAN_MAX_AIM_DEG; linear between the calibrated 0, 22.5, 45, 60
   and 70 degree positions.  scan_angle10_us: the angle in tenths. */
IMPORT INT  scan_angle_us(INT deg);
IMPORT INT  scan_angle10_us(INT deg10);

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
/* Feed one interrupt: now_us = APP_NOW_US() (timer counts), edges = ECHO_EDGE_xxx bits.
   Returns TRUE when the pulse is complete (width_us is valid). */
IMPORT BOOL echo_edge(ECHO *e, UW now_us, UW edges);
/* Pulse width -> mm, or SCAN_NO_ECHO for 0, longer than ECHO_MAX_US, or a
   distance under SCAN_MIN_MM */
IMPORT INT  echo_us_to_mm(UW width_us);

#endif /* SCAN_LOGIC_H */
