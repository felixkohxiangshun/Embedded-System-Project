/*
 *	scan_logic.h
 *	Mission/obstacle -- pure obstacle profiling (no hardware, no kernel calls).
 *
 *	The servo turns the ultrasonic sensor through five angles
 *	(-60, -30, 0, +30, +60 degrees, FR-08).  Treating the front of the
 *	obstacle as a flat face at the distance D measured straight ahead, a
 *	beam at angle a would read D / cos(a) if it still hit the obstacle.
 *	A reading no longer than that (plus a tolerance) is a "hit"; a longer one
 *	means the beam passed the edge.  The outermost hits on each side give the
 *	obstacle's half widths; the free reading beyond gives the clearance.
 *
 *	This is an estimate from five samples.  NFR-09 asks for width within
 *	20 %; check the real error on known boxes and tune the constants [CAL].
 */

#ifndef SCAN_LOGIC_H
#define SCAN_LOGIC_H

#define SCAN_ANGLES		5
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

/* d_mm[0..4] are the readings at -60, -30, 0, +30, +60 degrees (0 = no echo).
   Returns FALSE when the centre reading is missing, or neither side has the
   SCAN_CAR_CLEAR_MM of free space the car needs. */
IMPORT BOOL scan_evaluate(const INT d_mm[SCAN_ANGLES], SCAN_RESULT *r);

/* Servo angle (degrees, - = left) for sweep step 0..4 */
IMPORT INT  scan_angle_deg(INT step);

#endif /* SCAN_LOGIC_H */
