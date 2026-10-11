/*
 *	line_ctrl.h
 *	Line/barcode -- pure line-following maths (no hardware, no kernel calls).
 *
 *	Two sensors, LEFT and CENTRE, straddle the line: the set-point is the
 *	point midway between them, so the sign of the error says which way the
 *	line has moved.  (With only two sensors the line is "lost" when neither
 *	sees it, and a line to the right of the centre sensor cannot be told
 *	apart from "off the line"; the last error direction is used to search.)
 *
 *	*** [CAL] = placeholder, measure on the real track under venue light.
 */

#ifndef LINE_CTRL_H
#define LINE_CTRL_H

/* 12-bit ADC counts for each sensor on white paper / on the black line.
   TCRT5000 modules read HIGHER over black here; set IR_DARK_IS_HIGH to 0 if
   the module is wired the other way round. [CAL] */
#define IR_DARK_IS_HIGH		1
#define IR_WHITE_LEFT		400
#define IR_BLACK_LEFT		3200
#define IR_WHITE_CENTER		400
#define IR_BLACK_CENTER		3200

#define LINE_SEEN_MIN		250	/* dl + dc below this = off the line */
#define LINE_TARGET		(-500)	/* midway between the sensors */
#define LINE_KP			600	/* 1/256 units */
#define LINE_KD			300
#define LINE_LOST_TICKS		5	/* ticks off the line before LINE_LOST */
#define LINE_TIMEOUT_MS		3000	/* lost this long -> report to Mission */
#define LINE_TICK_MS		10	/* NFR-03: sample at least every 10 ms */

typedef struct {
	INT	prev_err;
} LINE_PD;

/* 0 (white) .. 1000 (black) from a raw ADC reading, clamped */
IMPORT INT  ir_darkness(INT raw, INT white, INT black);

/* Position error -500..+500 (negative = line toward the left sensor).
   Returns FALSE and leaves *err alone when neither sensor sees the line. */
IMPORT BOOL line_error(INT dark_left, INT dark_center, INT *err);

/* PD controller: error -> steer -STEER_MAX..+STEER_MAX */
IMPORT void line_pd_init(LINE_PD *pd);
IMPORT INT  line_pd_step(LINE_PD *pd, INT err);

#endif /* LINE_CTRL_H */
