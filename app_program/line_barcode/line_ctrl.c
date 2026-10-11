/*
 *	line_ctrl.c
 *	Line/barcode -- pure line-following maths.  See line_ctrl.h.
 */

#include "line_barcode.h"

EXPORT INT ir_darkness(INT raw, INT white, INT black)
{
	INT span = black - white;
	INT d;

#if !IR_DARK_IS_HIGH
	raw = white + black - raw;
#endif
	if (span == 0) {
		return 0;
	}
	d = ((raw - white) * 1000) / span;
	if (d < 0) {
		d = 0;
	}
	if (d > 1000) {
		d = 1000;
	}
	return d;
}

EXPORT BOOL line_error(INT dark_left, INT dark_center, INT *err)
{
	INT sum = dark_left + dark_center;
	INT pos;

	if (sum < LINE_SEEN_MIN) {
		return FALSE;
	}
	/* weighted position: left sensor at -1000, centre sensor at 0 */
	pos = (-1000 * dark_left) / sum;
	*err = pos - LINE_TARGET;
	return TRUE;
}

EXPORT void line_pd_init(LINE_PD *pd)
{
	pd->prev_err = 0;
}

EXPORT INT line_pd_step(LINE_PD *pd, INT err)
{
	INT d = err - pd->prev_err;
	INT out = (LINE_KP * err + LINE_KD * d) / 256;

	pd->prev_err = err;
	if (out > STEER_MAX) {
		out = STEER_MAX;
	}
	if (out < -STEER_MAX) {
		out = -STEER_MAX;
	}
	return out;
}
