/*
 *	scan_logic.c
 *	Mission/obstacle -- pure obstacle profiling.  See scan_logic.h.
 */

#include "mission_obstacle.h"

/* index:                        -60  -30    0   +30  +60 */
LOCAL const INT cos_pm[5]  = {   500, 866, 1000, 866, 500 };
LOCAL const INT tan_pm[5]  = {  1732, 577,    0, 577, 1732 };
LOCAL const INT sin_pm[5]  = {   866, 500,    0, 500, 866 };

EXPORT INT scan_angle_deg(INT step)
{
	return (step - 2) * 30;
}

/* One side: idx_far is the +/-60 beam, idx_near the +/-30 beam.
   Returns the half width of the obstacle on that side and the clearance. */
LOCAL void one_side(const INT *d, const BOOL *hit, INT D, INT idx_far,
		    INT idx_near, INT *half, INT *clear)
{
	INT lat_far = (D * tan_pm[idx_far]) / 1000;
	INT lat_near = (D * tan_pm[idx_near]) / 1000;

	if (hit[idx_far]) {
		*half = lat_far;			/* at least this wide */
		*clear = 0;				/* hits right to the sweep edge */
		return;
	}
	if (hit[idx_near]) {
		*half = (lat_near + lat_far) / 2;	/* edge lies between the beams */
	} else {
		*half = lat_near / 2;			/* edge between centre and 30 degrees */
	}
	/* lateral room out to whatever the +/-60 beam reached, minus the obstacle */
	*clear = (d[idx_far] * sin_pm[idx_far]) / 1000 - *half;
	if (*clear < 0) {
		*clear = 0;
	}
}

EXPORT BOOL scan_evaluate(const INT d_mm[SCAN_ANGLES], SCAN_RESULT *r)
{
	INT  d[SCAN_ANGLES];
	BOOL hit[SCAN_ANGLES];
	INT  i, D, half_l, half_r;

	if (d_mm[2] == SCAN_NO_ECHO) {
		return FALSE;			/* nothing straight ahead: no profile */
	}
	for (i = 0; i < SCAN_ANGLES; i++) {
		d[i] = (d_mm[i] == SCAN_NO_ECHO) ? SCAN_MAX_RANGE_MM : d_mm[i];
	}
	D = d[2];
	for (i = 0; i < SCAN_ANGLES; i++) {
		hit[i] = (d[i] <= (D * 1000) / cos_pm[i] + SCAN_HIT_TOL_MM) ? TRUE : FALSE;
	}

	one_side(d, hit, D, 0, 1, &half_l, &r->clear_left_mm);
	one_side(d, hit, D, 4, 3, &half_r, &r->clear_right_mm);
	r->width_mm = half_l + half_r;

	if (r->clear_left_mm < SCAN_CAR_CLEAR_MM && r->clear_right_mm < SCAN_CAR_CLEAR_MM) {
		return FALSE;			/* nowhere to go round */
	}
	r->side = (r->clear_right_mm >= r->clear_left_mm) ? OBS_SIDE_RIGHT : OBS_SIDE_LEFT;
	return TRUE;
}
