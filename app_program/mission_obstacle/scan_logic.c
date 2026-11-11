/*
 *	scan_logic.c
 *	Mission/obstacle -- pure obstacle profiling.  See scan_logic.h.
 */

#include "mission_obstacle.h"

LOCAL const INT pulse_us[SCAN_ANGLES] = {
	SCAN_US_FAR_LEFT, SCAN_US_NEAR_LEFT, SCAN_US_CENTRE,
	SCAN_US_NEAR_RIGHT, SCAN_US_FAR_RIGHT
};

EXPORT INT scan_pulse_us(INT step)
{
	if (step < 0 || step >= SCAN_ANGLES) {
		return SCAN_US_CENTRE;
	}
	return pulse_us[step];
}

/* Does reading d (at a beam with this cos) still hit a flat face D ahead? */
LOCAL BOOL hits(INT d, INT D, INT cos_pm)
{
	return (d <= (D * 1000) / cos_pm + SCAN_HIT_TOL_MM) ? TRUE : FALSE;
}

/* One side: d_far / d_near are the 45 / 22.5 degree readings.
   Returns the half width of the obstacle on that side and the clearance. */
LOCAL void one_side(INT d_far, INT d_near, INT D, INT *half, INT *clear)
{
	INT lat_far = (D * SCAN_FAR_TAN_PM) / 1000;
	INT lat_near = (D * SCAN_NEAR_TAN_PM) / 1000;

	if (hits(d_far, D, SCAN_FAR_COS_PM)) {
		*half = lat_far;			/* at least this wide */
		*clear = 0;				/* hits right to the sweep edge */
		return;
	}
	if (hits(d_near, D, SCAN_NEAR_COS_PM)) {
		*half = (lat_near + lat_far) / 2;	/* edge between the two beams */
	} else {
		*half = lat_near / 2;			/* edge between centre and near */
	}
	/* lateral room out to whatever the far beam reached, minus the obstacle */
	*clear = (d_far * SCAN_FAR_SIN_PM) / 1000 - *half;
	if (*clear < 0) {
		*clear = 0;
	}
}

EXPORT BOOL scan_evaluate(const INT d_mm[SCAN_ANGLES], SCAN_RESULT *r)
{
	INT d[SCAN_ANGLES];
	INT i, D, half_l, half_r;

	if (d_mm[SCAN_CENTRE] == SCAN_NO_ECHO) {
		return FALSE;			/* nothing straight ahead: no profile */
	}
	for (i = 0; i < SCAN_ANGLES; i++) {
		d[i] = (d_mm[i] == SCAN_NO_ECHO) ? SCAN_MAX_RANGE_MM : d_mm[i];
	}
	D = d[SCAN_CENTRE];

	one_side(d[SCAN_FAR_LEFT], d[SCAN_NEAR_LEFT], D, &half_l, &r->clear_left_mm);
	one_side(d[SCAN_FAR_RIGHT], d[SCAN_NEAR_RIGHT], D, &half_r, &r->clear_right_mm);
	r->width_mm = half_l + half_r;

	if (r->clear_left_mm < SCAN_CAR_CLEAR_MM && r->clear_right_mm < SCAN_CAR_CLEAR_MM) {
		return FALSE;			/* nowhere to go round */
	}
	r->side = (r->clear_right_mm >= r->clear_left_mm) ? OBS_SIDE_RIGHT : OBS_SIDE_LEFT;
	return TRUE;
}

/* ---- echo pulse capture ------------------------------------------------ */

EXPORT void echo_arm(ECHO *e)
{
	e->have_rise = 0;
	e->width_us = 0;
	e->armed = 1;
}

EXPORT void echo_disarm(ECHO *e)
{
	e->armed = 0;
}

EXPORT BOOL echo_edge(ECHO *e, UW now_us, UW edges)
{
	if (!e->armed) {
		return FALSE;			/* not waiting for an echo: stale edge */
	}
	if (edges & ECHO_EDGE_RISE) {
		e->t_rise = now_us;
		e->have_rise = 1;
	}
	if ((edges & ECHO_EDGE_FALL) && e->have_rise) {
		e->width_us = (UW)(now_us - e->t_rise);	/* wrap-safe */
		e->have_rise = 0;
		e->armed = 0;
		return TRUE;
	}
	return FALSE;
}

EXPORT INT echo_us_to_mm(UW width_us)
{
	INT mm;

	if (width_us == 0 || width_us > ECHO_MAX_US) {
		return SCAN_NO_ECHO;
	}
	mm = (INT)((width_us * 343) / 2000);	/* 0.343 mm/us, there and back */
	return (mm < SCAN_MIN_MM) ? SCAN_NO_ECHO : mm;	/* self-echo, not an object */
}
