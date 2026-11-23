/*
 *	scan_logic.c
 *	Mission/obstacle -- pure obstacle profiling.  See scan_logic.h.
 */

#include "mission_obstacle.h"

LOCAL const INT pulse_us[SCAN_ANGLES] = {
	SCAN_US_FAR_LEFT, SCAN_US_NEAR_LEFT, SCAN_US_CENTRE,
	SCAN_US_NEAR_RIGHT, SCAN_US_FAR_RIGHT
};

EXPORT INT scan_angle_us(INT deg)
{
	/* calibrated points, tenths of a degree */
	static const INT at[5] = { 0, 225, 450, 600, 700 };
	INT us[5];
	INT a = ((deg < 0) ? -deg : deg) * 10;
	INT i;

	us[0] = SCAN_US_CENTRE;
	us[1] = (deg < 0) ? SCAN_US_NEAR_LEFT : SCAN_US_NEAR_RIGHT;
	us[2] = (deg < 0) ? SCAN_US_FAR_LEFT : SCAN_US_FAR_RIGHT;
	us[3] = (deg < 0) ? SCAN_US_60_LEFT : SCAN_US_60_RIGHT;
	us[4] = (deg < 0) ? SCAN_US_SIDE_LEFT : SCAN_US_SIDE_RIGHT;
	if (a >= at[4]) {
		return us[4];
	}
	for (i = 1; a > at[i]; i++) {
		;
	}
	return us[i - 1] + ((us[i] - us[i - 1]) * (a - at[i - 1])) / (at[i] - at[i - 1]);
}

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
   Returns the half width of the obstacle on that side, the clearance, and
   how far out the obstacle reached (SCAN_EXTENT_xxx). */
LOCAL void one_side(INT d_wide, INT d_far, INT d_near, INT D, INT *half, INT *clear,
		    INT *extent, INT *reach)
{
	INT lat_far = (D * SCAN_FAR_TAN_PM) / 1000;
	INT lat_near = (D * SCAN_NEAR_TAN_PM) / 1000;
	INT side_far = (d_far * SCAN_FAR_SIN_PM) / 1000;
	INT side_wide = (d_wide == SCAN_NOT_MEASURED) ? 0
			: (d_wide * SCAN_WIDE_SIN_PM) / 1000;

	if (hits(d_far, D, SCAN_FAR_COS_PM)) {
		*half = lat_far;			/* at least this wide */
		*extent = SCAN_EXTENT_FAR;
		*reach = side_far;
		/* the 70 degree beam: open there = room past the edge */
		if (d_wide == SCAN_NOT_MEASURED || hits(d_wide, D, SCAN_WIDE_COS_PM)) {
			*clear = 0;			/* hits right to the sweep edge */
		} else {
			*clear = side_wide - *half;
		}
	} else {
		if (hits(d_near, D, SCAN_NEAR_COS_PM)) {
			*half = (lat_near + lat_far) / 2;	/* edge between the two beams */
			*extent = SCAN_EXTENT_NEAR;
			*reach = (d_near * SCAN_NEAR_SIN_PM) / 1000;
		} else {
			*half = lat_near / 2;			/* edge between centre and near */
			*extent = SCAN_EXTENT_CENTRE;
			*reach = 0;
		}
		/* lateral room out to whatever the outer beams reached, minus the obstacle */
		*clear = ((side_wide > side_far) ? side_wide : side_far) - *half;
	}
	if (*clear < 0) {
		*clear = 0;
	}
}

/* How open one side looks: the two outer readings added (an unanswered beam
   counts as SCAN_MAX_RANGE_MM).  Only the outer beams: the ultrasonic cone is
   about 30 degrees wide, so the 22.5 degree beams nearly always see the
   obstacle whichever side is free. */
LOCAL INT openness(INT d_wide, INT d_far)
{
	return d_far + ((d_wide == SCAN_NOT_MEASURED) ? 0 : d_wide);
}

EXPORT BOOL scan_evaluate_wide(const INT d_mm[SCAN_ANGLES], INT wide_l, INT wide_r,
			       SCAN_RESULT *r)
{
	INT d[SCAN_ANGLES];
	INT i, D, half_l, half_r, open_l, open_r;
	BOOL room_l, room_r;

	if (d_mm[SCAN_CENTRE] == SCAN_NO_ECHO) {
		return FALSE;			/* nothing straight ahead: no profile */
	}
	for (i = 0; i < SCAN_ANGLES; i++) {
		d[i] = (d_mm[i] == SCAN_NO_ECHO) ? SCAN_MAX_RANGE_MM : d_mm[i];
	}
	if (wide_l == SCAN_NO_ECHO) {
		wide_l = SCAN_MAX_RANGE_MM;
	}
	if (wide_r == SCAN_NO_ECHO) {
		wide_r = SCAN_MAX_RANGE_MM;
	}
	D = d[SCAN_CENTRE];

	one_side(wide_l, d[SCAN_FAR_LEFT], d[SCAN_NEAR_LEFT], D, &half_l, &r->clear_left_mm,
		 &r->extent_left, &r->reach_left_mm);
	one_side(wide_r, d[SCAN_FAR_RIGHT], d[SCAN_NEAR_RIGHT], D, &half_r, &r->clear_right_mm,
		 &r->extent_right, &r->reach_right_mm);
	r->width_mm = half_l + half_r;

	room_l = (r->clear_left_mm >= SCAN_CAR_CLEAR_MM) ? TRUE : FALSE;
	room_r = (r->clear_right_mm >= SCAN_CAR_CLEAR_MM) ? TRUE : FALSE;
	open_l = openness(wide_l, d[SCAN_FAR_LEFT]);
	open_r = openness(wide_r, d[SCAN_FAR_RIGHT]);

	/* The side to go: the only one with room, else the more open one.
	   Set even when there is no room: the forced bypass after repeated
	   "no way round" (Mission) uses it. */
	if (room_l != room_r) {
		r->side = room_l ? OBS_SIDE_LEFT : OBS_SIDE_RIGHT;
	} else if (room_l && r->extent_left != r->extent_right) {
		/* both free: the side the obstacle reaches less far (shorter way) */
		r->side = (r->extent_left < r->extent_right) ? OBS_SIDE_LEFT : OBS_SIDE_RIGHT;
	} else if (open_l > open_r + SCAN_SIDE_MARGIN_MM) {
		r->side = OBS_SIDE_LEFT;
	} else {
		r->side = OBS_SIDE_RIGHT;	/* more open, or about the same */
	}

	if (!room_l && !room_r) {
		return FALSE;			/* nowhere to go round */
	}
	return TRUE;
}

EXPORT BOOL scan_evaluate(const INT d_mm[SCAN_ANGLES], SCAN_RESULT *r)
{
	return scan_evaluate_wide(d_mm, SCAN_NOT_MEASURED, SCAN_NOT_MEASURED, r);
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
