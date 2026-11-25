/**
 * @file scan_logic.c
 *      Mission/obstacle -- pure obstacle profiling.  See scan_logic.h.
 */

#include "mission_obstacle.h"

EXPORT INT scan_angle_us(INT deg)
{
    return scan_angle10_us(deg * 10);
}

EXPORT INT scan_angle10_us(INT deg10)
{
    /* calibrated points, tenths of a degree */
    static const INT at[5] = { 0, 225, 450, 600, SCAN_MAX_AIM_DEG * 10 };
    INT us[5];
    INT deg = deg10;
    INT a = (deg10 < 0) ? -deg10 : deg10;
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
        return FALSE;                   /* not waiting for an echo: stale edge */
    }
    if (edges & ECHO_EDGE_RISE) {
        e->t_rise = now_us;
        e->have_rise = 1;
    }
    if ((edges & ECHO_EDGE_FALL) && e->have_rise) {
        e->width_us = (UW)(now_us - e->t_rise); /* wrap-safe */
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
    mm = (INT)((width_us * 343) / 2000);        /* 0.343 mm/us, there and back */
    return (mm < SCAN_MIN_MM) ? SCAN_NO_ECHO : mm;      /* self-echo, not an object */
}

/* ---- Measured obstacle width -------------------------------------------- */

/* tan and cos x 1000 every 5 degrees, 0..90 (linear in between) */
LOCAL const INT tan5[19] = { 0, 87, 176, 268, 364, 466, 577, 700, 839, 1000,
                  1192, 1428, 1732, 2145, 2747, 3732, 5671, 11430, 57290 };
LOCAL const INT cos5[19] = { 1000, 996, 985, 966, 940, 906, 866, 819, 766, 707,
                  643, 574, 500, 423, 342, 259, 174, 87, 0 };

LOCAL INT table_pm(const INT *t, INT deg10)
{
    INT i;

    if (deg10 <= 0) {
        return t[0];
    }
    if (deg10 >= 900) {
        return t[18];
    }
    i = deg10 / 50;
    return t[i] + ((t[i + 1] - t[i]) * (deg10 % 50)) / 50;
}

EXPORT INT scan_tan_pm(INT deg10)
{
    return table_pm(tan5, deg10);
}

EXPORT INT scan_cos_pm(INT deg10)
{
    return table_pm(cos5, deg10);
}

EXPORT INT scan_sin_pm(INT deg10)
{
    return table_pm(cos5, 900 - deg10);
}

EXPORT INT scan_stan_pm(INT deg10)
{
    return (deg10 < 0) ? -scan_tan_pm(-deg10) : scan_tan_pm(deg10);
}

LOCAL INT ssin_pm(INT deg10)
{
    return (deg10 < 0) ? -scan_sin_pm(-deg10) : scan_sin_pm(deg10);
}

LOCAL INT clampi(INT v, INT lo, INT hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* A reading, with no echo counted as far away */
LOCAL INT reading(const INT *mm, INT i)
{
    return (mm[i] == SCAN_NO_ECHO) ? SCAN_MAX_RANGE_MM : mm[i];
}

EXPORT BOOL scan_profile(const INT mm[SCAN_STEPS], SCAN_RESULT *r)
{
    const INT lo = SCAN_STEP_CENTRE - (SCAN_SWEEP_DEG * 10) / SCAN_STEP_DEG10;
    const INT hi = SCAN_STEP_CENTRE + (SCAN_SWEEP_DEG * 10) / SCAN_STEP_DEG10;
    INT  i, nearest = SCAN_STEP_CENTRE, first, last, band, le10, re10, c10, face;
    INT  left_x, right_x, left_off, right_off, left_need, right_need;
    BOOL room_l, room_r;

    r->side = OBS_SIDE_RIGHT;
    r->width_mm = 0;
    r->half_left_mm = r->half_right_mm = 0;
    r->out_left = r->out_right = FALSE;
    r->clear_left_mm = r->clear_right_mm = 0;
    if (mm[SCAN_STEP_CENTRE] == SCAN_NO_ECHO) {
        return FALSE;                   /* nothing straight ahead: no profile */
    }
    /* the obstacle: the nearest reading and its neighbours within the band */
    for (i = lo; i <= hi; i++) {
        if (reading(mm, i) < reading(mm, nearest)) {
            nearest = i;
        }
    }
    band = reading(mm, nearest) + SCAN_BAND_MM;
    for (first = nearest; first > lo && reading(mm, first - 1) <= band; first--) {
        ;
    }
    for (last = nearest; last < hi && reading(mm, last + 1) <= band; last++) {
        ;
    }
    /* edges: half a step out, half a beam in (the beam sees an edge half a
       beam before pointing at it) */
    le10 = SCAN_STEP_DEG10_OF(first) - SCAN_STEP_DEG10 / 2 + SCAN_BEAM_DEG * 5;
    re10 = SCAN_STEP_DEG10_OF(last) + SCAN_STEP_DEG10 / 2 - SCAN_BEAM_DEG * 5;
    if (le10 > re10) {
        le10 = re10 = (le10 + re10) / 2;        /* narrower than the beam */
    }
    c10 = (le10 + re10) / 2;
    /* the face, square to its centre; the closest range corrected to it */
    i = SCAN_STEP_DEG10_OF(nearest) - c10;
    face = (reading(mm, nearest) * scan_cos_pm((i < 0) ? -i : i)) / 1000;
    r->width_mm = clampi((face * (scan_stan_pm(re10 - c10) - scan_stan_pm(le10 - c10))) / 1000,
                 0, SCAN_WIDTH_MAX_MM);
    r->out_left = (first == lo) ? TRUE : FALSE;
    r->out_right = (last == hi) ? TRUE : FALSE;
    /* clearance: the nearest reading beyond each edge (0 if unseen) */
    r->clear_left_mm = r->out_left ? 0 : SCAN_MAX_RANGE_MM;
    for (i = lo; i < first; i++) {
        r->clear_left_mm = (reading(mm, i) < r->clear_left_mm) ? reading(mm, i) : r->clear_left_mm;
    }
    r->clear_right_mm = r->out_right ? 0 : SCAN_MAX_RANGE_MM;
    for (i = last + 1; i <= hi; i++) {
        r->clear_right_mm = (reading(mm, i) < r->clear_right_mm) ? reading(mm, i) : r->clear_right_mm;
    }
    /* sideways position of each edge (- left), and the move to clear it */
    left_x = (reading(mm, first) * ssin_pm(le10)) / 1000;
    right_x = (reading(mm, last) * ssin_pm(re10)) / 1000;
    r->half_left_mm = (left_x < 0) ? -left_x : 0;
    r->half_right_mm = (right_x > 0) ? right_x : 0;
    left_off = SCAN_PATH_HALF_MM - left_x;
    right_off = SCAN_PATH_HALF_MM + right_x;
    left_need = clampi(left_off, SCAN_OFFSET_MIN_MM, SCAN_OFFSET_MAX_MM) + SCAN_CLEAR_MARGIN_MM;
    right_need = clampi(right_off, SCAN_OFFSET_MIN_MM, SCAN_OFFSET_MAX_MM) + SCAN_CLEAR_MARGIN_MM;
    /* room: clear beyond the edge, or the edge unseen (not blocked) */
    room_l = (r->clear_left_mm > left_need || r->out_left) ? TRUE : FALSE;
    room_r = (r->clear_right_mm > right_need || r->out_right) ? TRUE : FALSE;
    r->side = (room_l && (!room_r || left_off <= right_off)) ? OBS_SIDE_LEFT :
          (room_r ? OBS_SIDE_RIGHT : ((left_off <= right_off) ? OBS_SIDE_LEFT : OBS_SIDE_RIGHT));
    return (room_l || room_r) ? TRUE : FALSE;
}

EXPORT BOOL scan_width_calibrated(SCAN_RESULT *r, INT D_mm)
{
    INT raw = r->width_mm, w;

    if (raw <= 0 || D_mm < SCAN_CAL_D_MIN_MM || D_mm > SCAN_CAL_D_MAX_MM) {
        return FALSE;                   /* outside the calibration: as read */
    }
    w = (raw * SCAN_WIDTH_GAIN_PM) / 1000 - SCAN_WIDTH_OFFSET_MM;
    w = clampi(w, 0, SCAN_WIDTH_MAX_MM);
    r->half_left_mm = (r->half_left_mm * w) / raw;
    r->half_right_mm = (r->half_right_mm * w) / raw;
    r->width_mm = w;
    return TRUE;
}

/*** end of file ***/
