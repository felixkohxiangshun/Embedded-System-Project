/*
 *	motor_ctrl.c
 *	Motor control -- pure motor-control maths.  See motor_ctrl.h.
 */

#include "motor_control.h"

EXPORT INT clamp_i(INT v, INT lo, INT hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

EXPORT void pid_init(PID *p, INT kp, INT ki, INT kd, INT out_min, INT out_max)
{
	p->kp = kp;
	p->ki = ki;
	p->kd = kd;
	p->out_min = out_min;
	p->out_max = out_max;
	pid_reset(p);
}

EXPORT void pid_reset(PID *p)
{
	p->integ = 0;
	p->prev_err = 0;
}

/* Positional PID with a clamped integral (anti-windup).  The setpoint and
   measurement are wheel speeds in mm/s; the result is a duty correction in
   permille, added to the feed-forward term by the caller. */
EXPORT INT pid_step(PID *p, INT setpoint, INT measured)
{
	INT err = setpoint - measured;
	INT deriv = err - p->prev_err;
	INT out;

	p->integ = clamp_i(p->integ + err, -PID_INTEG_MAX, PID_INTEG_MAX);
	p->prev_err = err;
	out = (p->kp * err + p->ki * p->integ + p->kd * deriv) / PID_SCALE;
	return clamp_i(out, p->out_min, p->out_max);
}

/* prev*4 + cur -> step.  Forward sequence is 00 -> 01 -> 11 -> 10 -> 00. */
EXPORT INT qdec_step(UB prev_state, UB cur_state)
{
	static const signed char tbl[16] = {
	/* prev 00 */  0, +1, -1,  0,
	/* prev 01 */ -1,  0,  0, +1,
	/* prev 10 */ +1,  0,  0, -1,
	/* prev 11 */  0, -1, +1,  0,
	};

	return tbl[((prev_state & 3) << 2) | (cur_state & 3)];
}

EXPORT INT counts_to_mmps(INT delta_counts)
{
	/* mm = counts * mm_per_rev / counts_per_rev; per second = * 1000/tick */
	return (delta_counts * WHEEL_MM_PER_REV * (1000 / MOTOR_TICK_MS))
		/ ENC_COUNTS_PER_REV;
}

EXPORT INT mm_to_counts(INT mm)
{
	return (mm * ENC_COUNTS_PER_REV) / WHEEL_MM_PER_REV;
}

/* In a pivot each wheel travels an arc of pi * TRACK * deg / 360 mm.
   (pi is taken as 314/100.)  NFR-15 asks for +/-5 degrees. */
EXPORT INT deg_to_counts(INT deg)
{
	INT mm = (deg * TRACK_MM * 314) / 36000;

	return mm_to_counts(mm);
}

/* Inverse of deg_to_counts(): the angle of a pivot in which each wheel
   moved 'counts'.  Used for the FR-16 heading (dead reckoning). */
EXPORT INT counts_to_deg(INT counts)
{
	INT mm = (counts * WHEEL_MM_PER_REV) / ENC_COUNTS_PER_REV;

	return (mm * 36000) / (TRACK_MM * 314);
}
