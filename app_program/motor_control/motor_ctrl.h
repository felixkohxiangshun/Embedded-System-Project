/*
 *	motor_ctrl.h
 *	Motor control -- pure motor-control maths (no hardware, no kernel calls).
 *	Integer only: the RP2040's Cortex-M0+ has no FPU, so float is slow.
 *	Because it touches no hardware, this file is unit-tested on a PC too
 *	(tools are described in testing/README.md).
 *
 *	*** CALIBRATE ON THE REAL CAR: every constant marked [CAL] is a
 *	*** placeholder until it has been measured.
 */

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

/* ---- Mechanics [CAL] ---- */
#define ENC_COUNTS_PER_REV	960	/* quadrature counts per wheel turn (x4 decoding) */
#define WHEEL_MM_PER_REV	204	/* wheel circumference, mm (65 mm wheel) */
#define TRACK_MM		130	/* distance between the two wheels, mm */
#define ENC_SIGN_LEFT		(+1)	/* flip if the count runs backwards */
#define ENC_SIGN_RIGHT		(+1)

/* ---- Speeds, mm/s [CAL].  FR-11 says "1-2 cm/s", which reads as a typo
 *      for 10-20 cm/s; 12 cm/s is used until the team confirms. ---- */
#define SPEED_NORMAL_MMPS	120
#define SPEED_SLOW_MMPS		60
#define SPEED_TURN_MMPS		60
#define STEER_GAIN_MMPS		60	/* wheel speed difference at full steer */

/* ---- Control loop ---- */
#define MOTOR_TICK_MS		10	/* NFR-14: PID loop every 10 ms */

/* PID gains in 1/256 units, output in duty permille (0..1000) */
#define PID_KP			512	/* 2.0 */
#define PID_KI			40	/* 0.16 */
#define PID_KD			128	/* 0.5 */
#define PID_SCALE		256
#define PID_INTEG_MAX		2000
#define DUTY_FF_PER_MMPS	3	/* feed-forward: permille per mm/s [CAL] */
#define DUTY_MAX		1000

/* Encoder fault: commanded above this duty but not one count for this long */
#define ENC_FAULT_DUTY		300
#define ENC_FAULT_TICKS		30	/* 300 ms */

/* Straight-line drift trim (FR-13) */
#define TRIM_DEADBAND		30	/* |steer| below this counts as straight */
#define TRIM_MAX_MMPS		20

typedef struct {
	INT	kp, ki, kd;
	INT	integ;
	INT	prev_err;
	INT	out_min, out_max;
} PID;

IMPORT void pid_init(PID *p, INT kp, INT ki, INT kd, INT out_min, INT out_max);
IMPORT void pid_reset(PID *p);
IMPORT INT  pid_step(PID *p, INT setpoint, INT measured);

/* Quadrature decoder.  state = (A << 1) | B.  Returns +1, -1 or 0
   (0 = no move or an impossible jump, i.e. a missed edge). */
IMPORT INT  qdec_step(UB prev_state, UB cur_state);

IMPORT INT  counts_to_mmps(INT delta_counts);	/* over one MOTOR_TICK_MS */
IMPORT INT  mm_to_counts(INT mm);
IMPORT INT  counts_to_deg(INT counts);		/* inverse of deg_to_counts */
IMPORT INT  deg_to_counts(INT deg);		/* per wheel, for a pivot turn */
IMPORT INT  clamp_i(INT v, INT lo, INT hi);

#endif /* MOTOR_CTRL_H */
