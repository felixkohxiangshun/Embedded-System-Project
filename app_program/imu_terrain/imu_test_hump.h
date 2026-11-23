/*
 *	imu_test_hump.h
 *	IMU/terrain -- test helper: drives a simulated car over a trapezoid hump
 *	and feeds imu_feed() what the accelerometer and encoders would report.
 *	Integer only (no libm), so the SAME checks run on the PC (testing/host)
 *	and on the board (make TEST=1).  Included only by the two test files.
 *
 *	Car: drive wheels behind, caster TH_WHEELBASE_MM ahead, wheels
 *	CAR_TRACK_MM apart, 120 mm/s (1.2 mm per 10 ms), board tilted 2 deg
 *	nose-up and 1 deg sideways on its mount, +/-20 mg noise.
 *	A hump is either under the right wheel only (one_wheel = 1, our course:
 *	the car leans) or across the whole track (one_wheel = 0: it pitches).
 *	The physics test with speed changes and vibration is
 *	testing/imu_terrain/sim_hump.c.
 */

#ifndef IMU_TEST_HUMP_H
#define IMU_TEST_HUMP_H

#define TH_WHEELBASE_MM		CAR_WHEELBASE_MM
#define TH_STEP_UM		1200		/* 120 mm/s */
#define TH_TILT_SIN		1144		/* sin(2 deg), Q15 */
#define TH_TILT_COS		32748		/* cos(2 deg), Q15 */
#define TH_ROLL_SIN		572		/* sin(1 deg), Q15: mounting roll */
#define TH_ROLL_COS		32763
#define TH_Q			32768

typedef struct {
	INT	x0_mm;		/* hump starts here (drive-wheel position) */
	INT	h_mm;		/* 0 = flat road */
	INT	ramp_mm;
	INT	top_mm;
	INT	one_wheel;	/* 1 = under the right wheel only */
} TH_HUMP;

typedef struct {
	INT	starts, dones, lifts;
	INT	height_mm;	/* of the last HUMP_DONE */
} TH_OUT;

static UW th_seed = 12345U;

static INT th_noise(INT amp)
{
	th_seed = (th_seed * 1103515245U) + 12345U;
	return (INT)((th_seed >> 16) % (UW)((2 * amp) + 1)) - amp;
}

/* surface height in um at x um */
static INT th_y_um(const TH_HUMP *h, INT x_um)
{
	INT u = x_um - (h->x0_mm * 1000);
	INT r = h->ramp_mm * 1000;
	INT t = h->top_mm * 1000;

	if ((h->h_mm == 0) || (u <= 0) || (u >= ((2 * r) + t))) {
		return 0;
	}
	if (u < r) {
		return (INT)(((W)h->h_mm * u) / h->ramp_mm);		/* h_mm*1000*u/r */
	}
	if (u < (r + t)) {
		return h->h_mm * 1000;
	}
	return (INT)(((W)h->h_mm * ((2 * r) + t - u)) / h->ramp_mm);
}

/* Readings (mg, before the mounting tilt) of a car whose right wheel is
   lifted by y_um, left wheel and caster on the floor.  The body plane has
   tan(roll) a = y / T and nose-down slope c = y / (2 b); then
   side = a / sqrt(1+a^2), up = 1 / N, fwd = -c / (N sqrt(1+a^2)),
   N = sqrt(1 + a^2 + c^2). */
static void th_one_wheel(INT y_um, INT *fwd, INT *side, INT *up)
{
	INT a = (y_um * 16384) / (CAR_TRACK_MM * 500);		/* Q15 */
	INT c = (y_um * 16384) / (TH_WHEELBASE_MM * 1000);	/* Q15 */
	INT sa = (INT)imu_isqrt((UW)(TH_Q * TH_Q) + (UW)(a * a));
	INT n = (INT)imu_isqrt((UW)(TH_Q * TH_Q) + (UW)(a * a) + (UW)(c * c));

	*side = (1000 * a) / sa;
	*up = (1000 * TH_Q) / n;
	*fwd = -((((c * TH_Q) / n) * 1000) / sa);
}

/* What the sensors see with the drive wheels at x_um (previous at x_prev) */
static void th_sense(const TH_HUMP *h, INT x_um, INT x_prev, IMU_VEC *a, INT *ds_um, INT noise)
{
	INT fwd, side, up, f2, u2, ddx, ddy;

	if (h->one_wheel) {
		th_one_wheel(th_y_um(h, x_um), &fwd, &side, &up);
	} else {
		INT b_um = TH_WHEELBASE_MM * 1000;
		INT dy = th_y_um(h, x_um + b_um) - th_y_um(h, x_um);	/* front - rear */
		INT chord = (INT)imu_isqrt((UW)((b_um / 10) * (b_um / 10)) + (UW)((dy / 10) * (dy / 10))) * 10;
		INT s = (dy * 16384) / (chord / 2);			/* sin(pitch), Q15 */
		INT c = (INT)imu_isqrt((UW)(Q15_ONE * Q15_ONE) - (UW)(s * s));

		fwd = (s * 1000) / TH_Q;
		side = 0;
		up = (c * 1000) / TH_Q;
	}
	/* mounting: 2 deg nose-up, then 1 deg roll */
	f2 = ((fwd * TH_TILT_COS) + (up * TH_TILT_SIN)) / TH_Q;
	u2 = ((up * TH_TILT_COS) - (fwd * TH_TILT_SIN)) / TH_Q;
	a->fwd = f2 + th_noise(noise);
	a->side = (((side * TH_ROLL_COS) + (u2 * TH_ROLL_SIN)) / TH_Q) + th_noise(noise);
	a->up = (((u2 * TH_ROLL_COS) - (side * TH_ROLL_SIN)) / TH_Q) + th_noise(noise);

	/* distance: mean of the left wheel (floor) and the right wheel */
	ddx = x_um - x_prev;
	ddy = th_y_um(h, x_um) - th_y_um(h, x_prev);
	if (h->one_wheel) {
		*ds_um = (ddx + (INT)imu_isqrt((UW)(ddx * ddx) + (UW)(ddy * ddy))) / 2;
	} else {
		*ds_um = (INT)imu_isqrt((UW)(ddx * ddx) + (UW)(ddy * ddy));
	}
}

/* Feed n samples of one acceleration, car standing still; returns how many
   of them reported IMU_EV_LIFTED */
static INT th_hold(IMU_LOGIC *l, INT fwd, INT side, INT up, INT n)
{
	IMU_VEC a;
	INT k, lifts = 0;

	for (k = 0; k < n; k++) {
		a.fwd = fwd;
		a.side = side;
		a.up = up;
		if (imu_feed(l, &a, 0, FALSE, TRUE) == IMU_EV_LIFTED) {
			lifts++;
		}
	}
	return lifts;
}

/* Calibrate: 1.2 s standing still */
static void th_calibrate(IMU_LOGIC *l)
{
	TH_HUMP flat = { 0, 0, 1, 0, 0 };
	IMU_VEC a;
	INT ds, k;

	for (k = 0; k < (IMU_CAL_SAMPLES + 20); k++) {
		th_sense(&flat, 0, 0, &a, &ds, 20);
		(void)imu_feed(l, &a, 0, TRUE, TRUE);
	}
}

/* Drive from x_from to x_to mm.  spike_at_mm >= 0 adds a spike of spike_mg
   on the forward axis for spike_n samples there.  turn_at_mm >= 0: the car
   is turning (allowed = FALSE) from there on. */
static TH_OUT th_drive(IMU_LOGIC *l, const TH_HUMP *h, INT x_from, INT x_to,
		       INT spike_at_mm, INT spike_mg, INT spike_n, INT turn_at_mm)
{
	TH_OUT o = { 0, 0, 0, 0 };
	IMU_VEC a;
	INT x = x_from * 1000;
	INT x_prev = x;
	INT ds, ev;
	INT left = 0;

	while (x < (x_to * 1000)) {
		x += TH_STEP_UM;
		th_sense(h, x, x_prev, &a, &ds, 20);
		x_prev = x;
		if ((spike_at_mm >= 0) && (left == 0) && (x >= (spike_at_mm * 1000)) &&
		    (x < ((spike_at_mm * 1000) + TH_STEP_UM))) {
			left = spike_n;
		}
		if (left > 0) {
			a.fwd += spike_mg;
			left--;
		}
		ev = imu_feed(l, &a, ds, FALSE,
			      ((turn_at_mm >= 0) && (x >= (turn_at_mm * 1000))) ? FALSE : TRUE);
		if (ev == IMU_EV_HUMP_START) {
			o.starts++;
		} else if (ev == IMU_EV_HUMP_DONE) {
			o.dones++;
			o.height_mm = l->height_mm;
		} else if (ev == IMU_EV_LIFTED) {
			o.lifts++;
		}
	}
	return o;
}

static INT th_abs(INT v)
{
	return (v < 0) ? -v : v;
}

#endif /* IMU_TEST_HUMP_H */
