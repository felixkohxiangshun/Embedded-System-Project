/*
 *	imu_logic.c
 *	IMU/terrain -- pure hump-height / shock logic.  See imu_logic.h.
 *	All arithmetic fits in 32 bits (checked against the worst-case inputs
 *	in the comments), so it needs no 64-bit helpers on the Cortex-M0+.
 */

#include "imu_terrain.h"

#define Q15_SCALE	32768
#define DS_MAX_UM	50000		/* one sample never moves 5 cm; clamp keeps sin*ds in 32 bits */

LOCAL INT iabs(INT v)
{
	return (v < 0) ? -v : v;
}

LOCAL INT clamp_q15(INT v)
{
	if (v > Q15_ONE) {
		return Q15_ONE;
	}
	if (v < -Q15_ONE) {
		return -Q15_ONE;
	}
	return v;
}

/* Integer square root (floor), bit by bit: no FPU, no division */
EXPORT UW imu_isqrt(UW v)
{
	UW res = 0U;
	UW bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (v >= (res + bit)) {
			v -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

/* Encoder counts -> micrometres, carrying the remainder so that nothing is
   lost to rounding however many samples are summed.  Pass 2 * counts per
   rev when dcounts is the SUM of both wheels (= the car's mean distance).
   |dcounts| <= 9000 keeps dcounts * 216 * 1000 in 32 bits. */
EXPORT INT imu_counts_to_um(INT *rem, INT dcounts, INT mm_per_rev, INT counts_per_rev)
{
	W num, um;

	if (dcounts > 9000) {
		dcounts = 9000;
	} else if (dcounts < -9000) {
		dcounts = -9000;
	}
	num = ((W)dcounts * mm_per_rev * 1000) + *rem;
	um = num / counts_per_rev;
	*rem = (INT)(num - (um * counts_per_rev));
	return (INT)um;
}

LOCAL UW mag3(INT x, INT y, INT z)
{
	/* each |v| <= 16000 mg here: 3 * 16000^2 = 7.7e8 */
	return imu_isqrt((UW)(x * x) + (UW)(y * y) + (UW)(z * z));
}

/* ------------------------------------------------------------------------
 *	Calibration: 1 s still on flat ground.  The mean is the rest vector
 *	(baseline) and gives the mounting tilt, removed from every pitch.
 * ---------------------------------------------------------------------- */
LOCAL void cal_reset(IMU_LOGIC *l)
{
	INT k;

	l->cal_n = 0;
	for (k = 0; k < 3; k++) {
		l->cal_sum[k] = 0;
	}
}

LOCAL BOOL cal_step(IMU_LOGIC *l, const IMU_VEC *a)
{
	INT v[3];
	INT k;
	UW  m = mag3(a->fwd, a->side, a->up);

	v[0] = a->fwd;
	v[1] = a->side;
	v[2] = a->up;
	if ((m < (UW)CAL_G_MIN_MG) || (m > (UW)CAL_G_MAX_MG)) {
		cal_reset(l);			/* not 1 g: moving or upside down */
		return FALSE;
	}
	for (k = 0; k < 3; k++) {
		if (l->cal_n == 0) {
			l->cal_min[k] = v[k];
			l->cal_max[k] = v[k];
		} else {
			if (v[k] < l->cal_min[k]) {
				l->cal_min[k] = v[k];
			}
			if (v[k] > l->cal_max[k]) {
				l->cal_max[k] = v[k];
			}
		}
		if ((l->cal_max[k] - l->cal_min[k]) > CAL_SPREAD_MG) {
			cal_reset(l);		/* the car was moved: start again */
			return FALSE;
		}
	}
	for (k = 0; k < 3; k++) {
		l->cal_sum[k] += v[k];
	}
	l->cal_n++;
	return (l->cal_n >= IMU_CAL_SAMPLES) ? TRUE : FALSE;
}

LOCAL void box_reset(IMU_LOGIC *l)
{
	INT k;

	for (k = 0; k < 3; k++) {
		l->box_sum[k] = 0;
	}
	l->box_i = 0;
	l->box_n = 0;
}

/* tan(roll) in Q15 = side / sqrt(fwd^2 + up^2), clamped to +/-ROLL_TAN_MAX.
   Inputs in any common unit up to 32752 (the filter's mg * 4). */
LOCAL INT roll_tan(INT side, INT fwd, INT up)
{
	UW den = imu_isqrt((UW)(fwd * fwd) + (UW)(up * up));
	INT t;

	if (den == 0U) {
		return 0;
	}
	if ((UW)iabs(side) >= den) {
		return (side < 0) ? -ROLL_TAN_MAX : ROLL_TAN_MAX;
	}
	t = (side * Q15_SCALE) / (INT)den;	/* |side| < den <= 46000: fits */
	if (t > ROLL_TAN_MAX) {
		return ROLL_TAN_MAX;
	}
	return (t < -ROLL_TAN_MAX) ? -ROLL_TAN_MAX : t;
}

LOCAL void cal_finish(IMU_LOGIC *l)
{
	INT fwd = (INT)(l->cal_sum[0] / l->cal_n);
	INT side = (INT)(l->cal_sum[1] / l->cal_n);
	INT up = (INT)(l->cal_sum[2] / l->cal_n);
	UW  n = mag3(fwd, side, up);
	UW  nc = imu_isqrt((UW)(side * side) + (UW)(up * up));

	l->rest.fwd = fwd;
	l->rest.side = side;
	l->rest.up = up;
	l->s0 = clamp_q15((fwd * Q15_SCALE) / (INT)n);	/* n is 850..1150 here */
	l->c0 = clamp_q15((INT)((nc * Q15_SCALE) / n));
	l->t0 = roll_tan(side, fwd, up);
	l->f[0] = fwd * (1 << IMU_LPF_FRAC);		/* start the filter at rest */
	l->f[1] = side * (1 << IMU_LPF_FRAC);
	l->f[2] = up * (1 << IMU_LPF_FRAC);
	l->primed = TRUE;
	l->spike_n = 0;
	box_reset(l);
	l->calibrated = TRUE;
	cal_reset(l);
}

/* ------------------------------------------------------------------------
 *	Pitch.  fwd axis = g sin(pitch) + a_fwd: a_fwd (encoders) is removed,
 *	the axes are low-pass filtered (a single knock is skipped), then the
 *	mounting tilt is rotated out:
 *	    sin(pitch - tilt) = sin(pitch) cos(tilt) - cos(pitch) sin(tilt)
 *	Sizes: |a| <= 8188 mg (+/-8 g), so f <= 32752 and f^2 sums < 2^32.
 * ---------------------------------------------------------------------- */
LOCAL INT pitch_step(IMU_LOGIC *l, const IMU_VEC *a, INT acc_mg)
{
	INT  x[3];
	INT  k, s, c;
	UW   n, nc;
	BOOL far = FALSE;

	x[0] = a->fwd - acc_mg;
	x[1] = a->side;
	x[2] = a->up;

	/* knock check on the raw sample, against the filter */
	if (l->primed) {
		for (k = 0; k < 3; k++) {
			if (iabs((x[k] * (1 << IMU_LPF_FRAC)) - l->f[k]) > (IMU_SPIKE_MG * (1 << IMU_LPF_FRAC))) {
				far = TRUE;
			}
		}
		if (far && (l->spike_n < IMU_SPIKE_MAX)) {
			l->spike_n++;		/* a knock is not a tilt: skip it */
		} else {
			far = FALSE;
			l->spike_n = 0;
		}
	}

	if (!far) {
		/* 8-sample average: the car's 20-30 Hz shaking cancels out */
		for (k = 0; k < 3; k++) {
			if (l->box_n < IMU_BOX) {
				l->box_sum[k] += x[k];
			} else {
				l->box_sum[k] += x[k] - l->box[k][l->box_i];
			}
			l->box[k][l->box_i] = x[k];
			x[k] = (INT)((l->box_sum[k] * (1 << IMU_LPF_FRAC)) /
				     ((l->box_n < IMU_BOX) ? (l->box_n + 1) : IMU_BOX));
		}
		l->box_i = (l->box_i + 1) % IMU_BOX;
		if (l->box_n < IMU_BOX) {
			l->box_n++;
		}

		/* then the low-pass filter */
		if (!l->primed) {
			for (k = 0; k < 3; k++) {
				l->f[k] = x[k];
			}
			l->primed = TRUE;
			l->spike_n = 0;
		} else {
			for (k = 0; k < 3; k++) {
				l->f[k] += (x[k] - l->f[k]) / (1 << IMU_LPF_SHIFT);
			}
		}
	}

	/* roll, mounting roll removed: tan(r - r0) = (t - t0) / (1 + t t0) */
	{
		INT t = roll_tan(l->f[1], l->f[0], l->f[2]);

		l->tan_roll_q15 = ((t - l->t0) * Q15_SCALE) /
				  (Q15_SCALE + ((t * l->t0) / Q15_SCALE));
	}

	nc = imu_isqrt((UW)(l->f[1] * l->f[1]) + (UW)(l->f[2] * l->f[2]));
	n = imu_isqrt((UW)(l->f[0] * l->f[0]) + (nc * nc));
	if (n == 0U) {
		return 0;
	}
	s = (l->f[0] * Q15_SCALE) / (INT)n;
	c = (INT)((nc * Q15_SCALE) / n);
	return clamp_q15(((s * l->c0) / Q15_SCALE) - ((c * l->s0) / Q15_SCALE));
}

/* ------------------------------------------------------------------------
 *	Forward acceleration from the encoders.
 *	a = (distance in the last W samples - the W before) / (W T)^2
 * ---------------------------------------------------------------------- */
LOCAL void acc_reset(IMU_LOGIC *l)
{
	INT k;

	for (k = 0; k < (2 * ACC_WIN); k++) {
		l->acc_ds[k] = 0;
	}
	l->acc_i = 0;
	l->acc_n = 0;
}

/* Returns the forward acceleration ACC_WIN samples ago, and in *a the
   accelerometer sample from that same moment (so the two line up: the
   delay is the same for both, and a pure delay does not change the
   rebuilt height). */
LOCAL INT acc_step(IMU_LOGIC *l, INT ds_um, IMU_VEC *a)
{
	INT k, idx;
	W   s_new = 0;
	W   s_old = 0;
	INT di = l->acc_i % ACC_WIN;
	IMU_VEC now = *a;

	if (l->acc_n >= ACC_WIN) {
		*a = l->dly[di];		/* the sample from ACC_WIN ago */
	}
	l->dly[di] = now;
	l->acc_ds[l->acc_i] = ds_um;
	l->acc_i = (l->acc_i + 1) % (2 * ACC_WIN);
	if (l->acc_n < (2 * ACC_WIN)) {
		l->acc_n++;
		return 0;
	}
	for (k = 1; k <= (2 * ACC_WIN); k++) {
		idx = (l->acc_i - k + (2 * ACC_WIN)) % (2 * ACC_WIN);
		if (k <= ACC_WIN) {
			s_new += l->acc_ds[idx];
		} else {
			s_old += l->acc_ds[idx];
		}
	}
	return (INT)(((s_new - s_old) * ACC_UM_S2_PER_DIFF) / UM_S2_PER_MG);
}

/* ------------------------------------------------------------------------
 *	Surface profile:  y_lead(X) = y_lead(X - b) + b sin(pitch)
 *	History is stored once per mm of forward travel.
 *	b sin in um = sin_q15 * b_mm * 1000 / 32768 = sin_q15 * b_mm * 125 / 4096
 *	(32767 * 159 * 125 = 6.5e8, fits).
 * ---------------------------------------------------------------------- */
LOCAL void prof_zero(IMU_LOGIC *l)
{
	INT k;

	for (k = 0; k < PROF_RING_MM; k++) {
		l->ring[k] = 0;
	}
	for (k = 0; k < HUMP_SMOOTH; k++) {
		l->sm[k] = 0;
	}
	l->sm_i = 0;
	l->prof_um = 0;
	l->plain_rem = 0;
}

LOCAL INT prof_step(IMU_LOGIC *l, INT sin_q15, INT ds_um)
{
	INT m, k, back, num, dh;

	if (l->b_mm == 0) {			/* plain SUM(sin x ds) */
		num = (sin_q15 * ds_um) + l->plain_rem;	/* <= 32767 * 50000 */
		dh = num / Q15_SCALE;
		l->plain_rem = num - (dh * Q15_SCALE);
		l->prof_um += dh;
		return l->prof_um;
	}
	if (ds_um <= 0) {
		return l->prof_um;		/* standing or reversing: no new point */
	}
	l->pos_um += ds_um;
	m = (INT)(l->pos_um / 1000);
	back = l->ring[(((m - l->b_mm) % PROF_RING_MM) + PROF_RING_MM) % PROF_RING_MM];
	l->prof_um = back + ((sin_q15 * l->b_mm * 125) / 4096);
	for (k = 0; (l->last_mm < m) && (k < PROF_RING_MM); k++) {
		l->last_mm++;
		l->ring[l->last_mm % PROF_RING_MM] = l->prof_um;
	}
	l->last_mm = m;
	return l->prof_um;
}

LOCAL INT smooth_step(IMU_LOGIC *l, INT v)
{
	INT k;
	W   sum = 0;

	l->sm[l->sm_i] = v;
	l->sm_i = (l->sm_i + 1) % HUMP_SMOOTH;
	for (k = 0; k < HUMP_SMOOTH; k++) {
		sum += l->sm[k];
	}
	return (INT)(sum / HUMP_SMOOTH);
}

/* ------------------------------------------------------------------------
 *	Hump detector, on the smoothed profile height.
 *	FLAT : remember the last HUMP_HIST heights.  Pitch above 2 deg starts
 *	       tracking; the level is taken from before the start, so the gentle
 *	       start of the ramp counts.  A long level run re-zeroes the profile
 *	       so small pitch errors cannot pile up.
 *	TRACK: rel = height above that level, peak = max(rel), confirmed at
 *	       3 mm.  Over when level for HUMP_QUIET_END moving samples AND the
 *	       height is back under peak/4.  (The car is also level while it
 *	       straddles a crest or sits on a flat top, but the height is still
 *	       high there, so the hump does not end early.)
 * ---------------------------------------------------------------------- */
LOCAL void hump_to_flat(IMU_LOGIC *l, INT h_now)
{
	INT k;

	l->tracking = FALSE;
	l->confirmed = FALSE;
	l->quiet_n = 0;
	l->still_n = 0;
	l->flat_um = 0;
	l->rel_um = 0;
	l->peak_um = 0;
	l->len_um = 0;
	for (k = 0; k < HUMP_HIST; k++) {
		l->hist[k] = h_now;
	}
	l->hist_i = 0;
}

LOCAL void hump_give_up(IMU_LOGIC *l)
{
	if (l->confirmed) {
		l->aborts++;
	}
	prof_zero(l);
	hump_to_flat(l, 0);
}

/* A confirmed hump is over: keep its peak as the result */
LOCAL void hump_record(IMU_LOGIC *l)
{
	l->height_mm = (l->peak_um + 500) / 1000;
	l->residual_um = l->rel_um;
	l->len_mm = (INT)(l->len_um / 1000);
	l->humps++;
	if ((l->height_mm <= HUMP_MAX_MM) && (l->height_mm > l->max_height_mm)) {
		l->max_height_mm = l->height_mm;
	}
}

/* Roll mode: height of the hump-side wheel, um.  Signed: > 0 = that wheel
   up; < 0 = the car leans the other way (sway), never a hump.
   (32767-scaled tan) * T mm * 1000 / 32768 = tan * T * 125 / 4096 */
LOCAL INT roll_height_um(const IMU_LOGIC *l)
{
	return (l->tan_roll_q15 * CAR_TRACK_MM * 125) / 4096;
}

LOCAL INT hump_step(IMU_LOGIC *l, INT sin_q15, INT ds_um, BOOL allowed)
{
	INT  h, raw;
	INT  ev = IMU_EV_NONE;
	BOOL level, start;

	if (!allowed) {
		hump_give_up(l);	/* turning / bypass: the readings do not mean height */
		return IMU_EV_NONE;
	}
	if (l->mode == HUMP_MODE_ROLL) {
		raw = roll_height_um(l);
		level = (raw < ROLL_LEVEL_UM) ? TRUE : FALSE;	/* leaning the other way is level too */
		start = ((raw > ROLL_START_UM) && (ds_um > 0)) ? TRUE : FALSE;	/* a hump needs movement */
	} else {
		raw = prof_step(l, sin_q15, ds_um);
		level = (iabs(sin_q15) < HUMP_START_SIN) ? TRUE : FALSE;
		start = (sin_q15 > HUMP_START_SIN) ? TRUE : FALSE;
	}
	h = smooth_step(l, raw);

	if (!l->tracking) {
		if (start) {
			l->tracking = TRUE;
			/* roll: the calibration is the level; pitch: the profile before the start */
			l->h_ref = (l->mode == HUMP_MODE_ROLL) ? 0 : l->hist[l->hist_i];
			l->rel_um = h - l->h_ref;
			l->peak_um = (l->rel_um > 0) ? l->rel_um : 0;
			l->len_um = iabs(ds_um);
			l->quiet_n = 0;
			return IMU_EV_NONE;
		}
		l->hist[l->hist_i] = h;
		l->hist_i = (l->hist_i + 1) % HUMP_HIST;
		if (level && (ds_um > 0)) {
			l->flat_um += ds_um;
			if (l->flat_um > (((W)l->b_mm + HUMP_ZERO_EXTRA_MM) * 1000)) {
				prof_zero(l);	/* long level run: back to 0 */
				hump_to_flat(l, 0);
			}
		} else if (!level) {
			l->flat_um = 0;
		}
		return IMU_EV_NONE;
	}

	l->rel_um = h - l->h_ref;
	l->len_um += iabs(ds_um);
	if (l->rel_um > l->peak_um) {
		l->peak_um = l->rel_um;
	}
	if (!l->confirmed &&
	    (l->rel_um >= ((l->mode == HUMP_MODE_ROLL) ? ROLL_MIN_UM : HUMP_MIN_UM))) {
		l->confirmed = TRUE;
		ev = IMU_EV_HUMP_START;
	}
	if (ds_um != 0) {		/* standing still on a slope is not "level" */
		l->quiet_n = level ? (l->quiet_n + 1) : 0;
		l->still_n = 0;
	} else if (++l->still_n >= HUMP_STILL_DROP) {
		if (l->confirmed) {
			l->aborts++;
		}
		hump_to_flat(l, h);	/* standing still that long: handled, not driven */
		return IMU_EV_NONE;
	}

	if ((l->quiet_n >= HUMP_QUIET_END) &&
	    (!l->confirmed || ((l->rel_um * HUMP_END_DIV) < l->peak_um))) {
		if (l->confirmed) {
			hump_record(l);
			ev = IMU_EV_HUMP_DONE;
		}
		hump_to_flat(l, h);
		return ev;
	}
	if (l->len_um > ((W)HUMP_MAX_LEN_MM * 1000)) {
		if (l->confirmed) {
			l->aborts++;	/* a long slope or a ramp onto a platform */
		}
		hump_to_flat(l, h);
		return IMU_EV_NONE;
	}
	return ev;
}

/* ------------------------------------------------------------------------
 *	Public API
 * ---------------------------------------------------------------------- */
EXPORT void imu_set_wheelbase(IMU_LOGIC *l, INT b_mm)
{
	if (b_mm >= PROF_RING_MM) {
		b_mm = PROF_RING_MM - 1;
	}
	l->b_mm = (b_mm < 0) ? 0 : b_mm;
	prof_zero(l);
	hump_to_flat(l, 0);
}

EXPORT void imu_run_reset(IMU_LOGIC *l)
{
	l->run_peak_um = 0;
	l->run_dev[0] = 0;
	l->run_dev[1] = 0;
	l->run_dev[2] = 0;
}

EXPORT INT imu_finish(IMU_LOGIC *l)
{
	INT h = l->h_ref + l->rel_um;

	if (l->tracking && l->confirmed) {
		hump_record(l);
		hump_to_flat(l, h);
		l->state = IMU_HUMP_COMPLETE;
		return IMU_EV_HUMP_DONE;
	}
	return IMU_EV_NONE;
}

EXPORT void imu_set_mode(IMU_LOGIC *l, UB mode)
{
	l->mode = mode;
	prof_zero(l);
	hump_to_flat(l, 0);
}

EXPORT void imu_logic_init(IMU_LOGIC *l)
{
	l->state = IMU_CALIBRATING;
	l->calibrated = FALSE;
	cal_reset(l);
	l->rest.fwd = 0;		/* until calibrated: level mount, 1 g up */
	l->rest.side = 0;
	l->rest.up = 1000;
	l->s0 = 0;
	l->c0 = Q15_ONE;
	l->t0 = 0;
	l->tan_roll_q15 = 0;
	l->mode = IMU_HUMP_MODE;
	l->primed = FALSE;
	l->spike_n = 0;
	box_reset(l);
	l->sin_q15 = 0;
	acc_reset(l);
	l->pos_um = 0;
	l->last_mm = 0;
	l->over_cnt = 0;
	l->shock_pending = FALSE;
	l->height_mm = 0;
	l->max_height_mm = 0;
	l->residual_um = 0;
	l->len_mm = 0;
	l->humps = 0U;
	l->aborts = 0U;
	imu_run_reset(l);
	imu_set_wheelbase(l, CAR_WHEELBASE_MM);
}

EXPORT void imu_fault(IMU_LOGIC *l)
{
	hump_give_up(l);
	l->state = IMU_SENSOR_FAULT;
	l->over_cnt = 0;
	l->shock_pending = FALSE;
}

/* The mounting tilt does not change with an I2C fault, and the car may be
   driving (or on a hump) when the sensor comes back, so the baseline is
   kept; only the filters start again.  Not calibrated yet -> CALIBRATING. */
EXPORT void imu_recovered(IMU_LOGIC *l)
{
	l->primed = FALSE;
	box_reset(l);
	acc_reset(l);
	cal_reset(l);
	prof_zero(l);
	hump_to_flat(l, 0);
	l->over_cnt = 0;
	l->state = l->calibrated ? IMU_MONITORING : IMU_CALIBRATING;
}

EXPORT INT imu_feed(IMU_LOGIC *l, const IMU_VEC *a, INT ds_um, BOOL still, BOOL allowed)
{
	INT  acc, ev, was_over;
	UW   dev;
	IMU_VEC ad;

	if (l->state == IMU_SENSOR_FAULT) {
		return IMU_EV_NONE;
	}
	if ((l->state == IMU_HUMP_COMPLETE) || (l->state == IMU_SHOCK_EVENT)) {
		l->state = l->confirmed ? IMU_HUMP_TRACKING : IMU_MONITORING;
	}
	if (ds_um > DS_MAX_UM) {
		ds_um = DS_MAX_UM;
	} else if (ds_um < -DS_MAX_UM) {
		ds_um = -DS_MAX_UM;
	}
	/* shock: far from the rest vector, judged when the spike ends */
	dev = mag3(a->fwd - l->rest.fwd, a->side - l->rest.side, a->up - l->rest.up);
	was_over = l->over_cnt;
	if (dev > (UW)SHOCK_MG) {
		l->over_cnt++;
	} else {
		l->over_cnt = 0;
		if ((was_over > 0) && (was_over < SHOCK_MAX_SAMPLES)) {
			l->shock_pending = TRUE;
		}
	}

	/* calibration: the car must stand still for IMU_CAL_SAMPLES.  Until it
	   has been measured nothing is measured (an unknown mounting tilt looks
	   exactly like a lifted wheel), so the state stays CALIBRATING; if the
	   car drives off first, it finishes at the next stop. */
	if (!l->calibrated) {
		if (still) {
			if (cal_step(l, a)) {
				cal_finish(l);
				l->state = IMU_MONITORING;
				prof_zero(l);
				hump_to_flat(l, 0);
			}
		} else {
			cal_reset(l);
		}
		if (!l->calibrated) {
			l->state = IMU_CALIBRATING;
			if (l->shock_pending) {
				l->shock_pending = FALSE;
				return IMU_EV_SHOCK;
			}
			return IMU_EV_NONE;
		}
	}

	ad = *a;
	acc = acc_step(l, ds_um, &ad);		/* ad = the sample lined up with acc */
	l->sin_q15 = pitch_step(l, &ad, acc);

	/* per-run statistics, whatever the mode (tests, axis check) */
	{
		INT d0 = iabs(a->fwd - l->rest.fwd);
		INT d1 = iabs(a->side - l->rest.side);
		INT d2 = iabs(a->up - l->rest.up);
		INT rh = roll_height_um(l);

		if (d0 > l->run_dev[0]) {
			l->run_dev[0] = d0;
		}
		if (d1 > l->run_dev[1]) {
			l->run_dev[1] = d1;
		}
		if (d2 > l->run_dev[2]) {
			l->run_dev[2] = d2;
		}
		if (rh > l->run_peak_um) {
			l->run_peak_um = rh;
		}
	}
	ev = hump_step(l, l->sin_q15, ds_um, allowed);

	if (ev == IMU_EV_HUMP_DONE) {
		l->state = IMU_HUMP_COMPLETE;
	} else if (ev == IMU_EV_HUMP_START) {
		l->state = IMU_HUMP_TRACKING;
	} else {
		l->state = l->confirmed ? IMU_HUMP_TRACKING : IMU_MONITORING;
		if (l->shock_pending) {
			l->shock_pending = FALSE;
			l->state = IMU_SHOCK_EVENT;
			ev = IMU_EV_SHOCK;
		}
	}
	return ev;
}

EXPORT INT imu_pitch_deg(const IMU_LOGIC *l)
{
	return (l->sin_q15 * 5730) / (100 * Q15_SCALE);	/* 180/pi = 57.30 */
}

EXPORT INT imu_height_now_um(const IMU_LOGIC *l)
{
	return l->tracking ? l->rel_um : 0;
}
