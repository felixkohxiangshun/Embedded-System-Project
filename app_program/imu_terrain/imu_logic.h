/*
 *	imu_logic.h
 *	IMU/terrain -- pure hump-height / shock logic (no hardware, no kernel).
 *	Unit-tested on a PC (testing/host) as well as on the target.
 *	Integer only, 32-bit only: the RP2040's Cortex-M0+ has no FPU.
 *
 *	Two ways to get the hump height, chosen by IMU_HUMP_MODE:
 *
 *	HUMP_MODE_ROLL (default, our course): the hump is narrow, so only ONE
 *	drive wheel climbs it while the other wheel and the caster stay on the
 *	floor.  The car leans sideways, and the lifted wheel's height is
 *	         h = CAR_TRACK_MM x tan(roll)
 *	         tan(roll) = side / sqrt(fwd^2 + up^2)
 *	This is exact for that geometry, needs no distance, and is not upset by
 *	speed changes (they only show on the forward axis) or by the board being
 *	tilted nose-up on its mount.  Only a lean with the hump-side wheel UP
 *	counts (side axis signed in imu_terrain.h): the car also sways a few mm
 *	either way on a flat floor, and a lean the other way is never a hump.
 *
 *	Tuned on a recording from the car (2026-11-01, testing/imu_terrain/logs):
 *	the forward axis shakes ~330 mg rms at ~24 Hz (motor/gears), the car
 *	sways about +/-6 mm on the flat, and a ~31 mm hump shows clearly once
 *	the axes are averaged over 8 samples and low-pass filtered (~160 ms).
 *
 *	HUMP_MODE_PITCH (a hump across the whole track, both wheels and caster
 *	go over it): Tribunal 1, A3, "filtered pitch x encoder distance":
 *
 *	  pitch  from the accelerometer.  At a steady speed the forward axis
 *	         reads g*sin(pitch).  The car's own acceleration, worked out from
 *	         the encoders, is subtracted first, so the PID (and Mission's
 *	         NAV_SLOW at the hump) does not look like tilt.  The board's
 *	         mounting tilt, measured while the car stands still, is removed.
 *	  ds     distance from app_state.enc_left/right (raw counts, both wheels).
 *	  height The car stands on two contact points, drive wheels and caster,
 *	         CAR_WHEELBASE_MM apart, and its pitch is the slope BETWEEN them:
 *	             b sin(pitch) = y_lead - y_trail
 *	         The trailing point is where the leading point was b mm ago, so
 *	             y_lead(X) = y_lead(X - b) + b sin(pitch(X))
 *	         rebuilds the real surface height for any hump length.  (The
 *	         plain SUM(sin x ds) gives the hump averaged over the wheelbase:
 *	         a 20 mm hump 100 mm long would read about 10 mm.)
 *	  check  after the hump the height should come back to ~0; what is left
 *	         is kept in residual_um (a measure of the pitch error).
 *
 *	Shock (FR-06): the acceleration vector is more than SHOCK_MG away from
 *	the rest vector for fewer than SHOCK_MAX_SAMPLES samples.
 *
 *	*** Every constant marked [CAL] is a placeholder until measured on the car.
 */

#ifndef IMU_LOGIC_H
#define IMU_LOGIC_H

/* ---- States (design review 6.2) ---- */
#define IMU_CALIBRATING		0
#define IMU_MONITORING		1
#define IMU_HUMP_TRACKING	2
#define IMU_HUMP_COMPLETE	3	/* one sample, then back to MONITORING */
#define IMU_SHOCK_EVENT		4	/* one sample, then back to MONITORING */
#define IMU_SENSOR_FAULT	5

/* What imu_feed() reports for a sample */
#define IMU_EV_NONE		0
#define IMU_EV_HUMP_START	1	/* hump confirmed (>= HUMP_MIN_UM) */
#define IMU_EV_HUMP_DONE	2	/* height_mm valid */
#define IMU_EV_SHOCK		3

/* ---- Which method (see the top of this file) ---- */
#define HUMP_MODE_ROLL		0	/* narrow hump under one wheel (our course) */
#define HUMP_MODE_PITCH		1	/* hump across the whole track */
#ifndef IMU_HUMP_MODE
#define IMU_HUMP_MODE		HUMP_MODE_ROLL
#endif

/* ---- Roll mode [CAL] ----
   MEASURE: centre of the left wheel's contact to centre of the right
   wheel's contact (across the car), in mm.  The height scales with it:
   5 mm wrong on 110 mm = 4.5 % of the height (1 mm on a 22 mm hump). */
#define CAR_TRACK_MM		110
#define ROLL_START_UM		5000	/* wheel lifted > 5 mm: start tracking (sway is ~+/-6 mm, mostly the other way) */
#define ROLL_LEVEL_UM		3000	/* below 3 mm (or leaning the other way) counts as level */
#define ROLL_MIN_UM		8000	/* a hump must lift the wheel 8 mm (smaller is sway) [CAL] */
#define ROLL_TAN_MAX		24575	/* tan clamp, Q15 (37 deg): keeps the maths in 32 bits */

/* ---- Pitch mode: car geometry [CAL] ----
   MEASURE: centre of the drive-wheel contact to the centre of the caster
   contact, in mm.  10 mm wrong costs about 3 mm of height.
   0 = plain SUM(sin x ds) (only for comparison). */
#define CAR_WHEELBASE_MM	100
#define PROF_RING_MM		160	/* history kept, must be > CAR_WHEELBASE_MM */

/* ---- Sampling and filters ---- */
#define IMU_TICK_MS		10	/* NFR-06 */
#ifndef IMU_BOX
#define IMU_BOX			8	/* first: average of the last 8 samples (cancels 12.5/25/37.5 Hz) */
#endif
#ifndef IMU_LPF_SHIFT
#define IMU_LPF_SHIFT		4	/* then f += (x - f) / 16: ~160 ms, removes motor vibration [CAL] */
#endif
#define IMU_LPF_FRAC		2	/* filter keeps 2 extra fraction bits (mg * 4) */
#define IMU_SPIKE_MG		1500	/* a sample this far from the filter is a knock: skipped
					   (the car's normal shaking reaches ~800 mg) */
#define IMU_SPIKE_MAX		3	/* ... unless it lasts longer (then it is real) */
#define ACC_WIN			5	/* encoder acceleration over 2 x 50 ms; it is
					   centred ACC_WIN samples back, so the
					   accelerometer is delayed by as much */
#define ACC_UM_S2_PER_DIFF	400	/* 1e6 / (ACC_WIN * IMU_TICK_MS)^2  (ms -> s) */
#define UM_S2_PER_MG		9807	/* 1 mg = 9.80665 mm/s^2 */

/* ---- Calibration: car standing still on flat ground ---- */
#define IMU_CAL_SAMPLES		100	/* 1 s */
#define CAL_G_MIN_MG		750	/* |a| must look like 1 g (one board read 885 mg: */
#define CAL_G_MAX_MG		1250	/* the sensor's scale can be ~12 % off) ... */
#define CAL_SPREAD_MG		150	/* ... and stay within this per axis [CAL] */

/* ---- Hump detector [CAL] ---- */
#define Q15_ONE			32767
#define HUMP_START_SIN		1144	/* sin(2 deg) in Q15: pitch above this starts tracking */
#define HUMP_MIN_UM		3000	/* must reach 3 mm to count as a hump */
#define HUMP_HIST		16	/* samples kept before the start (gentle ramp) */
#ifndef HUMP_SMOOTH
#define HUMP_SMOOTH		8	/* height averaged over 8 samples (~10 mm) */
#endif
#define HUMP_QUIET_END		20	/* level for 20 moving samples ... */
#define HUMP_END_DIV		4	/* ... and height below peak / 4 = hump over */
#define HUMP_MAX_LEN_MM		1000	/* longer = a slope or ramp, given up */
#define HUMP_ZERO_EXTRA_MM	50	/* level for wheelbase + this: profile back to 0 */
#define HUMP_STILL_DROP		300	/* tracking while stopped this long (3 s) is dropped: the car
					   was being handled, not driven over a hump */

/* Minimum-bit range (Tribunal 1, Version C): heights are 0..HUMP_MAX_MM
   (6 bits).  A hump above it is still reported, but never becomes the
   tallest of the run, and Mission should set FAULT_HUMP_RANGE. [agree] */
#ifndef HUMP_MAX_MM
#define HUMP_MAX_MM		63
#endif

/* ---- Shock (FR-06, telemetry only) ---- */
#define SHOCK_MG		3000	/* 3 g away from the rest vector ... */
#define SHOCK_MAX_SAMPLES	3	/* ... for fewer than 3 samples */

/* Accelerometer sample in car axes, mg */
typedef struct {
	INT	fwd;			/* + = towards the front */
	INT	side;
	INT	up;			/* + = up, about +1000 at rest */
} IMU_VEC;

typedef struct {
	UB	state;			/* IMU_xxx */
	BOOL	calibrated;		/* mounting tilt measured */

	/* calibration */
	INT	cal_n;
	W	cal_sum[3];
	INT	cal_min[3], cal_max[3];
	IMU_VEC	rest;			/* mean at rest (the baseline), mg */
	INT	s0, c0;			/* sin / cos of the mounting tilt, Q15 */
	INT	t0;			/* tan of the mounting roll, Q15 */
	UB	mode;			/* HUMP_MODE_xxx */

	/* pitch */
	INT	box[3][IMU_BOX];	/* last samples for the 8-sample average */
	W	box_sum[3];
	INT	box_i, box_n;
	BOOL	primed;
	INT	f[3];			/* filtered fwd, side, up: mg << IMU_LPF_FRAC */
	INT	spike_n;
	INT	sin_q15;		/* pitch now */
	INT	tan_roll_q15;		/* roll now, tan, mounting removed */

	/* encoder acceleration, and the samples delayed to line up with it */
	INT	acc_ds[2 * ACC_WIN];
	INT	acc_i, acc_n;
	IMU_VEC	dly[ACC_WIN];

	/* surface profile */
	INT	b_mm;
	W	pos_um;
	INT	last_mm;
	INT	ring[PROF_RING_MM];	/* y_lead at each mm of travel, um */
	INT	prof_um;
	INT	plain_rem;		/* remainder for the plain-sum mode */
	INT	sm[HUMP_SMOOTH];
	INT	sm_i;

	/* hump detector */
	BOOL	tracking;
	BOOL	confirmed;
	INT	quiet_n;
	INT	still_n;		/* samples without movement while tracking */
	W	flat_um;
	INT	hist[HUMP_HIST];
	INT	hist_i;
	INT	h_ref;
	INT	rel_um;			/* height above the level before the hump */
	INT	peak_um;
	W	len_um;

	/* shock */
	INT	over_cnt;
	BOOL	shock_pending;

	/* results */
	INT	height_mm;		/* last completed hump */
	INT	max_height_mm;		/* tallest of the run, 0..HUMP_MAX_MM */
	INT	residual_um;		/* height left after the last hump */
	INT	len_mm;			/* length of the last hump */
	UW	humps;			/* completed humps */
	UW	aborts;			/* humps given up (turn, slope, fault) */

	/* per-run statistics (tests): since imu_run_reset() */
	INT	run_peak_um;		/* highest lean height seen (roll x track), um */
	INT	run_dev[3];		/* largest change from rest: fwd, side, up, mg */
} IMU_LOGIC;

IMPORT void imu_logic_init(IMU_LOGIC *l);
IMPORT void imu_set_wheelbase(IMU_LOGIC *l, INT b_mm);	/* tests only */
IMPORT void imu_set_mode(IMU_LOGIC *l, UB mode);	/* tests only */

/* One 10 ms sample.  a = acceleration in car axes; ds_um = distance moved
   since the last sample (signed); still = car stopped and not moving
   (calibration allowed); allowed = car driving straight or stopped (FALSE
   while turning, bypassing or faulty: the hump is given up).
   Returns IMU_EV_xxx. */
IMPORT INT  imu_feed(IMU_LOGIC *l, const IMU_VEC *a, INT ds_um, BOOL still, BOOL allowed);

IMPORT void imu_fault(IMU_LOGIC *l);		/* I2C reads failing */

/* Tests: clear the per-run statistics; and when the car has stopped, close
   a hump that is still under way (returns IMU_EV_HUMP_DONE if it was one). */
IMPORT void imu_run_reset(IMU_LOGIC *l);
IMPORT INT  imu_finish(IMU_LOGIC *l);
IMPORT void imu_recovered(IMU_LOGIC *l);	/* sensor answers again */

/* Helpers, also used by the tests */
IMPORT UW   imu_isqrt(UW v);
IMPORT INT  imu_counts_to_um(INT *rem, INT dcounts, INT mm_per_rev, INT counts_per_rev);
IMPORT INT  imu_pitch_deg(const IMU_LOGIC *l);	/* whole degrees, small-angle */
IMPORT INT  imu_height_now_um(const IMU_LOGIC *l);

#endif /* IMU_LOGIC_H */
