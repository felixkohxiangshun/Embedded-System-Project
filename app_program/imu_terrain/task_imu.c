/*
 *	task_imu.c
 *	IMU/terrain -- the IMU / terrain task (design review 6.2).
 *
 *	States  CALIBRATING, MONITORING, HUMP_TRACKING, HUMP_COMPLETE,
 *	        SHOCK_EVENT, SENSOR_FAULT   (the maths is in imu_logic.c)
 *
 *	Every 10 ms:
 *	  1. read enc_left / enc_right / motor_state from app_state (TMO_POL)
 *	  2. distance since the last sample, from the encoder counts
 *	  3. read X, Y, Z from the accelerometer
 *	  4. imu_feed(): pitch, surface height, hump / shock events
 *	  5. post events to Mission on mbf_terrain
 *	A failed I2C read keeps its distance for the next good sample.  Three in
 *	a row put the task in SENSOR_FAULT: it keeps retrying and posts
 *	TERR_RECOVERED when the sensor answers.  A fault only costs terrain
 *	data; it never stops the car (FR-19).
 */

#include "imu_terrain.h"

EXPORT volatile UW  s3_ticks;
EXPORT volatile UB  s3_state;
EXPORT volatile UW  s3_i2c_errors;
EXPORT volatile INT s3_pitch_deg;
EXPORT volatile INT s3_h_um;
EXPORT volatile INT s3_residual_um;
EXPORT volatile INT s3_raw_mg[3];
EXPORT volatile UB  s3_run_cmd;		/* S3_CMD_xxx from the test, cleared when done */
EXPORT volatile INT s3_run_peak_um;
EXPORT volatile INT s3_run_dev[3];
EXPORT volatile BOOL s3_calibrated;
EXPORT volatile INT s3_rest[3];
EXPORT volatile UW  s3_humps;
EXPORT volatile INT s3_last_mm;
EXPORT volatile INT s3_max_mm;

#if APP_TEST
/* Raw recording for S3_TEST_LOG (test builds only): one line per 10 ms */
EXPORT volatile BOOL s3_log_on;
EXPORT volatile INT  s3_log_n;
EXPORT H  s3_log_raw[S3_LOG_MAX][3];	/* sensor X, Y, Z, mg (as read, not remapped) */
EXPORT H  s3_log_ds[S3_LOG_MAX];	/* distance this tick, um */
EXPORT H  s3_log_h[S3_LOG_MAX];		/* roll height now, 0.1 mm */
EXPORT UB s3_log_st[S3_LOG_MAX];	/* motor state * 16 + IMU state */
#endif

/* ~1 KB: static, not on the 2 KB task stack */
LOCAL IMU_LOGIC	lg;

LOCAL BOOL	have_last;
LOCAL INT	last_l, last_r;
LOCAL INT	um_rem;			/* remainder of the counts -> um conversion */
LOCAL INT	ds_carry;		/* distance of samples that failed to read */

LOCAL BOOL	pend;			/* one message kept if mbf_terrain was full */
LOCAL MSG_TERRAIN pend_msg;

EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_imu);
}

LOCAL void post(UB kind)
{
	MSG_TERRAIN m;

	m.kind = kind;
	m.height_mm = lg.height_mm;
	m.max_mm = lg.max_height_mm;
	if (MBF_SEND(mbf_terrain, m) < E_OK) {
		pend = TRUE;		/* never block: try again next tick */
		pend_msg = m;
	}
}

LOCAL void flush_pending(void)
{
	if (pend && (MBF_SEND(mbf_terrain, pend_msg) >= E_OK)) {
		pend = FALSE;
	}
}

/* Motor's encoder counts and state.  FALSE if the lock is busy: the counts
   are cumulative, so the distance is simply picked up next time. */
LOCAL BOOL read_motion(INT *el, INT *er, UB *ms)
{
	if (state_lock(TMO_POL) < E_OK) {
		return FALSE;
	}
	*el = app_state.enc_left;
	*er = app_state.enc_right;
	*ms = app_state.motor_state;
	state_unlock();
	return TRUE;
}

/* Mean distance of both wheels since the last call, um (signed) */
LOCAL INT step_distance(BOOL ok, INT el, INT er)
{
	INT dsum;

	if (!ok) {
		return 0;
	}
	if (!have_last) {
		last_l = el;
		last_r = er;
		have_last = TRUE;
		return 0;
	}
	dsum = (el - last_l) + (er - last_r);
	last_l = el;
	last_r = er;
	return imu_counts_to_um(&um_rem, dsum, WHEEL_MM_PER_REV, 2 * ENC_COUNTS_PER_REV);
}

LOCAL void to_car_axes(const INT raw[3], IMU_VEC *a)
{
	a->fwd = IMU_SIGN_FWD * raw[IMU_AXIS_FWD];
	a->up = IMU_SIGN_UP * raw[IMU_AXIS_UP];
	a->side = IMU_SIGN_SIDE * raw[3 - IMU_AXIS_FWD - IMU_AXIS_UP];
}

LOCAL void hump_done(void)
{
	s3_residual_um = lg.residual_um;
	post(TERR_HUMP_DONE);
#if IMU_LOG
	tm_printf((UB*)"[IMU] hump %d mm (tallest %d, residual %d um, %d mm long)\n",
		  lg.height_mm, lg.max_height_mm, lg.residual_um, lg.len_mm);
#endif
}

/* Requests from the test task (imu_terrain_test.c), done here because only
   this task touches lg */
LOCAL void run_command(void)
{
	if (s3_run_cmd == S3_CMD_RESET) {
		imu_run_reset(&lg);
		s3_run_cmd = S3_CMD_NONE;
	} else if (s3_run_cmd == S3_CMD_FINISH) {
		if (imu_finish(&lg) == IMU_EV_HUMP_DONE) {
			hump_done();
		}
		s3_run_cmd = S3_CMD_NONE;
	}
}

LOCAL void observe(void)
{
	s3_state = lg.state;
	s3_pitch_deg = imu_pitch_deg(&lg);
	s3_h_um = imu_height_now_um(&lg);
	s3_calibrated = lg.calibrated;
	s3_rest[0] = lg.rest.fwd;
	s3_rest[1] = lg.rest.side;
	s3_rest[2] = lg.rest.up;
	s3_humps = lg.humps;
	s3_run_peak_um = lg.run_peak_um;
	s3_run_dev[0] = lg.run_dev[0];
	s3_run_dev[1] = lg.run_dev[1];
	s3_run_dev[2] = lg.run_dev[2];
	s3_last_mm = lg.height_mm;
	s3_max_mm = lg.max_height_mm;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	INT	raw[3];
	IMU_VEC	a;
	INT	el = 0;
	INT	er_r = 0;
	UB	ms = MOT_STOPPED;
	INT	ds, ev, k;
	INT	fail_n = 0;
	INT	retry_n = 0;
	BOOL	ok, still, allowed;
	ER	er;

	(void)stacd; (void)exinf;
	imu_logic_init(&lg);

	er = imu_hw_open();
	if (er < E_OK) {
		s3_i2c_errors++;
		imu_fault(&lg);
		post(TERR_FAULT);
	}

	while (1) {
		tk_slp_tsk(TMO_FEVR);		/* woken by imu_cyclic_handler */
		tk_can_wup(TSK_SELF);
		s3_ticks++;
		flush_pending();

		/* 1-2. distance from Motor's encoder counts */
		ok = read_motion(&el, &er_r, &ms);
		ds = step_distance(ok, el, er_r) + ds_carry;
		ds_carry = 0;

		if (lg.state == IMU_SENSOR_FAULT) {
			/* retry every 100 ms: re-configure, then read once to prove it */
			if (++retry_n >= IMU_RETRY_TICKS) {
				retry_n = 0;
				er = imu_hw_open();
				if (er >= E_OK) {
					er = imu_read_accel(raw);
				}
				if (er >= E_OK) {
					fail_n = 0;
					imu_recovered(&lg);
					post(TERR_RECOVERED);
				} else {
					s3_i2c_errors++;
				}
			}
			observe();
			continue;
		}

		/* 3. sample */
		er = imu_read_accel(raw);
		if (er < E_OK) {
			s3_i2c_errors++;
			ds_carry = ds;		/* use this distance with the next sample */
			if (++fail_n >= IMU_FAULT_READS) {
				ds_carry = 0;
				retry_n = 0;
				imu_fault(&lg);
				post(TERR_FAULT);
			}
			observe();
			continue;
		}
		fail_n = 0;
		for (k = 0; k < 3; k++) {
			s3_raw_mg[k] = raw[k];
		}
		to_car_axes(raw, &a);

		/* 4. the maths */
		/* "still" = Motor is stopped.  Not "no encoder counts": an encoder
		   can twitch a few counts at power-up with the motors off, and that
		   used to stop the calibration from ever running. */
		still = (ms == MOT_STOPPED) ? TRUE : FALSE;
		allowed = ((ms == MOT_DRIVING) || (ms == MOT_STOPPED)) ? TRUE : FALSE;
		ev = imu_feed(&lg, &a, ds, still, allowed);

		/* 5. events */
		switch (ev) {
		case IMU_EV_HUMP_START:
			post(TERR_HUMP_START);
#if IMU_LOG
			tm_printf((UB*)"[IMU] hump detected\n");
#endif
			break;
		case IMU_EV_HUMP_DONE:
			hump_done();
			break;
		case IMU_EV_SHOCK:
			post(TERR_SHOCK);
			break;
		default:
			break;
		}
		run_command();
		observe();
#if APP_TEST
		if (s3_log_on && (s3_log_n < S3_LOG_MAX)) {
			INT i = s3_log_n;

			s3_log_raw[i][0] = (H)raw[0];
			s3_log_raw[i][1] = (H)raw[1];
			s3_log_raw[i][2] = (H)raw[2];
			s3_log_ds[i] = (H)((ds > 32767) ? 32767 : ds);
			s3_log_h[i] = (H)(((lg.tan_roll_q15 < 0) ? -lg.tan_roll_q15 : lg.tan_roll_q15)
					  * CAR_TRACK_MM * 125 / 4096 / 100);
			s3_log_st[i] = (UB)((ms << 4) | (lg.state & 0x0F));
			s3_log_n = i + 1;
		}
#endif
	}
}
