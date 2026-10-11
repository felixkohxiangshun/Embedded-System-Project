/*
 *	imu_terrain_test.c
 *	On-target tests for the IMU/terrain subsystem (IMU / terrain).
 *	Built only with: make TEST=1.  Results: testing/imu_terrain/TEST_LOG.md
 */

#include "imu_terrain.h"
#include "../test_report.h"

#if APP_TEST

/* Feed n samples of the same value, return the last event that was not NONE */
LOCAL INT feed(IMU_LOGIC *l, INT z, INT n)
{
	INT i, ev, last = IMU_EV_NONE;

	for (i = 0; i < n; i++) {
		ev = imu_feed(l, z);
		if (ev != IMU_EV_NONE) {
			last = ev;
		}
	}
	return last;
}

LOCAL void calibrated(IMU_LOGIC *l)
{
	imu_logic_init(l);
	feed(l, 1000, IMU_CAL_SAMPLES);		/* 1 g at rest */
}

EXPORT void imu_terrain_test(void)
{
	IMU_LOGIC l;
	UW        t0;
	INT       n, ev;

	TEST_INIT();

	/* S3-U01: baseline is measured, then quiet samples give no event */
	calibrated(&l);
	TEST_CHECK("S3-U01", l.state == IMU_MONITORING && l.baseline_mg == 1000 &&
		   feed(&l, 1020, 50) == IMU_EV_NONE, "calibrates, then a flat road is quiet");

	/* S3-U02: a bump above the threshold is tracked and completes */
	calibrated(&l);
	ev = imu_feed(&l, 1400);
	TEST_CHECK("S3-U02a", ev == IMU_EV_HUMP_START && l.state == IMU_HUMP_TRACKING,
		   "Z above threshold -> HUMP_TRACKING");
	imu_feed(&l, 1600);
	ev = imu_feed(&l, 1010);
	TEST_CHECK("S3-U02b", ev == IMU_EV_HUMP_DONE && l.peak_mg == 600 &&
		   l.height_mm == (600 * HUMP_MM_PER_G) / 1000, "Z back at baseline -> hump height");

	/* S3-U03: the tallest hump of the run is kept (FR-05) */
	imu_feed(&l, 1000);
	imu_feed(&l, 1200); imu_feed(&l, 1005);		/* a smaller hump */
	TEST_CHECK("S3-U03", l.max_height_mm == (600 * HUMP_MM_PER_G) / 1000, "tallest hump is kept");

	/* S3-U04: a 2-sample spike above 3 g is a SHOCK (FR-06) */
	calibrated(&l);
	imu_feed(&l, 1000 + SHOCK_MG + 500);
	imu_feed(&l, 1000 + SHOCK_MG + 500);
	ev = imu_feed(&l, 1000);
	TEST_CHECK("S3-U04", ev == IMU_EV_SHOCK, "short spike above 3 g -> SHOCK");

	/* S3-U05: a long high reading (3+ samples) is not a shock */
	calibrated(&l);
	feed(&l, 1000 + SHOCK_MG + 500, 5);
	ev = imu_feed(&l, 1000);
	TEST_CHECK("S3-U05", ev != IMU_EV_SHOCK, "a long reading is not a SHOCK");

	/* S3-U06: fault, no events while faulted, recovery recalibrates */
	calibrated(&l);
	imu_fault(&l);
	TEST_CHECK("S3-U06a", l.state == IMU_SENSOR_FAULT && imu_feed(&l, 5000) == IMU_EV_NONE,
		   "SENSOR_FAULT ignores samples");
	imu_recovered(&l);
	TEST_CHECK("S3-U06b", l.state == IMU_CALIBRATING, "recovery -> CALIBRATING");

	/* S3-U07: the IMU task runs every 10 ms (NFR-06) even if the sensor is
	   missing, because a failed read must not stop the task */
	t0 = s3_ticks;
	tk_dly_tsk(1000);
	n = (INT)(s3_ticks - t0);
	TEST_CHECK("S3-U07", n >= 95 && n <= 105, "IMU task runs every 10 ms");

	/* S3-U08 (hardware): the sensor answers on I2C0 */
	TEST_CHECK("S3-H01", s3_state != IMU_SENSOR_FAULT, "LSM303DLHC answers on I2C (needs the sensor wired)");

	TEST_SUMMARY("imu_terrain");
}

#endif /* APP_TEST */
