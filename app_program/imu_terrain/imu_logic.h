/*
 *	imu_logic.h
 *	IMU/terrain -- pure hump / shock detection (no hardware, no kernel).
 *	Unit-tested on a PC as well as on the target.
 *
 *	The Z-axis acceleration is compared with a rest baseline measured while
 *	the car stands still (CALIBRATING).  A rise above HUMP_START_MG starts
 *	HUMP_TRACKING; the peak is kept, and when Z falls back below HUMP_END_MG
 *	the hump is complete.  A spike above SHOCK_MG that lasts fewer than
 *	SHOCK_MAX_SAMPLES samples is a SHOCK (FR-06).
 *
 *	*** [CAL] HUMP_MM_PER_G is a placeholder.  The height of a hump is NOT
 *	*** a direct function of vertical acceleration: it depends on speed and
 *	*** wheel geometry.  Fit the constant on humps of known height at the
 *	*** operating speed (NFR-07 asks for +/- 5 mm) and revisit the method if
 *	*** the spread is too large.
 */

#ifndef IMU_LOGIC_H
#define IMU_LOGIC_H

#define IMU_CALIBRATING		0
#define IMU_MONITORING		1
#define IMU_HUMP_TRACKING	2
#define IMU_HUMP_COMPLETE	3	/* one sample, then back to MONITORING */
#define IMU_SHOCK_EVENT		4	/* one sample, then back to MONITORING */
#define IMU_SENSOR_FAULT	5

#define IMU_CAL_SAMPLES		32	/* 320 ms at rest */
#define HUMP_START_MG		150
#define HUMP_END_MG		80
#define SHOCK_MG		3000	/* FR-06: 3 g above baseline */
#define SHOCK_MAX_SAMPLES	3	/* ... lasting fewer than 3 samples */
#define HUMP_MM_PER_G		25	/* mm of height per g of peak [CAL] */

/* What imu_feed() reports for a sample */
#define IMU_EV_NONE		0
#define IMU_EV_HUMP_START	1
#define IMU_EV_HUMP_DONE	2
#define IMU_EV_SHOCK		3

typedef struct {
	UB	state;
	INT	baseline_mg;
	INT	cal_sum;
	INT	cal_n;
	INT	peak_mg;
	INT	over_cnt;		/* consecutive samples above SHOCK_MG */
	INT	height_mm;		/* last completed hump */
	INT	max_height_mm;		/* tallest of the run */
} IMU_LOGIC;

IMPORT void imu_logic_init(IMU_LOGIC *l);
IMPORT INT  imu_feed(IMU_LOGIC *l, INT z_mg);		/* returns IMU_EV_xxx */
IMPORT void imu_fault(IMU_LOGIC *l);			/* I2C read failed */
IMPORT void imu_recovered(IMU_LOGIC *l);		/* read works: recalibrate */

#endif /* IMU_LOGIC_H */
