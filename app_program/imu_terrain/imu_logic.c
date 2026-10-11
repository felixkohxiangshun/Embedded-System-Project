/*
 *	imu_logic.c
 *	IMU/terrain -- pure hump / shock detection.  See imu_logic.h.
 */

#include "imu_terrain.h"

LOCAL INT iabs(INT v)
{
	return v < 0 ? -v : v;
}

EXPORT void imu_logic_init(IMU_LOGIC *l)
{
	l->state = IMU_CALIBRATING;
	l->baseline_mg = 0;
	l->cal_sum = 0;
	l->cal_n = 0;
	l->peak_mg = 0;
	l->over_cnt = 0;
	l->height_mm = 0;
	l->max_height_mm = 0;
}

EXPORT void imu_fault(IMU_LOGIC *l)
{
	l->state = IMU_SENSOR_FAULT;
	l->over_cnt = 0;
}

/* After a fault the baseline may have changed (tilt, power), so measure it
   again.  Note this assumes the car is level and not on a hump. */
EXPORT void imu_recovered(IMU_LOGIC *l)
{
	l->state = IMU_CALIBRATING;
	l->cal_sum = 0;
	l->cal_n = 0;
	l->over_cnt = 0;
}

EXPORT INT imu_feed(IMU_LOGIC *l, INT z_mg)
{
	INT dev, ev = IMU_EV_NONE;
	INT was_over;

	if (l->state == IMU_SENSOR_FAULT) {
		return IMU_EV_NONE;
	}
	if (l->state == IMU_CALIBRATING) {
		l->cal_sum += z_mg;
		if (++l->cal_n >= IMU_CAL_SAMPLES) {
			l->baseline_mg = l->cal_sum / l->cal_n;
			l->state = IMU_MONITORING;
		}
		return IMU_EV_NONE;
	}
	if (l->state == IMU_HUMP_COMPLETE || l->state == IMU_SHOCK_EVENT) {
		l->state = IMU_MONITORING;		/* the event was one sample long */
	}

	dev = iabs(z_mg - l->baseline_mg);

	/* shock: a short spike above SHOCK_MG, judged when it ends */
	was_over = l->over_cnt;
	if (dev > SHOCK_MG) {
		l->over_cnt++;
	} else {
		l->over_cnt = 0;
		if (was_over > 0 && was_over < SHOCK_MAX_SAMPLES) {
			l->state = IMU_SHOCK_EVENT;
			return IMU_EV_SHOCK;
		}
	}
	if (l->over_cnt > 0) {
		return IMU_EV_NONE;			/* inside a spike: not a hump */
	}

	if (l->state == IMU_MONITORING) {
		if (dev > HUMP_START_MG) {
			l->state = IMU_HUMP_TRACKING;
			l->peak_mg = dev;
			ev = IMU_EV_HUMP_START;
		}
	} else if (l->state == IMU_HUMP_TRACKING) {
		if (dev > l->peak_mg) {
			l->peak_mg = dev;
		}
		if (dev < HUMP_END_MG) {
			l->height_mm = (l->peak_mg * HUMP_MM_PER_G) / 1000;
			if (l->height_mm > l->max_height_mm) {
				l->max_height_mm = l->height_mm;
			}
			l->state = IMU_HUMP_COMPLETE;
			ev = IMU_EV_HUMP_DONE;
		}
	}
	return ev;
}
