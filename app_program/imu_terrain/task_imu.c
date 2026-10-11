/*
 *	task_imu.c
 *	IMU/terrain -- the IMU / terrain task (design review 6.2).
 *
 *	States  CALIBRATING, MONITORING, HUMP_TRACKING, HUMP_COMPLETE,
 *	        SHOCK_EVENT, SENSOR_FAULT   (the maths is in imu_logic.c)
 *
 *	The task samples the Z axis every 10 ms and posts hump / shock events to
 *	Mission on mbf_terrain.  A failed I2C read puts it in SENSOR_FAULT: it
 *	keeps retrying and posts TERR_RECOVERED when the sensor answers.  A fault
 *	only costs terrain data; it never stops the car (FR-19).
 */

#include "imu_terrain.h"

EXPORT volatile UW s3_ticks;
EXPORT volatile UB s3_state;
EXPORT volatile UW s3_i2c_errors;

EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_imu);
}

LOCAL void post(UB kind, const IMU_LOGIC *l)
{
	MSG_TERRAIN m;

	m.kind = kind;
	m.height_mm = l->height_mm;
	m.max_mm = l->max_height_mm;
	MBF_SEND(mbf_terrain, m);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	IMU_LOGIC l;
	INT       z, ev;
	ER        er;

	(void)stacd; (void)exinf;
	imu_logic_init(&l);

	er = imu_hw_open();
	if (er < E_OK) {
		s3_i2c_errors++;
		imu_fault(&l);
		post(TERR_FAULT, &l);
	}

	while (1) {
		tk_slp_tsk(TMO_FEVR);		/* woken by imu_cyclic_handler */
		tk_can_wup(TSK_SELF);
		s3_ticks++;

		if (l.state == IMU_SENSOR_FAULT) {
			/* retry: re-configure, then read once to prove it works */
			er = imu_hw_open();
			if (er >= E_OK) {
				er = imu_read_z_mg(&z);
			}
			if (er >= E_OK) {
				imu_recovered(&l);
				post(TERR_RECOVERED, &l);
			} else {
				s3_i2c_errors++;
			}
			s3_state = l.state;
			continue;
		}

		er = imu_read_z_mg(&z);
		if (er < E_OK) {
			s3_i2c_errors++;
			imu_fault(&l);
			post(TERR_FAULT, &l);
			s3_state = l.state;
			continue;
		}

		ev = imu_feed(&l, z);
		switch (ev) {
		case IMU_EV_HUMP_START:
			post(TERR_HUMP_START, &l);
			break;
		case IMU_EV_HUMP_DONE:
			post(TERR_HUMP_DONE, &l);
			break;
		case IMU_EV_SHOCK:
			post(TERR_SHOCK, &l);
			break;
		default:
			break;
		}
		s3_state = l.state;
	}
}
