/*
 *	imu_terrain.h
 *	IMU AND TERRAIN subsystem -- owner Lau Sook Yee (2503178).
 *	Design review: section 6.2, FR-04..FR-06, NFR-06, NFR-07.
 *
 *	Does:         samples the LSM303DLHC accelerometer every 10 ms, detects
 *	              humps and measures their peak height, keeps the tallest of
 *	              the run, and flags SHOCK impulses (3 g above baseline for
 *	              fewer than 3 samples).  A failed I2C read is a SENSOR_FAULT
 *	              that is retried; it never stops the car (FR-19).
 *	              States: CALIBRATING, MONITORING, HUMP_TRACKING, HUMP_COMPLETE,
 *	              SHOCK_EVENT, SENSOR_FAULT.
 *	Task:         imu_task (PRI_IMU, woken by cyc_imu every 10 ms)
 *	Peripheral:   I2C0 on GP4 (SDA) / GP5 (SCL) through the kernel's I2C driver
 *	              (device "iica"); sensor address 0x19.  Only imu_task uses it.
 *	Publishes:    MSG_TERRAIN on mbf_terrain (IF-04) to Mission
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include "../app.h"
#include "imu_logic.h"

IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;

/* Observable values for the tests */
IMPORT volatile UW s3_ticks;
IMPORT volatile UB s3_state;		/* IMU_xxx */
IMPORT volatile UW s3_i2c_errors;

/* drv_imu.c: the only code that touches the I2C device */
IMPORT ER   imu_hw_open(void);			/* open + configure sensor */
IMPORT ER   imu_read_z_mg(INT *z_mg);		/* Z acceleration in milli-g */

IMPORT void imu_task(INT stacd, void *exinf);
IMPORT void imu_cyclic_handler(void *exinf);

#endif /* IMU_TERRAIN_H */
