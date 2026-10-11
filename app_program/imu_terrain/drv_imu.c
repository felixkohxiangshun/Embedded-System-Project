/*
 *	drv_imu.c
 *	IMU/terrain -- LSM303DLHC accelerometer over the kernel I2C driver.
 *
 *	Settings: 100 Hz, all axes on (CTRL_REG1_A = 0x57); +/-8 g and
 *	high-resolution mode (CTRL_REG4_A = 0x28), which gives 4 mg per LSB on
 *	the 12-bit value left-justified in the 16-bit output.  +/-8 g is needed
 *	because the shock threshold is 3 g above baseline.
 *
 *	Only imu_task calls these functions (one owning task per peripheral).
 *	The kernel I2C driver serialises transfers inside the device driver.
 */

#include "imu_terrain.h"
#include "../../device/include/dev_i2c.h"

#define ACC_ADDR		0x19	/* 7-bit */
#define REG_CTRL1_A		0x20
#define REG_CTRL4_A		0x23
#define REG_OUT_Z_L_A		0x2C
#define REG_OUT_Z_H_A		0x2D
#define MG_PER_LSB		4	/* +/-8 g, high-resolution mode */

LOCAL ID dd = -1;

EXPORT ER imu_hw_open(void)
{
	ER er;

	if (dd < E_OK) {
		dd = tk_opn_dev((UB*)"iica", TD_UPDATE);	/* I2C unit 0 */
		if (dd < E_OK) {
			return dd;
		}
	}
	er = i2c_write_reg(dd, ACC_ADDR, REG_CTRL1_A, 0x57);
	if (er < E_OK) {
		return er;
	}
	return i2c_write_reg(dd, ACC_ADDR, REG_CTRL4_A, 0x28);
}

EXPORT ER imu_read_z_mg(INT *z_mg)
{
	UB  lo = 0, hi = 0;
	ER  er;
	INT raw;

	er = i2c_read_reg(dd, ACC_ADDR, REG_OUT_Z_L_A, &lo);
	if (er < E_OK) {
		return er;
	}
	er = i2c_read_reg(dd, ACC_ADDR, REG_OUT_Z_H_A, &hi);
	if (er < E_OK) {
		return er;
	}
	raw = (INT)(short)(((UH)hi << 8) | lo);		/* signed 16-bit */
	*z_mg = (raw / 16) * MG_PER_LSB;		/* 12-bit left-justified */
	return E_OK;
}
