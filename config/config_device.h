/*
 *----------------------------------------------------------------------
 *    Device Driver for μT-Kernel 3.00.05
 *
 *    Copyright (C) 2020-2021 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2021/11.
 *
 *----------------------------------------------------------------------
 */


/*
 *	config_device.h
 *	Device configuration definition
 */

#ifndef	__DEV_CONFIG_H__
#define	__DEV_CONFIG_H__

/* ------------------------------------------------------------------------ */
/* Device usage settings
 *	1: Use   0: Do not use
 */

#define DEVCNF_USE_SER		1		// Serial communication device 
#define DEVCNF_USE_ADC		0		// A/D conversion device (OFF: the IR sampler in subsystem 2 owns the ADC, see TEAM.md)
#define DEVCNF_USE_IIC		1		// I2C communication device (ON: IMU on I2C0, GP4/GP5)
		/* I2C0 pins are chosen in device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h
		   (SDA GP4, SCL GP5 for this project). */

#endif	/* __DEV_CONFIG_H__ */