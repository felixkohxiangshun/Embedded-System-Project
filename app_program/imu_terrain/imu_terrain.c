/*
 *	imu_terrain.c
 *	IMU/terrain -- IMU: creates its cyclic handler and starts its task.
 *
 *	This subsystem creates no message buffer: its output buffer
 *	(mbf_terrain) belongs to Mission, which receives it.
 */

#include "imu_terrain.h"

EXPORT ID	tid_imu;
EXPORT ID	cyc_imu;

EXPORT ER imu_terrain_init(void)
{
	T_CCYC ccyc = {0};

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = 10;		/* NFR-06: sample every 10 ms */
	ccyc.cycphs = 10;
	cyc_imu = tk_cre_cyc(&ccyc);
	return (cyc_imu < E_OK) ? cyc_imu : E_OK;
}

EXPORT ER imu_terrain_start(void)
{
	tid_imu = app_start_task(imu_task, PRI_IMU, 0, "imu_task");
	if (tid_imu < E_OK) {
		return tid_imu;
	}
	return tk_sta_cyc(cyc_imu);
}
