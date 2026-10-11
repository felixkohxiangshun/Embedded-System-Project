/*
 *	mission_obstacle.c
 *	Mission/obstacle -- creates the Mission-side message buffers and the shared
 *	state mutex, and starts the three tasks.
 *
 */

#include "mission_obstacle.h"

EXPORT ID	mbf_steer;	/* IF-01  Line      -> Mission */
EXPORT ID	mbf_bccmd;	/* IF-03  Barcode   -> Mission */
EXPORT ID	mbf_terrain;	/* IF-04  IMU       -> Mission */
EXPORT ID	mbf_obs;	/* IF-05  Scan      -> Mission */
EXPORT ID	mbf_scancmd;	/* IF-06  Mission   -> Scan    */
EXPORT ID	mbf_mqttcmd;	/* IF-07  Dashboard -> Mission */
EXPORT ID	mtx_state;	/* IF-11  guards app_state     */
EXPORT APP_STATE app_state;

EXPORT ID	tid_mission;
EXPORT ID	tid_scan;

/* Lock the shared state.  The mutex uses priority inheritance (TA_INHERIT):
   if a low-priority task such as Telemetry holds it while Motor wants it,
   Telemetry is raised to Motor's priority until it unlocks, so Motor is
   never held up by a mid-priority task. */
EXPORT ER state_lock(TMO tmout)
{
	return tk_loc_mtx(mtx_state, tmout);
}

EXPORT void state_unlock(void)
{
	tk_unl_mtx(mtx_state);
}

LOCAL ER make_mbf(ID *id, INT size, INT count)
{
	*id = app_make_mbf(size, count);
	return (*id < E_OK) ? *id : E_OK;
}

EXPORT ER mission_obstacle_init(void)
{
	T_CMTX cmtx = {0};
	ER     er;

	er = make_mbf(&mbf_steer,   (INT)sizeof(MSG_STEER),   32);
	if (er < E_OK) return er;
	er = make_mbf(&mbf_bccmd,   (INT)sizeof(MSG_BCCMD),    8);
	if (er < E_OK) return er;
	er = make_mbf(&mbf_terrain, (INT)sizeof(MSG_TERRAIN),  8);
	if (er < E_OK) return er;
	er = make_mbf(&mbf_obs,     (INT)sizeof(MSG_OBS),      4);
	if (er < E_OK) return er;
	er = make_mbf(&mbf_scancmd, (INT)sizeof(MSG_SCANCMD),  4);
	if (er < E_OK) return er;
	er = make_mbf(&mbf_mqttcmd, (INT)sizeof(MSG_MQTTCMD),  4);
	if (er < E_OK) return er;

	cmtx.mtxatr = TA_INHERIT;
	mtx_state = tk_cre_mtx(&cmtx);
	if (mtx_state < E_OK) {
		return mtx_state;
	}
	app_state.sys_state = SYS_INIT;
	return E_OK;
}

EXPORT ER mission_obstacle_start(void)
{
	ultra_hw_init();

	tid_mission = app_start_task(mission_task, PRI_MISSION, 0, "mission_task");
	if (tid_mission < E_OK) {
		return tid_mission;
	}
	tid_scan = app_start_task(scan_task, PRI_SCAN, 0, "scan_task");
	return (tid_scan < E_OK) ? tid_scan : E_OK;
}
