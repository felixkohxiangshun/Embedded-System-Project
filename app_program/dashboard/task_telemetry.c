/*
 *	task_telemetry.c
 *	Dashboard -- the Telemetry task.
 *
 *	Every TELEMETRY_MS (FR-16: 500 ms) it copies app_state under mtx_state,
 *	adds the uptime, and sends one MSG_TELEM frame to the Dashboard on
 *	mbf_telem.  Low priority: a late frame only delays the dashboard.  It
 *	waits for the mutex (with priority inheritance), because it has no
 *	deadline to protect; the control tasks never wait for it.
 */

#include "dashboard.h"
#include "../motor_control/motor_ctrl.h"

EXPORT volatile UW s4_telem_sent;

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	MSG_TELEM t;
	SYSTIM    now;

	(void)stacd; (void)exinf;

	while (1) {
		tk_dly_tsk(TELEMETRY_MS);

		if (state_lock(100) != E_OK) {
			continue;
		}
		t.sys_state = app_state.sys_state;
		t.speed_mmps = app_state.speed_mmps;
		t.dist_mm = app_state.dist_mm;
		t.enc_left = app_state.enc_left;
		t.enc_right = app_state.enc_right;
		t.heading_deg = counts_to_deg((app_state.enc_left - app_state.enc_right) / 2);
		t.ir_line = app_state.ir_line;
		t.obs_dist_mm = app_state.obs_dist_mm;
		t.last_barcode = app_state.last_barcode;
		t.hump_max_mm = app_state.hump_max_mm;
		t.shock = app_state.shock;
		t.fault_flags = app_state.fault_flags;
		state_unlock();

		tk_get_otm(&now);		/* ms since boot */
		t.uptime_ms = now.lo;		/* low 32 bits: wraps after 49 days */
		if (MBF_SEND(mbf_telem, t) == E_OK) {
			s4_telem_sent++;
		}
	}
}
