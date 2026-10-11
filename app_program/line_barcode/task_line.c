/*
 *	task_line.c
 *	Line/barcode -- the Line Tracking task (design review 6.1).
 *
 *	States  SAMPLING  read the left and centre IR sensors
 *	        STEERING  PD correction, result sent on mbf_steer
 *	        LINE_LOST both sensors off the line
 *
 *	Every 10 ms (NFR-03) the task reads the filtered sensor values, computes
 *	the steering and posts it to Mission.  It never commands the motors.
 *	If the line is lost for LINE_TIMEOUT_MS the message carries
 *	STEER_F_TIMEOUT and Mission enters FAULT_RECOVERY.
 */

#include "line_barcode.h"

EXPORT volatile UW s2_line_ticks;
EXPORT volatile UB s2_line_state;

EXPORT void line_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_line);
}

EXPORT void line_task(INT stacd, void *exinf)
{
	LINE_PD   pd;
	MSG_STEER m;
	UB        state = LN_SAMPLING;
	INT       dl, dc, err, last_err = 0;
	INT       lost_ticks = 0;
	INT       lost_ms = 0;

	(void)stacd; (void)exinf;
	line_pd_init(&pd);

	while (1) {
		tk_slp_tsk(TMO_FEVR);		/* woken by line_cyclic_handler */
		tk_can_wup(TSK_SELF);
		s2_line_ticks++;

		/* SAMPLING */
		dl = ir_darkness(ir_filtered(0), IR_WHITE_LEFT, IR_BLACK_LEFT);
		dc = ir_darkness(ir_filtered(1), IR_WHITE_CENTER, IR_BLACK_CENTER);

		if (line_error(dl, dc, &err)) {
			/* STEERING */
			if (state == LN_LOST) {
				line_pd_init(&pd);	/* fresh derivative after a loss */
			}
			state = LN_STEERING;
			lost_ticks = 0;
			lost_ms = 0;
			last_err = err;
			m.steer = line_pd_step(&pd, err);
			m.flags = 0;
		} else {
			/* off the line: keep turning the way it was last seen */
			if (lost_ticks < LINE_LOST_TICKS) {
				lost_ticks++;
				m.steer = line_pd_step(&pd, last_err);
				m.flags = 0;
			} else {
				state = LN_LOST;
				lost_ms += LINE_TICK_MS;
				m.steer = (last_err >= 0) ? STEER_MAX / 2 : -STEER_MAX / 2;
				m.flags = STEER_F_LOST;
				if (lost_ms >= LINE_TIMEOUT_MS) {
					m.flags |= STEER_F_TIMEOUT;
				}
			}
		}
		s2_line_state = state;
		MBF_SEND(mbf_steer, m);		/* full buffer: drop, Mission is behind */
	}
}
