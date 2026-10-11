/*
 *	motor_control_test.c
 *	On-target tests for the Motor control subsystem (Motor).  Built only with: make TEST=1
 *	Results go in testing/motor_control/TEST_LOG.md.
 *
 *	The motion test (S1-M01) makes the wheels turn, so it is OFF by default.
 *	Lift the car off the table, then build with -DS1_TEST_MOVE=1.
 */

#include "motor_control.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void motor_control_test(void)
{
	UW  t0;
	INT i, n;

	TEST_INIT();

	/* S1-U01: quadrature decoder, forward sequence 00 01 11 10 00 */
	n = qdec_step(0, 1) + qdec_step(1, 3) + qdec_step(3, 2) + qdec_step(2, 0);
	TEST_CHECK("S1-U01", n == 4, "quadrature: 4 forward edges = +4");

	/* S1-U02: reverse sequence and an impossible jump */
	n = qdec_step(0, 2) + qdec_step(2, 3) + qdec_step(3, 1) + qdec_step(1, 0);
	TEST_CHECK("S1-U02", n == -4 && qdec_step(0, 3) == 0,
		   "quadrature: 4 reverse edges = -4, a double jump counts 0");

	/* S1-U03: unit conversions agree with the constants */
	TEST_CHECK("S1-U03",
		   mm_to_counts(WHEEL_MM_PER_REV) == ENC_COUNTS_PER_REV &&
		   counts_to_mmps(ENC_COUNTS_PER_REV / 10) == WHEEL_MM_PER_REV * 10,
		   "one turn = ENC_COUNTS_PER_REV counts = WHEEL_MM_PER_REV mm; 10 turns/s = 10x mm/s");

	/* S1-U04: PID output follows the sign of the error and is clamped */
	{
		PID p;

		pid_init(&p, PID_KP, PID_KI, PID_KD, -DUTY_MAX, DUTY_MAX);
		TEST_CHECK("S1-U04", pid_step(&p, 100, 0) > 0, "PID: too slow -> push harder");
		pid_reset(&p);
		TEST_CHECK("S1-U05", pid_step(&p, 0, 100) < 0, "PID: too fast -> back off");
		pid_reset(&p);
		for (i = 0; i < 1000; i++) {
			n = pid_step(&p, 10000, 0);
		}
		TEST_CHECK("S1-U06", n == DUTY_MAX, "PID: output clamped to DUTY_MAX");
	}

	/* S1-U07: the Motor loop runs every 10 ms (NFR-14): ~100 in 1 s */
	t0 = s1_ticks;
	tk_dly_tsk(1000);
	n = (INT)(s1_ticks - t0);
	TEST_CHECK("S1-U07", n >= 95 && n <= 105, "motor loop runs every 10 ms");

	/* S1-U08: after boot the motors are STOPPED */
	TEST_CHECK("S1-U08", s1_state == MOT_STOPPED, "motor state is STOPPED after boot");

	/* S1-U09: the GPIO interrupt is installed: pins idle, count is stable */
	n = enc_cnt[0];
	tk_dly_tsk(100);
	TEST_CHECK("S1-U09", enc_cnt[0] == n, "encoder count is steady with the wheels still");

#if defined(S1_TEST_MOVE) && S1_TEST_MOVE
	/* S1-M01: START -> DRIVING -> STOP -> STOPPED */
	tk_set_flg(flg_evt, EVT_START);
	tk_dly_tsk(50);
	TEST_CHECK("S1-M01a", s1_state == MOT_DRIVING, "EVT_START -> DRIVING");
	tk_set_flg(flg_evt, EVT_STOP);
	tk_dly_tsk(50);
	TEST_CHECK("S1-M01b", s1_state == MOT_STOPPED, "EVT_STOP -> STOPPED");
#endif

	TEST_SUMMARY("motor_control");
}

#endif /* APP_TEST */
