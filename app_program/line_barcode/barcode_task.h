/** @file barcode_task.h
 *
 * @brief Barcode task: collects the bar/space edges sent by the sampler,
 *        decodes the Code 39 "*X*" message and reports it to Mission.
 *
 * @par
 * States (design review statechart 6.1):
 *  WAITING    blocked on mbf_edge, using no CPU, until a dark bar begins;
 *  CAPTURING  collecting edges until a whole code is in, the buffer is full
 *             or the line goes quiet;
 *  DECODING   measuring the widths and reading the character;
 *  POST       sending BC_OK and the command to Mission;
 *  FAILED     sending BC_FAIL (bad widths, bad frame, unknown character).
 *
 * @par
 * BC_FIRST is sent when the second dark bar of a capture starts within
 * LB_BC_FIRST_WINDOW_MM of the first (a junction line is one bar).  Mission
 * may answer with EVT_BC_HOLD (drop the capture, ignore edges while the
 * car backs up) and later EVT_BC_ARM (start a fresh capture for the slow
 * pass).  With no answer the capture simply runs on.
 * BC_SEEN is sent as soon as the capture holds a few edges, so Mission can
 * enter BARCODE_SCAN.  Every BC_SEEN is followed by exactly one BC_OK or
 * BC_FAIL, so Mission never waits forever.  The car keeps following the line
 * while this task works; it only reports and never drives the motors.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef BARCODE_TASK_H
#define BARCODE_TASK_H

#include "../app.h"

/*!
 * @brief The barcode task.  Never returns.
 *
 * @param[in] stacd    Start code given by the kernel; unused.
 * @param[in] p_exinf  Extended information; unused.
 */
void barcode_task(INT stacd, void * p_exinf);

#endif /* BARCODE_TASK_H */
/*** end of file ***/
