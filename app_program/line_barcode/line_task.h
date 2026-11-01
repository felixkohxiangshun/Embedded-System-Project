/** @file line_task.h
 *
 * @brief Line tracking task: every 10 ms turns the two line sensors into a
 *        steering value for Mission.
 *
 * @par
 * One tick (design review statechart 6.1: SAMPLING, STEERING, LINE_LOST):
 *  1. read the filtered left and centre sensors and normalise them with the
 *     calibration (ir_calib),
 *  2. line_ctrl turns them into a position error and a PD steering,
 *  3. line_recov watches for a faded, lost or veered-off line and replaces
 *     the steering with a coast or a search when needed,
 *  4. the result is posted to Mission as MSG_STEER (IF-01).
 *
 * @par
 * The task never commands the motors.  Mission forwards the steering to the
 * Motor task only in the states where the car follows the line.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef LINE_TASK_H
#define LINE_TASK_H

#include "../app.h"

/*!
 * @brief Cyclic handler (every LB_LINE_TICK_MS): wakes line_task.
 *
 * @param[in] p_exinf  Extended information of the cyclic handler; unused.
 */
void line_wakeup_isr(void * p_exinf);

/*!
 * @brief The line tracking task.  Never returns.
 *
 * @param[in] stacd    Start code given by the kernel; unused.
 * @param[in] p_exinf  Extended information; unused.
 */
void line_task(INT stacd, void * p_exinf);

#endif /* LINE_TASK_H */
/*** end of file ***/
