/** @file line_ctrl.h
 *
 * @brief Line position estimate and PD steering for the two line sensors.
 *
 * @par
 * The black line runs between the centre and the right IR sensor, which
 * both normally see white.  A sensor that sees black means the line has
 * drifted under it, so the car steers toward that side to put the line back
 * between them (negative = left, positive = right).  When a black reading
 * ends after the sensor was fully over the line, the line may have left on
 * that side: the controller steers gently that way, and reports the line
 * lost if no black comes back (see LB_STRADDLE_FULL in lb_config.h).
 *
 * @par
 * What happens when the line is gone altogether is decided by line_recov.c,
 * not here.  This module only reports "not seen" and holds its last output.
 * Faded ink is handled by the calibration (ir_calib.c), not here.
 *
 * @par
 * Pure arithmetic: no hardware, no kernel calls, unit tested on a PC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef LINE_CTRL_H
#define LINE_CTRL_H

#include <stdint.h>
#include <stdbool.h>

/** State of the controller.  Treat as private; use the functions below. */
typedef struct
{
    int32_t  prev_err;          /* error of the previous tick */
    int32_t  err_filt;          /* last error, held while the line is lost */
    int32_t  steer;             /* last steering output, for slew and hold */
    bool     b_seen;            /* line visible */
    bool     b_primed;          /* prev_err and err_filt hold real values */
    int32_t  out_side;          /* -1 / +1: the line may have left past the
                                   centre / right sensor; 0: it is inside */
    int32_t  ep_side;           /* side of the darkest reading so far */
    int32_t  ep_peak;           /* darkest reading of this black episode */
    uint32_t seek_ticks;        /* ticks spent steering toward out_side */
    bool     b_episode;         /* a sensor is seeing black */
} line_ctrl_t;

/** Branch detector state. */
typedef struct
{
    uint32_t start_mm;         /* distance when the dark run began */
    bool     b_armed;          /* white seen since the last report */
    bool     b_in_run;         /* the sensor is in a dark run */
} line_junction_t;

/** Result of one control tick. */
typedef struct
{
    int32_t err;                /* position error, -500 to +500 */
    int32_t steer;              /* steering, -LB_STEER_MAX to +LB_STEER_MAX */
    bool    b_seen;             /* the line is visible */
} line_ctrl_out_t;

/*!
 * @brief Reset the controller (line not seen, steering straight).
 *
 * @param[out] p_ctrl  Controller to initialise; must not be NULL.
 */
void line_ctrl_init(line_ctrl_t * p_ctrl);

/*!
 * @brief Forget the derivative history after the line was lost and found.
 *
 * The next tick starts with a zero derivative and the slew limiter starts
 * from the steering the car was really given, so the wheels are not jerked.
 *
 * @param[in,out] p_ctrl      Controller; must not be NULL.
 * @param[in]     last_steer  Steering that was last sent to Mission.
 */
void line_ctrl_restart(line_ctrl_t * p_ctrl, int32_t last_steer);

/*!
 * @brief Run one control tick (every LB_LINE_TICK_MS).
 *
 * @param[in,out] p_ctrl         Controller; must not be NULL.
 * @param[in]     dark_centre    Centre sensor darkness, 0 to LB_DARK_MAX.
 * @param[in]     dark_right     Right sensor darkness, 0 to LB_DARK_MAX.
 * @param[out]    p_out          Result; must not be NULL.
 */
void line_ctrl_update(line_ctrl_t * p_ctrl,
                      int32_t dark_centre,
                      int32_t dark_right,
                      line_ctrl_out_t * p_out);

/*!
 * @brief Reset the branch detector: it needs white before it can report.
 *
 * @param[out] p_jct  Detector state.
 */
void line_junction_init(line_junction_t * p_jct);

/*!
 * @brief One detector step.
 *
 * @param[in,out] p_jct       Detector state.
 * @param[in]     dark_centre Centre sensor darkness, 0 to LB_DARK_MAX.
 * @param[in]     dark_right  Right sensor darkness, 0 to LB_DARK_MAX.
 * @param[in]     dist_mm     Distance driven so far.
 *
 * @return true once for each branch.
 */
bool line_junction_step(line_junction_t * p_jct,
                        int32_t dark_centre,
                        int32_t dark_right,
                        uint32_t dist_mm);

#endif /* LINE_CTRL_H */
/*** end of file ***/
