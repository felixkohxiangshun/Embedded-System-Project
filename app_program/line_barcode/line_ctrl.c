/** @file line_ctrl.c
 *
 * @brief Line position estimate and PD steering.  See line_ctrl.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "line_ctrl.h"

/* Private function prototypes. */
static int32_t straddle_error(line_ctrl_t * p_ctrl,
                              int32_t dark_centre,
                              int32_t dark_right);
static int32_t compute_steer(line_ctrl_t * p_ctrl, int32_t err);
static int32_t clamp_i32(int32_t value, int32_t low, int32_t high);

void
line_ctrl_init (line_ctrl_t * p_ctrl)
{
    if (NULL != p_ctrl)
    {
        p_ctrl->prev_err        = 0;
        p_ctrl->err_filt        = 0;
        p_ctrl->steer           = 0;
        p_ctrl->b_seen          = false;
        p_ctrl->b_primed        = false;
        p_ctrl->out_side        = 0;
        p_ctrl->ep_side         = 0;
        p_ctrl->ep_peak         = 0;
        p_ctrl->seek_ticks      = 0u;
        p_ctrl->b_episode       = false;
    }
    else
    {
        /* NULL pointer: nothing to initialise. */
    }

    return;
}

void
line_ctrl_restart (line_ctrl_t * p_ctrl, int32_t last_steer)
{
    if (NULL != p_ctrl)
    {
        p_ctrl->b_primed   = false;
        p_ctrl->steer      = clamp_i32(last_steer, -LB_STEER_MAX, LB_STEER_MAX);
        /* The memory of where the line went is from before the stop or
         * turn: forget it. */
        p_ctrl->out_side   = 0;
        p_ctrl->b_episode  = false;
        p_ctrl->seek_ticks = 0u;
    }
    else
    {
        /* NULL pointer: nothing to restart. */
    }

    return;
}

void
line_ctrl_update (line_ctrl_t * p_ctrl,
                  int32_t dark_centre,
                  int32_t dark_right,
                  line_ctrl_out_t * p_out)
{
    int32_t err = 0;

    if ((NULL != p_ctrl) && (NULL != p_out))
    {
        /* Both white is "on the line"; the steering comes from the
         * difference, or from the side the line was last seen. */
        err = straddle_error(p_ctrl, dark_centre, dark_right);

        if (p_ctrl->b_seen)
        {
            p_ctrl->steer = compute_steer(p_ctrl, err);
        }
        else
        {
            /* Lost: line_recov.c takes over with the remembered error. */
            err = p_ctrl->err_filt;
        }

        p_out->err        = err;
        p_out->steer      = p_ctrl->steer;
        p_out->b_seen     = p_ctrl->b_seen;
    }
    else
    {
        /* NULL pointer: no result can be produced. */
    }

    return;
}

/*!
 * @brief Position error, and whether the line is still there.
 *
 * -500 (centre sensor black, steer left) to +500 (right sensor black, steer
 * right), 0 with both white.  Remembers on which side the line may have
 * slid out (see LB_STRADDLE_FULL in lb_config.h) and steers back toward it;
 * sets b_seen to false once that seeking has found nothing for SEEK_MS.
 */
static int32_t
straddle_error (line_ctrl_t * p_ctrl, int32_t dark_centre, int32_t dark_right)
{
    int32_t err    = 0;
    int32_t darker = 0;
    bool    b_both = (dark_centre >= LB_JUNCTION_DARK_MIN)
                     && (dark_right >= LB_JUNCTION_DARK_MIN);

    /* Rescale so that LB_STRADDLE_SAT reads as fully black: a faded line
     * then steers and is tracked like a fresh one. */
    dark_centre = (clamp_i32(dark_centre, 0, LB_STRADDLE_SAT) * LB_DARK_MAX)
                  / LB_STRADDLE_SAT;
    dark_right  = (clamp_i32(dark_right, 0, LB_STRADDLE_SAT) * LB_DARK_MAX)
                  / LB_STRADDLE_SAT;
    darker      = (dark_centre > dark_right) ? dark_centre : dark_right;

    p_ctrl->b_seen = true;

    if (darker >= LB_STRADDLE_BLACK_MIN)
    {
        /* Black under a sensor: steer on the difference. */
        if (!p_ctrl->b_episode)
        {
            p_ctrl->b_episode = true;
            p_ctrl->ep_peak   = 0;
            p_ctrl->ep_side   = 0;
        }
        else
        {
            /* The episode goes on. */
        }

        if (darker >= p_ctrl->ep_peak)
        {
            p_ctrl->ep_peak = darker;

            if (b_both)
            {
                p_ctrl->ep_side = 0;    /* both dark: a junction */
            }
            else
            {
                p_ctrl->ep_side = (dark_right > dark_centre) ? 1 : -1;
            }
        }
        else
        {
            /* Not darker than before: keep the side of the peak. */
        }

        p_ctrl->seek_ticks = 0u;
        err = (dark_right - dark_centre) / 2;
    }
    else
    {
        if (p_ctrl->b_episode)
        {
            /* The black reading just ended: inside or out past it? */
            p_ctrl->b_episode = false;
            p_ctrl->out_side  = (p_ctrl->ep_peak >= LB_STRADDLE_FULL)
                                ? p_ctrl->ep_side : 0;
        }
        else
        {
            /* Still white. */
        }

        if (0 != p_ctrl->out_side)
        {
            err = p_ctrl->out_side * LB_STRADDLE_SEEK_ERR;

            /* Casts: both are small positive constants. */
            if ((p_ctrl->seek_ticks * (uint32_t)LB_LINE_TICK_MS)
                < (uint32_t)LB_STRADDLE_SEEK_MS)
            {
                p_ctrl->seek_ticks++;
            }
            else
            {
                p_ctrl->b_seen = false;     /* sought long enough: lost */
            }
        }
        else
        {
            /* Line between the sensors: straight on. */
        }
    }

    return (err);
}

/*!
 * @brief PD controller with output limit and slew limit.
 */
static int32_t
compute_steer (line_ctrl_t * p_ctrl, int32_t err)
{
    int32_t derivative = 0;
    int32_t target     = 0;
    int32_t step       = 0;

    p_ctrl->err_filt = err;     /* kept for line_recov.c if the line is lost */

    if (p_ctrl->b_primed)
    {
        derivative = err - p_ctrl->prev_err;
    }
    else
    {
        /* First tick after start or after the line was found again: there
         * is no history, so the derivative must not kick. */
        derivative = 0;
        p_ctrl->b_primed = true;
    }

    p_ctrl->prev_err = err;

    target = ((LB_KP * err) + (LB_KD * derivative)) / LB_GAIN_SCALE;
    target = clamp_i32(target, -LB_STEER_MAX, LB_STEER_MAX);

    /* Slew limit: the steering may change by LB_STEER_SLEW_MAX per tick. */
    step = clamp_i32(target - p_ctrl->steer,
                     -LB_STEER_SLEW_MAX,
                     LB_STEER_SLEW_MAX);

    return (clamp_i32(p_ctrl->steer + step, -LB_STEER_MAX, LB_STEER_MAX));
}

/*!
 * @brief Limit a value to the range low to high.
 */
static int32_t
clamp_i32 (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* Already inside the range. */
    }

    return (result);
}

void
line_junction_init (line_junction_t * p_jct)
{
    if (NULL != p_jct)
    {
        p_jct->start_mm = 0u;
        p_jct->b_armed  = false;
        p_jct->b_in_run = false;
    }
    else
    {
        /* NULL pointer: nothing to reset. */
    }

    return;
}

bool
line_junction_step (line_junction_t * p_jct,
                    int32_t dark_centre,
                    int32_t dark_right,
                    uint32_t dist_mm)
{
    bool b_found = false;

    if (NULL == p_jct)
    {
        /* NULL pointer: nothing to report. */
    }
    else if ((dark_centre < LB_BRANCH_DARK_MIN)
             || (dark_right < LB_BRANCH_DARK_MIN))
    {
        p_jct->b_armed  = true;
        p_jct->b_in_run = false;
    }
    else if (p_jct->b_armed)
    {
        if (!p_jct->b_in_run)
        {
            p_jct->b_in_run = true;
            p_jct->start_mm = dist_mm;
        }
        else
        {
            /* The dark run goes on. */
        }

        /* Unsigned subtraction stays correct if the distance wraps. */
        if ((dist_mm - p_jct->start_mm) >= (uint32_t)LB_BRANCH_RUN_MM)
        {
            p_jct->b_armed  = false;
            p_jct->b_in_run = false;
            b_found = true;
        }
        else
        {
            /* Too short so far: could still be a speck. */
        }
    }
    else
    {
        /* Already reported: wait for white. */
    }

    return (b_found);
}

/*** end of file ***/
