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

/* The position of the line is measured from the left sensor (-1000) to the
 * centre sensor (0), so the sensor spacing is 1000 position units. */
#define POSITION_LEFT_SENSOR    (-1000)

/* Largest count of junction ticks that is remembered (avoids overflow). */
#define JUNCTION_TICKS_CAP      1000u

#define PERCENT_DIVISOR         100

/* Private function prototypes. */
static void update_flags(line_ctrl_t * p_ctrl,
                         int32_t dark_left,
                         int32_t dark_centre,
                         int32_t mass);
static int32_t estimate_error(line_ctrl_t const * p_ctrl,
                              int32_t dark_left,
                              int32_t mass);
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
        p_ctrl->mass_avg        = LB_MASS_NOMINAL;
        p_ctrl->junction_ticks  = 0u;
        p_ctrl->b_seen          = false;
        p_ctrl->b_faded         = false;
        p_ctrl->b_primed        = false;
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
        p_ctrl->b_primed = false;
        p_ctrl->steer    = clamp_i32(last_steer, -LB_STEER_MAX, LB_STEER_MAX);
    }
    else
    {
        /* NULL pointer: nothing to restart. */
    }

    return;
}

void
line_ctrl_update (line_ctrl_t * p_ctrl,
                  int32_t dark_left,
                  int32_t dark_centre,
                  line_ctrl_out_t * p_out)
{
    int32_t mass = 0;
    int32_t err  = 0;

    if ((NULL != p_ctrl) && (NULL != p_out))
    {
        mass = dark_left + dark_centre;

        update_flags(p_ctrl, dark_left, dark_centre, mass);

        if (p_ctrl->b_seen)
        {
            err = estimate_error(p_ctrl, dark_left, mass);
            p_ctrl->steer = compute_steer(p_ctrl, err);
        }
        else
        {
            /* Line not visible: keep the last error and the last steering
             * so line_recov.c can decide what to do. */
            err = p_ctrl->err_filt;
        }

        p_out->err        = err;
        p_out->mass       = mass;
        p_out->steer      = p_ctrl->steer;
        p_out->b_seen     = p_ctrl->b_seen;
        p_out->b_faded    = p_ctrl->b_faded;
        p_out->b_junction = (LB_JUNCTION_TICKS <= p_ctrl->junction_ticks)
                            && (0 != LB_JUNCTION_ENABLE);
    }
    else
    {
        /* NULL pointer: no result can be produced. */
    }

    return;
}

/*!
 * @brief Update the "seen", "faded" and "junction" decisions.
 *
 * All three use hysteresis (or a count) so that a value hovering at a limit
 * does not make the decision flicker.
 */
static void
update_flags (line_ctrl_t * p_ctrl,
              int32_t dark_left,
              int32_t dark_centre,
              int32_t mass)
{
    /* Seen: switch on at SEEN_ON, off only below SEEN_OFF. */
    if (p_ctrl->b_seen)
    {
        p_ctrl->b_seen = (mass >= LB_MASS_SEEN_OFF);
    }
    else
    {
        p_ctrl->b_seen = (mass >= LB_MASS_SEEN_ON);
    }

    /* Faded: judged on the averaged mass so one dark speck or one white
     * gap cannot switch the gains. */
    p_ctrl->mass_avg += (mass - p_ctrl->mass_avg) / LB_MASS_AVG_DIV;

    if (p_ctrl->b_faded)
    {
        p_ctrl->b_faded = (p_ctrl->mass_avg < LB_MASS_FADED_OFF);
    }
    else
    {
        p_ctrl->b_faded = (p_ctrl->mass_avg < LB_MASS_FADED_ON);
    }

    /* Junction: both sensors dark for several ticks in a row. */
    if ((dark_left >= LB_JUNCTION_DARK_MIN)
        && (dark_centre >= LB_JUNCTION_DARK_MIN))
    {
        if (p_ctrl->junction_ticks < JUNCTION_TICKS_CAP)
        {
            p_ctrl->junction_ticks++;
        }
        else
        {
            /* Counter saturated. */
        }
    }
    else
    {
        p_ctrl->junction_ticks = 0u;
    }

    return;
}

/*!
 * @brief Line position error, -500 (under left) to +500 (under centre).
 *
 * Position = -1000 * left / (left + centre): the share of the darkness that
 * the left sensor sees.  The mass cancels, so a faded line gives the same
 * position as a fresh one.  The caller guarantees mass > 0 (the line is
 * seen, so the mass is at least LB_MASS_SEEN_OFF).
 */
static int32_t
estimate_error (line_ctrl_t const * p_ctrl, int32_t dark_left, int32_t mass)
{
    int32_t position = (POSITION_LEFT_SENSOR * dark_left) / mass;
    int32_t err      = position - LB_LINE_TARGET;

    /* On a junction both sensors are dark and the position means nothing:
     * pretend the line is dead ahead so the car drives straight over it. */
    if (LB_JUNCTION_TICKS <= p_ctrl->junction_ticks)
    {
        if (0 != LB_JUNCTION_ENABLE)
        {
            err = 0;
        }
        else
        {
            /* Junction handling switched off in lb_config.h. */
        }
    }
    else
    {
        /* Plain line: keep the measured error. */
    }

    return (err);
}

/*!
 * @brief PD controller with FADED-mode gains, output limit and slew limit.
 */
static int32_t
compute_steer (line_ctrl_t * p_ctrl, int32_t err)
{
    int32_t gain_p   = LB_KP;
    int32_t gain_d   = LB_KD;
    int32_t limit    = LB_STEER_MAX;
    int32_t err_used = err;
    int32_t derivative = 0;
    int32_t target   = 0;
    int32_t step     = 0;

    if (p_ctrl->b_faded)
    {
        /* Weak line: noisier estimate, so smooth it and steer gently. */
        p_ctrl->err_filt += (err - p_ctrl->err_filt) / LB_FADED_ERR_IIR_DIV;
        err_used = p_ctrl->err_filt;
        gain_p   = (LB_KP * LB_FADED_KP_PCT) / PERCENT_DIVISOR;
        gain_d   = (LB_KD * LB_FADED_KD_PCT) / PERCENT_DIVISOR;
        limit    = LB_FADED_STEER_MAX;
    }
    else
    {
        p_ctrl->err_filt = err;
    }

    if (p_ctrl->b_primed)
    {
        derivative = err_used - p_ctrl->prev_err;
    }
    else
    {
        /* First tick after start or after the line was found again: there
         * is no history, so the derivative must not kick. */
        derivative = 0;
        p_ctrl->b_primed = true;
    }

    p_ctrl->prev_err = err_used;

    target = ((gain_p * err_used) + (gain_d * derivative)) / LB_GAIN_SCALE;
    target = clamp_i32(target, -limit, limit);

    /* Slew limit: the steering may change by LB_STEER_SLEW_MAX per tick. */
    step = clamp_i32(target - p_ctrl->steer,
                     -LB_STEER_SLEW_MAX,
                     LB_STEER_SLEW_MAX);

    return (clamp_i32(p_ctrl->steer + step, -limit, limit));
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

/*** end of file ***/
