/** @file line_recov.c
 *
 * @brief Recovery when the line fades, vanishes, or the car veers off it.
 *        See line_recov.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "line_recov.h"

/* The sweep leg counter stops here so that it can never overflow. */
#define LEG_INDEX_CAP           100

#define SIDE_LEFT               (-1)
#define SIDE_RIGHT              1

/* Private function prototypes. */
static void step_track(line_recov_t * p_recov,
                       line_recov_in_t const * p_in,
                       line_recov_out_t * p_out);
static void step_search(line_recov_t * p_recov,
                        line_recov_in_t const * p_in,
                        line_recov_out_t * p_out);
static void enter_coast(line_recov_t * p_recov);
static void advance_search(line_recov_t * p_recov, int32_t dt_ms);
static int32_t coast_steer(line_recov_t const * p_recov);
static int32_t sweep_steer(line_recov_t const * p_recov);
static int32_t choose_side(line_recov_t const * p_recov);
static int32_t abs_i32(int32_t value);
static int32_t clamp_i32(int32_t value, int32_t low, int32_t high);

void
line_recov_init (line_recov_t * p_recov)
{
    if (NULL != p_recov)
    {
        p_recov->mode       = LINE_RECOV_TRACK;
        p_recov->miss_ticks = 0;
        p_recov->seen_ticks = 0;
        p_recov->err_mem    = 0;
        p_recov->steer_mem  = 0;
        p_recov->steer_out  = 0;
        p_recov->side       = LB_SIDE_DEFAULT;
        p_recov->coast_ms   = LB_COAST_FADE_MS;
        p_recov->mode_ms    = 0;
        p_recov->leg_index  = 0;
        p_recov->leg_ms     = 0;
        p_recov->lost_ms    = 0;
    }
    else
    {
        /* NULL pointer: nothing to initialise. */
    }

    return;
}

void
line_recov_step (line_recov_t * p_recov,
                 line_recov_in_t const * p_in,
                 line_recov_out_t * p_out)
{
    if ((NULL != p_recov) && (NULL != p_in) && (NULL != p_out))
    {
        p_out->flags     = 0u;

        if (LINE_RECOV_TRACK == p_recov->mode)
        {
            step_track(p_recov, p_in, p_out);
        }
        else
        {
            step_search(p_recov, p_in, p_out);
        }

        p_out->mode       = p_recov->mode;
        p_recov->steer_out = p_out->steer;
    }
    else
    {
        /* NULL pointer: no output can be produced. */
    }

    return;
}

/*!
 * @brief One tick in mode TRACK: pass the PD steering through, or bridge a
 *        short dropout, or give up and start to coast.
 */
static void
step_track (line_recov_t * p_recov,
            line_recov_in_t const * p_in,
            line_recov_out_t * p_out)
{
    if (p_in->b_seen)
    {
        p_recov->miss_ticks = 0;
        p_recov->lost_ms    = 0;

        /* Keep slow averages of the error and the steering.  They describe
         * where the car was heading BEFORE the line started to disappear,
         * which is better than the last, noisy, faded reading. */
        p_recov->err_mem   += (p_in->err - p_recov->err_mem) / LB_STEER_MEM_DIV;
        p_recov->steer_mem += (p_in->steer - p_recov->steer_mem)
                              / LB_STEER_MEM_DIV;
        p_out->steer = p_in->steer;
    }
    else
    {
        p_recov->miss_ticks++;

        if (p_recov->miss_ticks < LB_LOST_MISS_TICKS)
        {
            /* BRIDGE: hold the last steering for one missing tick. */
            p_out->steer = p_recov->steer_out;
        }
        else
        {
            enter_coast(p_recov);
            p_out->steer = coast_steer(p_recov);
            p_out->flags = LINE_RECOV_F_LOST;
        }
    }

    return;
}

/*!
 * @brief One tick in mode COAST or SWEEP: look for the line, otherwise
 *        carry on with the plan.
 */
static void
step_search (line_recov_t * p_recov,
             line_recov_in_t const * p_in,
             line_recov_out_t * p_out)
{
    int32_t dt_ms = clamp_i32(p_in->dt_ms, 0, LB_DT_MAX_MS);

    if (p_in->b_seen)
    {
        p_recov->seen_ticks++;
    }
    else
    {
        p_recov->seen_ticks = 0;
    }

    if (p_recov->seen_ticks >= LB_REACQ_TICKS)
    {
        /* REACQUIRE: the line has been steady for long enough. */
        p_recov->mode       = LINE_RECOV_TRACK;
        p_recov->miss_ticks = 0;
        p_recov->seen_ticks = 0;
        p_recov->lost_ms    = 0;
        p_out->steer        = p_in->steer;
    }
    else
    {
        advance_search(p_recov, dt_ms);

        if (LINE_RECOV_COAST == p_recov->mode)
        {
            p_out->steer = coast_steer(p_recov);
        }
        else
        {
            p_out->steer = sweep_steer(p_recov);
        }

        p_out->flags = LINE_RECOV_F_LOST;

        /* Count the time the line is missing, but only while the car is
         * supposed to be following a line. */
        if (p_in->b_timeout_active)
        {
            p_recov->lost_ms += dt_ms;
        }
        else
        {
            p_recov->lost_ms = 0;
        }

        if (p_recov->lost_ms >= LB_TIMEOUT_MS)
        {
            p_out->flags |= LINE_RECOV_F_TIMEOUT;
        }
        else
        {
            /* Not long enough to give up. */
        }
    }

    return;
}

/*!
 * @brief Declare the line lost: pick the search side and the coast length.
 *
 * A small error at the moment of loss means the line faded under a car that
 * is still on course, so the coast is long.  A large error means the car
 * itself left the line, so the search must begin almost at once.
 */
static void
enter_coast (line_recov_t * p_recov)
{
    p_recov->side = choose_side(p_recov);

    if (abs_i32(p_recov->err_mem) >= LB_VEER_ERR_MIN)
    {
        p_recov->coast_ms = LB_COAST_VEER_MS;
    }
    else
    {
        p_recov->coast_ms = LB_COAST_FADE_MS;
    }

    p_recov->mode       = LINE_RECOV_COAST;
    p_recov->seen_ticks = 0;
    p_recov->mode_ms    = 0;
    p_recov->leg_index  = 0;
    p_recov->leg_ms     = 0;
    p_recov->lost_ms    = 0;

    return;
}

/*!
 * @brief Move the timers on and switch from COAST to SWEEP or to the next
 *        sweep leg when the time is up.
 */
static void
advance_search (line_recov_t * p_recov, int32_t dt_ms)
{
    int32_t leg_limit_ms = LB_SWEEP_FAR_MS;

    p_recov->mode_ms += dt_ms;

    if (LINE_RECOV_COAST == p_recov->mode)
    {
        if (p_recov->mode_ms >= p_recov->coast_ms)
        {
            p_recov->mode      = LINE_RECOV_SWEEP;
            p_recov->mode_ms   = 0;
            p_recov->leg_index = 0;
            p_recov->leg_ms    = 0;
        }
        else
        {
            /* Still coasting. */
        }
    }
    else
    {
        p_recov->leg_ms += dt_ms;

        if (0 == p_recov->leg_index)
        {
            leg_limit_ms = LB_SWEEP_NEAR_MS;
        }
        else
        {
            /* Later legs use the longer far-side time. */
        }

        if (p_recov->leg_ms >= leg_limit_ms)
        {
            p_recov->leg_ms = 0;

            if (p_recov->leg_index < LEG_INDEX_CAP)
            {
                p_recov->leg_index++;
            }
            else
            {
                /* Keep alternating without counting further. */
            }
        }
        else
        {
            /* Leg not finished. */
        }
    }

    return;
}

/*!
 * @brief Steering while coasting: keep the remembered steering.
 *
 * Holding the steering keeps the car on the curve it was driving, so a line
 * that fades inside a bend is still followed around it.  On a straight the
 * remembered steering is close to zero, so the car simply keeps straight.
 * Over the last quarter of the coast the steering fades to zero, so there is
 * no jump when the search starts.
 */
static int32_t
coast_steer (line_recov_t const * p_recov)
{
    int32_t remaining_ms = p_recov->coast_ms - p_recov->mode_ms;
    int32_t fade_ms      = p_recov->coast_ms / 4;
    int32_t steer        = 0;

    if (remaining_ms > fade_ms)
    {
        steer = p_recov->steer_mem;
    }
    else if ((remaining_ms > 0) && (fade_ms > 0))
    {
        steer = (p_recov->steer_mem * remaining_ms) / fade_ms;
    }
    else
    {
        /* Coast over: straight ahead. */
    }

    return (steer);
}

/*!
 * @brief Steering while sweeping.
 *
 * Leg 0 steers toward the side where the line was last seen, leg 1 toward
 * the other side, and so on.  Inside a leg the amplitude ramps from a
 * minimum to a maximum; every leg is a little stronger than the one before.
 */
static int32_t
sweep_steer (line_recov_t const * p_recov)
{
    int32_t grow     = LB_SWEEP_STEER_GROW * p_recov->leg_index;
    int32_t amp_min  = clamp_i32(LB_SWEEP_STEER_MIN + grow,
                                 0, LB_SWEEP_STEER_CEIL);
    int32_t amp_max  = clamp_i32(LB_SWEEP_STEER_MAX + grow,
                                 0, LB_SWEEP_STEER_CEIL);
    int32_t ramp_ms  = clamp_i32(p_recov->leg_ms, 0, LB_SWEEP_RAMP_MS);
    int32_t amp      = amp_min
                       + (((amp_max - amp_min) * ramp_ms) / LB_SWEEP_RAMP_MS);
    int32_t direction = p_recov->side;

    if (1 == (p_recov->leg_index % 2))
    {
        direction = -p_recov->side;
    }
    else
    {
        /* Even legs go to the side where the line was last seen. */
    }

    /* The remembered steering is not added: after a veer it is large and
     * would keep both legs of the sweep turning the same way. */
    return (clamp_i32(direction * amp, -LB_STEER_MAX, LB_STEER_MAX));
}

/*!
 * @brief Which side was the line on when it vanished?
 *
 * The error says it: a negative error means the line was moving toward the
 * centre sensor.  If the error is too small to tell, the smoothed steering
 * decides, and if that is also tiny the default side is used.
 */
static int32_t
choose_side (line_recov_t const * p_recov)
{
    int32_t side = LB_SIDE_DEFAULT;

    if (abs_i32(p_recov->err_mem) >= LB_SIDE_ERR_MIN)
    {
        side = (p_recov->err_mem < 0) ? SIDE_LEFT : SIDE_RIGHT;
    }
    else if (abs_i32(p_recov->steer_mem) >= LB_SIDE_STEER_MIN)
    {
        side = (p_recov->steer_mem < 0) ? SIDE_LEFT : SIDE_RIGHT;
    }
    else
    {
        /* No information: keep the default side. */
    }

    return (side);
}

/*!
 * @brief Absolute value (the argument is never INT32_MIN here).
 */
static int32_t
abs_i32 (int32_t value)
{
    return ((value < 0) ? -value : value);
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
