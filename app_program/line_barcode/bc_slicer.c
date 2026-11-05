/** @file bc_slicer.c
 *
 * @brief Peak-and-valley edge detector for the barcode sensor.
 *        See bc_slicer.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "bc_slicer.h"

#define SPAN_NOMINAL        (LB_BC_BLACK_NOMINAL - LB_BC_WHITE_NOMINAL)
#define PROMINENCE_COUNTS   ((SPAN_NOMINAL * LB_BC_PROMINENCE_PCT) / 100)

/* Private function prototypes. */
static bool step_toward_peak(bc_slicer_t * p_slicer,
                             int32_t sample,
                             bc_slicer_edge_t * p_edge);
static bool step_toward_valley(bc_slicer_t * p_slicer,
                               int32_t sample,
                               bc_slicer_edge_t * p_edge);
static uint32_t find_crossing_time(bc_slicer_t const * p_slicer,
                                   uint32_t from_no,
                                   int32_t level,
                                   bool b_rising);
static int32_t clamp_count(int32_t value);
static int32_t edge_level(bc_slicer_t const * p_slicer,
                          int32_t valley,
                          int32_t peak);
static void learn_levels(bc_slicer_t * p_slicer, int32_t valley, int32_t peak);

void
bc_slicer_init (bc_slicer_t * p_slicer)
{
    uint32_t idx = 0u;

    if (NULL != p_slicer)
    {
        for (idx = 0u; idx < LB_BC_HISTORY_SAMPLES; idx++)
        {
            p_slicer->value[idx]   = 0;
            p_slicer->time_us[idx] = 0u;
        }

        p_slicer->sample_no   = 0u;
        p_slicer->last_no     = 0u;
        p_slicer->last_val    = 0;
        p_slicer->cand_no     = 0u;
        p_slicer->cand_val    = 0;
        p_slicer->b_ref       = LB_BC_BLACK_NOMINAL;
        p_slicer->w_ref       = LB_BC_WHITE_NOMINAL;
        p_slicer->warmup_left = LB_BC_WARMUP_SAMPLES;
        p_slicer->b_up        = true;
        p_slicer->b_primed    = false;
    }
    else
    {
        /* NULL pointer: nothing to initialise. */
    }

    return;
}

bool
bc_slicer_step (bc_slicer_t * p_slicer,
                int32_t dark,
                uint32_t now_us,
                bc_slicer_edge_t * p_edge)
{
    bool     b_edge = false;
    int32_t  sample = clamp_count(dark);
    uint32_t slot   = 0u;

    if ((NULL != p_slicer) && (NULL != p_edge))
    {
        /* Store the sample first: the edge search may need it. */
        slot = p_slicer->sample_no % LB_BC_HISTORY_SAMPLES;

        /* Cast: sample was clamped to 0 to 4095 above, so it fits int16_t. */
        p_slicer->value[slot]   = (int16_t)sample;
        p_slicer->time_us[slot] = now_us;

        if ((!p_slicer->b_primed) || (p_slicer->warmup_left > 0))
        {
            /* Start-up: just follow the signal and report nothing. */
            if (p_slicer->warmup_left > 0)
            {
                p_slicer->warmup_left--;
            }
            else
            {
                /* The first sample primes the slicer. */
            }

            p_slicer->last_no   = p_slicer->sample_no;
            p_slicer->last_val  = sample;
            p_slicer->cand_no   = p_slicer->sample_no;
            p_slicer->cand_val  = sample;
            p_slicer->b_up      = true;
            p_slicer->b_primed  = true;
        }
        else if (p_slicer->b_up)
        {
            b_edge = step_toward_peak(p_slicer, sample, p_edge);
        }
        else
        {
            b_edge = step_toward_valley(p_slicer, sample, p_edge);
        }

        p_slicer->sample_no++;
    }
    else
    {
        /* NULL pointer: no edge can be reported. */
    }

    return (b_edge);
}

int32_t
bc_slicer_get_threshold (bc_slicer_t const * p_slicer)
{
    int32_t threshold = 0;

    if (NULL != p_slicer)
    {
        threshold = (p_slicer->last_val + p_slicer->cand_val) / 2;
    }
    else
    {
        /* NULL pointer: report 0. */
    }

    return (threshold);
}

/*!
 * @brief One sample while waiting for a rise (the last extreme was a valley).
 *
 * The candidate is the highest sample since the valley.  When it is at
 * least the prominence above the valley and the signal has turned around or
 * stopped, the rise is finished and its edge is reported.  If the signal
 * instead dips below the valley, the valley is moved down.
 */
static bool
step_toward_peak (bc_slicer_t * p_slicer,
                  int32_t sample,
                  bc_slicer_edge_t * p_edge)
{
    bool    b_edge    = false;
    bool    b_turned  = false;
    bool    b_settled = false;
    int32_t rise      = 0;

    if (sample > p_slicer->cand_val)
    {
        p_slicer->cand_val = sample;
        p_slicer->cand_no  = p_slicer->sample_no;
    }
    else
    {
        /* Not a new highest sample. */
    }

    rise      = p_slicer->cand_val - p_slicer->last_val;
    b_turned  = ((p_slicer->cand_val - sample) >= PROMINENCE_COUNTS);
    b_settled = ((p_slicer->sample_no - p_slicer->cand_no)
                 >= LB_BC_SETTLE_SAMPLES);

    if ((rise >= PROMINENCE_COUNTS) && (b_turned || b_settled))
    {
        p_edge->time_us = find_crossing_time(
            p_slicer,
            p_slicer->last_no,
            edge_level(p_slicer, p_slicer->last_val, p_slicer->cand_val),
            true);
        p_edge->b_dark = true;
        b_edge         = true;

        learn_levels(p_slicer, p_slicer->last_val, p_slicer->cand_val);

        /* The peak becomes the last extreme; now look for a fall. */
        p_slicer->last_no  = p_slicer->cand_no;
        p_slicer->last_val = p_slicer->cand_val;
        p_slicer->cand_no  = p_slicer->sample_no;
        p_slicer->cand_val = sample;
        p_slicer->b_up     = false;
    }
    else if (sample < p_slicer->last_val)
    {
        /* Lower than the valley and no real rise yet: the valley moves. */
        p_slicer->last_no  = p_slicer->sample_no;
        p_slicer->last_val = sample;
        p_slicer->cand_no  = p_slicer->sample_no;
        p_slicer->cand_val = sample;
    }
    else
    {
        /* Still between valley and candidate: keep waiting. */
    }

    return (b_edge);
}

/*!
 * @brief One sample while waiting for a fall (the last extreme was a peak).
 *
 * Mirror image of step_toward_peak().
 */
static bool
step_toward_valley (bc_slicer_t * p_slicer,
                    int32_t sample,
                    bc_slicer_edge_t * p_edge)
{
    bool    b_edge    = false;
    bool    b_turned  = false;
    bool    b_settled = false;
    int32_t fall      = 0;

    if (sample < p_slicer->cand_val)
    {
        p_slicer->cand_val = sample;
        p_slicer->cand_no  = p_slicer->sample_no;
    }
    else
    {
        /* Not a new lowest sample. */
    }

    fall      = p_slicer->last_val - p_slicer->cand_val;
    b_turned  = ((sample - p_slicer->cand_val) >= PROMINENCE_COUNTS);
    b_settled = ((p_slicer->sample_no - p_slicer->cand_no)
                 >= LB_BC_SETTLE_SAMPLES);

    if ((fall >= PROMINENCE_COUNTS) && (b_turned || b_settled))
    {
        p_edge->time_us = find_crossing_time(
            p_slicer,
            p_slicer->last_no,
            edge_level(p_slicer, p_slicer->cand_val, p_slicer->last_val),
            false);
        p_edge->b_dark = false;
        b_edge         = true;

        learn_levels(p_slicer, p_slicer->cand_val, p_slicer->last_val);

        /* The valley becomes the last extreme; now look for a rise. */
        p_slicer->last_no  = p_slicer->cand_no;
        p_slicer->last_val = p_slicer->cand_val;
        p_slicer->cand_no  = p_slicer->sample_no;
        p_slicer->cand_val = sample;
        p_slicer->b_up     = true;
    }
    else if (sample > p_slicer->last_val)
    {
        /* Higher than the peak and no real fall yet: the peak moves. */
        p_slicer->last_no  = p_slicer->sample_no;
        p_slicer->last_val = sample;
        p_slicer->cand_no  = p_slicer->sample_no;
        p_slicer->cand_val = sample;
    }
    else
    {
        /* Still between peak and candidate: keep waiting. */
    }

    return (b_edge);
}

/*!
 * @brief Time at which the stored signal crossed a level.
 *
 * Searches forward from sample from_no for the first sample that is at or
 * beyond the level, then interpolates linearly between it and the sample
 * before it.
 *
 * NOTE: if the move was longer than the history, the oldest stored sample
 * is used as the start, so the result is slightly late rather than wrong.
 *
 * @param[in] p_slicer  Slicer with the sample history.
 * @param[in] from_no   Sample number where the move started.
 * @param[in] level     Level to find, in counts.
 * @param[in] b_rising  true: find the first sample at or above the level;
 *                      false: at or below.
 *
 * @return Time in microseconds.
 */
static uint32_t
find_crossing_time (bc_slicer_t const * p_slicer,
                    uint32_t from_no,
                    int32_t level,
                    bool b_rising)
{
    uint32_t newest   = p_slicer->sample_no;
    uint32_t oldest   = 0u;
    uint32_t number   = from_no;
    uint32_t slot     = 0u;
    uint32_t prev     = 0u;
    int32_t  value    = 0;
    int32_t  prev_val = 0;
    uint32_t step_us  = 0u;
    uint32_t result   = 0u;
    bool     b_found  = false;

    if (newest >= (LB_BC_HISTORY_SAMPLES - 1u))
    {
        oldest = newest - (LB_BC_HISTORY_SAMPLES - 1u);
    }
    else
    {
        /* History not yet full: sample 0 is the oldest. */
    }

    if (number < oldest)
    {
        number = oldest;
    }
    else
    {
        /* The move started inside the history. */
    }

    result = p_slicer->time_us[number % LB_BC_HISTORY_SAMPLES];

    while ((number <= newest) && (!b_found))
    {
        slot  = number % LB_BC_HISTORY_SAMPLES;

        /* Cast: widening int16_t to int32_t, no loss. */
        value = (int32_t)p_slicer->value[slot];

        if (b_rising ? (value >= level) : (value <= level))
        {
            b_found = true;
            result  = p_slicer->time_us[slot];

            if (number > from_no)
            {
                prev     = (number - 1u) % LB_BC_HISTORY_SAMPLES;

                /* Cast: widening int16_t to int32_t, no loss. */
                prev_val = (int32_t)p_slicer->value[prev];

                if (value != prev_val)
                {
                    /* Linear interpolation inside this 1 ms step.  Unsigned
                     * subtraction keeps the step right across a timer wrap.
                     * The step is limited to 65535 us so the product below
                     * (value difference under 4096) fits in 32 bits; this
                     * keeps 64-bit division, which the Cortex-M0+ would do
                     * in software, out of the 1 kHz handler. */
                    step_us = p_slicer->time_us[slot]
                              - p_slicer->time_us[prev];
                    if (step_us > 65535u)
                    {
                        step_us = 65535u;
                    }
                    else
                    {
                        /* Normal case: the step is about 1000 us. */
                    }

                    /* Cast: step_us is at most 65535, so it fits int32_t. */
                    result  = p_slicer->time_us[prev]
                              + (uint32_t)(((level - prev_val)
                                            * (int32_t)step_us)
                                           / (value - prev_val));
                }
                else
                {
                    /* Flat step: keep the later sample's time. */
                }
            }
            else
            {
                /* Crossed at the very first sample: no earlier sample. */
            }
        }
        else
        {
            number++;
        }
    }

    return (result);
}

/*!
 * @brief Level at which an edge between a valley and a peak is timed.
 *
 * The ideal level is half way between the learned paper and ink levels,
 * because a bar edge is exactly there when the sensor spot is half on the
 * bar.  If this move did not reach that far (a blurred narrow bar), the
 * level is kept half a prominence inside the move so that it is still
 * crossed.
 *
 * @param[in] p_slicer  Slicer with the learned levels.
 * @param[in] valley    Lowest value of the move.
 * @param[in] peak      Highest value of the move; at least a prominence
 *                      above the valley.
 *
 * @return Level in dark-scale counts.
 */
static int32_t
edge_level (bc_slicer_t const * p_slicer, int32_t valley, int32_t peak)
{
    int32_t level = (p_slicer->w_ref + p_slicer->b_ref) / 2;
    int32_t low   = valley + (PROMINENCE_COUNTS / 2);
    int32_t high  = peak - (PROMINENCE_COUNTS / 2);

    if (level < low)
    {
        level = low;
    }
    else if (level > high)
    {
        level = high;
    }
    else
    {
        /* The ideal level lies inside the move. */
    }

    return (level);
}

/*!
 * @brief Update the learned paper and ink levels with a confirmed move.
 *
 * A more extreme value replaces the learned level at once; a less extreme
 * one pulls it by one eighth, so levels recover after a lighting change
 * without being dragged by every blurred narrow bar.
 */
static void
learn_levels (bc_slicer_t * p_slicer, int32_t valley, int32_t peak)
{
    if (peak >= p_slicer->b_ref)
    {
        p_slicer->b_ref = peak;
    }
    else
    {
        p_slicer->b_ref -= (p_slicer->b_ref - peak) / 2;
    }

    if (valley <= p_slicer->w_ref)
    {
        p_slicer->w_ref = valley;
    }
    else
    {
        p_slicer->w_ref += (valley - p_slicer->w_ref) / 2;
    }

    return;
}

/*!
 * @brief Limit a count to the 12-bit ADC range.
 */
static int32_t
clamp_count (int32_t value)
{
    int32_t result = value;

    if (value < 0)
    {
        result = 0;
    }
    else if (value > LB_ADC_MAX_COUNT)
    {
        result = LB_ADC_MAX_COUNT;
    }
    else
    {
        /* Already inside the range. */
    }

    return (result);
}

/*** end of file ***/
