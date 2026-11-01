/** @file ir_calib.c
 *
 * @brief Raw ADC count to normalised darkness.  See ir_calib.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "ir_calib.h"

int32_t
ir_calib_raw_to_dark (uint16_t raw_adc)
{
    /* Cast: the ADC result is 12 bits (0 to 4095), so it always fits
     * int32_t. */
    int32_t count = (int32_t)raw_adc;

    /* Defensive clamp: a corrupt register value must not leave the 12-bit
     * range, or the polarity flip below would go negative. */
    if (count > LB_ADC_MAX_COUNT)
    {
        count = LB_ADC_MAX_COUNT;
    }
    else
    {
        /* Value already in range. */
    }

#if (0 == LB_IR_DARK_IS_HIGH)
    /* NOTE: this module outputs a LOWER voltage over black, so flip the
     * scale to make "higher is darker" true for the rest of the code. */
    count = LB_ADC_MAX_COUNT - count;
#endif

    return (count);
}

int32_t
ir_calib_darkness (ir_calib_t const * p_cal, int32_t dark_count)
{
    int32_t result = 0;
    int32_t span   = 0;

    if (NULL != p_cal)
    {
        span = p_cal->black_level - p_cal->white_level;
    }
    else
    {
        /* No calibration: span stays 0 and the sensor reads as white. */
    }

    if (span >= LB_CAL_SPAN_MIN)
    {
        /* Largest intermediate value is 4095 * 1000, which fits int32_t. */
        result = ((dark_count - p_cal->white_level) * LB_DARK_MAX) / span;

        if (result < 0)
        {
            result = 0;
        }
        else if (result > LB_DARK_MAX)
        {
            result = LB_DARK_MAX;
        }
        else
        {
            /* Inside 0 .. LB_DARK_MAX: keep as is. */
        }
    }
    else
    {
        /* Invalid calibration: report white so the car stops safely
         * (the line is never seen) instead of steering on nonsense. */
    }

    return (result);
}

bool
ir_calib_is_valid (ir_calib_t const * p_cal)
{
    bool b_valid = false;

    if (NULL != p_cal)
    {
        b_valid = ((p_cal->black_level - p_cal->white_level)
                   >= LB_CAL_SPAN_MIN);
    }
    else
    {
        /* NULL pointer: not valid. */
    }

    return (b_valid);
}

void
ir_calib_probe_init (ir_calib_probe_t * p_probe)
{
    if (NULL != p_probe)
    {
        p_probe->sum         = 0;
        p_probe->min_count   = LB_ADC_MAX_COUNT;
        p_probe->max_count   = 0;
        p_probe->num_samples = 0u;
    }
    else
    {
        /* NULL pointer: nothing to clear. */
    }

    return;
}

void
ir_calib_probe_push (ir_calib_probe_t * p_probe, int32_t dark_count)
{
    if (NULL != p_probe)
    {
        /* NOTE: a probe is fed for a few seconds at 100 samples per second,
         * so the sum stays far below the int32_t limit (300 * 4095). */
        p_probe->sum += dark_count;

        if (dark_count < p_probe->min_count)
        {
            p_probe->min_count = dark_count;
        }
        else
        {
            /* Not a new minimum. */
        }

        if (dark_count > p_probe->max_count)
        {
            p_probe->max_count = dark_count;
        }
        else
        {
            /* Not a new maximum. */
        }

        p_probe->num_samples++;
    }
    else
    {
        /* NULL pointer: nothing to add to. */
    }

    return;
}

int32_t
ir_calib_probe_mean (ir_calib_probe_t const * p_probe)
{
    int32_t mean = 0;

    if ((NULL != p_probe) && (p_probe->num_samples > 0u))
    {
        /* Cast: the sample count is at most a few thousand, so it always
         * fits int32_t. */
        mean = p_probe->sum / (int32_t)p_probe->num_samples;
    }
    else
    {
        /* No samples or NULL pointer: report 0. */
    }

    return (mean);
}

int32_t
ir_calib_probe_spread (ir_calib_probe_t const * p_probe)
{
    int32_t spread = 0;

    if ((NULL != p_probe) && (p_probe->num_samples > 0u))
    {
        spread = p_probe->max_count - p_probe->min_count;
    }
    else
    {
        /* No samples or NULL pointer: report 0. */
    }

    return (spread);
}

/*** end of file ***/
