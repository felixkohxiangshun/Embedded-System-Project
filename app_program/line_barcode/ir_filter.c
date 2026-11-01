/** @file ir_filter.c
 *
 * @brief Sliding-window average.  See ir_filter.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include "lb_config.h"
#include "ir_filter.h"

void
ir_filter_init (ir_filter_t * p_filter)
{
    uint32_t idx = 0u;

    if (NULL != p_filter)
    {
        for (idx = 0u; idx < LB_LINE_AVG_SAMPLES; idx++)
        {
            p_filter->sample[idx] = 0;
        }

        p_filter->sum    = 0;
        p_filter->next   = 0u;
        p_filter->filled = 0u;
    }
    else
    {
        /* NULL pointer: nothing to initialise. */
    }

    return;
}

int32_t
ir_filter_push (ir_filter_t * p_filter, int32_t sample)
{
    int32_t average = 0;

    if (NULL != p_filter)
    {
        /* Replace the oldest sample: subtract it from the running sum and
         * add the new one, so the cost does not depend on the window size.
         * An empty slot holds 0, so this is correct while filling too. */
        p_filter->sum -= p_filter->sample[p_filter->next];
        p_filter->sample[p_filter->next] = sample;
        p_filter->sum += sample;

        p_filter->next++;
        if (p_filter->next >= LB_LINE_AVG_SAMPLES)
        {
            p_filter->next = 0u;
        }
        else
        {
            /* Window not wrapped yet. */
        }

        if (p_filter->filled < LB_LINE_AVG_SAMPLES)
        {
            p_filter->filled++;
        }
        else
        {
            /* Window already full. */
        }

        /* Cast: filled is 1 to LB_LINE_AVG_SAMPLES (10), so it always fits
         * int32_t, and it is never 0 here, so the division is safe. */
        average = p_filter->sum / (int32_t)p_filter->filled;
    }
    else
    {
        /* NULL pointer: report 0. */
    }

    return (average);
}

/*** end of file ***/
