/** @file ir_filter.h
 *
 * @brief Sliding-window average for the two line sensors.
 *
 * @par
 * Called once per millisecond from the 1 kHz sampler.  Averaging the last
 * LB_LINE_AVG_SAMPLES samples (10 ms) cancels the 100 Hz flicker of mains
 * lighting and removes most ADC noise.  Pure arithmetic: unit tested on a PC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef IR_FILTER_H
#define IR_FILTER_H

#include <stdint.h>
#include "lb_config.h"

/** State of one filter.  Treat as private; use the functions below. */
typedef struct
{
    int32_t  sample[LB_LINE_AVG_SAMPLES];   /* the last samples */
    int32_t  sum;                           /* sum of the stored samples */
    uint32_t next;                          /* index that is overwritten next */
    uint32_t filled;                        /* how many samples are stored */
} ir_filter_t;

/*!
 * @brief Empty the window.
 *
 * @param[out] p_filter  Filter to initialise; must not be NULL.
 */
void ir_filter_init(ir_filter_t * p_filter);

/*!
 * @brief Add one sample and return the average of the window.
 *
 * Until the window is full the average is taken over the samples received so
 * far, so the first outputs are never pulled toward zero.
 *
 * @param[in,out] p_filter  Filter state; must not be NULL.
 * @param[in]     sample    New sample (any int32_t that cannot overflow
 *                          when LB_LINE_AVG_SAMPLES of them are added).
 *
 * @return Average of the window; 0 if p_filter is NULL.
 */
int32_t ir_filter_push(ir_filter_t * p_filter, int32_t sample);

#endif /* IR_FILTER_H */
/*** end of file ***/
