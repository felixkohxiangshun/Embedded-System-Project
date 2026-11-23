/** @file ir_driver.h
 *
 * @brief ADC driver and the 1 kHz sampler of the three IR sensors.
 *
 * @par
 * ir_sampler_isr() is a cyclic handler: the kernel calls it every 1 ms from
 * the timer interrupt (CNF_TIMER_PERIOD must be 1, see config/config.h).
 * It is the single reader of the ADC; no task touches the converter.  Each
 * run it
 *  - reads the right and centre sensors (analogue, ADC0 and ADC1) and the
 *    barcode sensor (digital DO pin on GP28, read as a GPIO level
 *    because LB_BC_USE_GPIO_LEVEL is 1; ADC2 is then unused),
 *  - averages centre and right over 10 ms (cancels 100 Hz mains flicker) for
 *    line_task to read with ir_driver_get_dark(),
 *  - filters the barcode level the same way and passes it to the
 *    slicer, which turns it into timed bar/space edges, and
 *  - sends each edge to barcode_task on mbf_edge.
 * It never blocks: the message is sent with a zero time-out.
 *
 * @par
 * Cost: a conversion takes about 2 microseconds.  With LB_ADC_OVERSAMPLE of
 * 2 and one dummy conversion per channel, one run is about 20 microseconds,
 * 2 percent of the CPU.  The longest run is kept in g_lb_stats.sampler_max_us
 * so the test build can check it.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef IR_DRIVER_H
#define IR_DRIVER_H

#include <stdint.h>

/** Sensor channel numbers.  Right and centre are also the ADC input numbers
 *  of GP26 and GP27; the barcode channel is GP28, read as a digital level. */
#define IR_CHANNEL_RIGHT    0u
#define IR_CHANNEL_CENTRE   1u
#define IR_CHANNEL_BARCODE  2u

/*!
 * @brief Prepare the pins and the ADC, and reset the filters and the slicer.
 *        Call once before the cyclic handler is started.
 */
void ir_driver_init(void);

/*!
 * @brief Cyclic handler, called every millisecond by the kernel.
 *
 * @param[in] p_exinf  Extended information of the cyclic handler; unused.
 */
void ir_sampler_isr(void * p_exinf);

/*!
 * @brief Latest 10 ms average of one sensor, on the dark scale.
 *
 * Safe to call from a task: the value is one aligned 32-bit word.
 *
 * @param[in] channel  IR_CHANNEL_RIGHT, IR_CHANNEL_CENTRE or
 *                     IR_CHANNEL_BARCODE.
 *
 * @return Dark-scale count (0 to LB_ADC_MAX_COUNT, higher is darker); 0 if the channel
 *         number is invalid.  For IR_CHANNEL_BARCODE in digital mode the value
 *         is one of two fixed counts (LB_BC_GPIO_LIGHT_COUNT or
 *         LB_BC_GPIO_DARK_COUNT) averaged over 10 ms.
 */
int32_t ir_driver_get_dark(uint32_t channel);

#endif /* IR_DRIVER_H */
/*** end of file ***/
