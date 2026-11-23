/** @file ir_calib.h
 *
 * @brief Turns a raw ADC count into a normalised darkness (0 white .. 1000
 *        black) using the white and black levels of one IR sensor.
 *
 * @par
 * Pure arithmetic: no hardware, no kernel calls, so it is unit tested on a
 * PC (testing/line_barcode/lb_host_tests.c).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef IR_CALIB_H
#define IR_CALIB_H

#include <stdint.h>
#include <stdbool.h>

/** Calibration of one sensor, in dark-scale counts (higher is darker). */
typedef struct
{
    int32_t white_level;    /* count over bare paper */
    int32_t black_level;    /* count over the fresh black line */
} ir_calib_t;

/** Statistics of one sensor while it is held still over one surface.  Used
 *  by the guided calibration of the test build.  Treat as private. */
typedef struct
{
    int32_t  sum;           /* sum of all samples */
    int32_t  min_count;     /* smallest sample */
    int32_t  max_count;     /* largest sample */
    uint32_t num_samples;   /* how many samples were added */
} ir_calib_probe_t;

/*!
 * @brief Apply the sensor polarity so a higher count always means darker.
 *
 * @param[in] raw_adc  ADC result, 0 to LB_ADC_MAX_COUNT (LB_ADC_BITS bits).
 *
 * @return Dark-scale count, 0 to LB_ADC_MAX_COUNT.
 */
int32_t ir_calib_raw_to_dark(uint16_t raw_adc);

/*!
 * @brief Normalise a dark-scale count to 0 (white) .. LB_DARK_MAX (black).
 *
 * @param[in] p_cal       Calibration of this sensor; must not be NULL.
 * @param[in] dark_count  Dark-scale count from ir_calib_raw_to_dark().
 *
 * @return Darkness 0 to LB_DARK_MAX; 0 if the calibration is invalid.
 */
int32_t ir_calib_darkness(ir_calib_t const * p_cal, int32_t dark_count);

/*!
 * @brief Check that a calibration is usable (black clearly above white).
 *
 * @param[in] p_cal  Calibration to check; must not be NULL.
 *
 * @return true if the span is at least LB_CAL_SPAN_MIN counts.
 */
bool ir_calib_is_valid(ir_calib_t const * p_cal);

/*!
 * @brief Start a new measurement of one surface.
 *
 * @param[out] p_probe  Probe to clear; must not be NULL.
 */
void ir_calib_probe_init(ir_calib_probe_t * p_probe);

/*!
 * @brief Add one dark-scale sample to a probe.
 *
 * @param[in,out] p_probe     Probe; must not be NULL.
 * @param[in]     dark_count  Dark-scale count, 0 to LB_ADC_MAX_COUNT.
 */
void ir_calib_probe_push(ir_calib_probe_t * p_probe, int32_t dark_count);

/*!
 * @brief Average of all samples of a probe.
 *
 * @param[in] p_probe  Probe; must not be NULL.
 *
 * @return Mean count; 0 if no sample was added.
 */
int32_t ir_calib_probe_mean(ir_calib_probe_t const * p_probe);

/*!
 * @brief Noise of a probe: largest minus smallest sample.
 *
 * @param[in] p_probe  Probe; must not be NULL.
 *
 * @return Peak-to-peak spread in counts; 0 if no sample was added.
 */
int32_t ir_calib_probe_spread(ir_calib_probe_t const * p_probe);

#endif /* IR_CALIB_H */
/*** end of file ***/
