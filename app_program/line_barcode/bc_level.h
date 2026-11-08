/** @file bc_level.h
 *
 * @brief Debouncer for the barcode sensor's digital output (DO on GP28).
 *
 * @par
 * In digital mode every change of the DO pin raises the bank-0 GPIO
 * interrupt, which calls bc_level_change() with a microsecond time stamp.
 * A change becomes an edge only once the level has held for
 * LB_BC_GPIO_DEBOUNCE_US, so comparator chatter at a bar edge is ignored.
 * The edge keeps the time of the last change, not the time it was
 * confirmed.  The edge is confirmed by the next change, or by
 * bc_level_poll() from the 1 kHz sampler when the level then stays put.
 *
 * @par
 * This replaces the 10 ms filter and the slicer for the digital sensor:
 * those flattened the 1 to 5 ms narrow elements seen at driving speed.
 * Pure logic with no hardware access, so it is tested on the PC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef BC_LEVEL_H
#define BC_LEVEL_H

#include <stdint.h>
#include <stdbool.h>
#include "bc_slicer.h"

/** State of the debouncer.  Treat as private; use the functions below. */
typedef struct
{
    uint32_t pend_us;       /* time of the last change of the pin */
    bool     b_pend_dark;   /* level after that change */
    bool     b_sent_dark;   /* level of the last edge reported */
} bc_level_t;

/*!
 * @brief Start from the current pin level (no edge is reported for it).
 */
void bc_level_init(bc_level_t * p_level, bool b_dark, uint32_t now_us);

/*!
 * @brief The pin changed (or may have): record it.
 *
 * @param[in]  b_dark  Level read from the pin now.
 * @param[out] p_edge  Filled when the previous change is confirmed.
 *
 * @return true if p_edge holds a confirmed edge.
 */
bool bc_level_change(bc_level_t * p_level, uint32_t now_us, bool b_dark,
                     bc_slicer_edge_t * p_edge);

/*!
 * @brief Periodic check: confirm a change that has held long enough.
 *
 * @return true if p_edge holds a confirmed edge.
 */
bool bc_level_poll(bc_level_t * p_level, uint32_t now_us,
                   bc_slicer_edge_t * p_edge);

#endif /* BC_LEVEL_H */
/*** end of file ***/
