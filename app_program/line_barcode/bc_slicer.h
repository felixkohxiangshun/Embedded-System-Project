/** @file bc_slicer.h
 *
 * @brief Digital slicer: turns the barcode sensor signal into a clean stream
 *        of bar/space edges with microsecond time stamps.
 *
 * @par
 * Called once per millisecond from the 1 kHz sampler interrupt.  It is a
 * peak-and-valley ("zig-zag") detector.  It remembers the last confirmed
 * extreme of the signal and a candidate for the next one.  When the signal
 * has moved at least LB_BC_PROMINENCE_PCT of the nominal swing and then
 * turned around (or stopped for LB_BC_SETTLE_SAMPLES), the move is a bar
 * edge.  The edge is placed where the signal crossed the mid-level between
 * the valley and the peak, interpolated between two samples, so its time is
 * accurate to a fraction of a millisecond.
 *
 * @par
 * Which mid-level: the edge of a bar is where the signal is half way
 * between bare paper and full ink.  Those two levels are learned from the
 * extremes seen so far (starting from LB_BC_WHITE_NOMINAL and
 * LB_BC_BLACK_NOMINAL).  When a narrow bar is blurred so much that it never
 * reaches full ink, the level is pulled down to stay inside that bar's own
 * rise, so its edges are still found.
 *
 * @par
 * Why not one fixed threshold: a narrow bar is often narrower than the
 * sensor spot, so it never reaches the full swing and a global threshold
 * would miss it or place its edges late.  Each edge here uses its own local
 * mid-level, so lighting changes and weak ink move the signal but not the
 * edge positions.
 *
 * @par
 * Pure arithmetic: unit tested on a PC (testing/line_barcode).  Cost: a few
 * operations per sample, plus a search of at most LB_BC_HISTORY_SAMPLES
 * samples when an edge is confirmed (about 10 microseconds on the RP2040).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef BC_SLICER_H
#define BC_SLICER_H

#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"

/** One bar/space edge. */
typedef struct
{
    uint32_t time_us;   /* time of the edge, interpolated */
    bool     b_dark;    /* true: the sensor now sees a dark bar */
} bc_slicer_edge_t;

/** State of the slicer.  Treat as private; use the functions below. */
typedef struct
{
    int16_t  value[LB_BC_HISTORY_SAMPLES];      /* recent samples (ring) */
    uint32_t time_us[LB_BC_HISTORY_SAMPLES];    /* their times (ring) */
    uint32_t sample_no;     /* number of samples received so far */
    uint32_t last_no;       /* sample number of the last confirmed extreme */
    int32_t  last_val;      /* its value */
    uint32_t cand_no;       /* sample number of the candidate extreme */
    int32_t  cand_val;      /* its value */
    int32_t  b_ref;         /* learned level of full ink (dark scale) */
    int32_t  w_ref;         /* learned level of bare paper */
    int32_t  warmup_left;   /* samples still to ignore after start-up */
    bool     b_up;          /* true: the last extreme was a valley, so the
                               signal is expected to rise next */
    bool     b_primed;      /* the first sample has been taken */
} bc_slicer_t;

/*!
 * @brief Reset the slicer (waiting for the first sample, warm-up started).
 *
 * @param[out] p_slicer  Slicer to initialise; must not be NULL.
 */
void bc_slicer_init(bc_slicer_t * p_slicer);

/*!
 * @brief Feed one 1 ms sample; report an edge when one is confirmed.
 *
 * An edge is reported a little AFTER it happened (when the move is
 * finished), but its time stamp is the time it really occurred.  At most
 * one edge is reported per call.
 *
 * @param[in,out] p_slicer  Slicer state; must not be NULL.
 * @param[in]     dark      Dark-scale count (0 to LB_ADC_MAX_COUNT, higher is darker).
 * @param[in]     now_us    Free-running microsecond time of this sample.
 * @param[out]    p_edge    Filled only when the function returns true.
 *
 * @return true if a new edge was confirmed.
 */
bool bc_slicer_step(bc_slicer_t * p_slicer,
                    int32_t dark,
                    uint32_t now_us,
                    bc_slicer_edge_t * p_edge);

#endif /* BC_SLICER_H */
/*** end of file ***/
