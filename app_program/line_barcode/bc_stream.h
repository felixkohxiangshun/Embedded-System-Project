/** @file bc_stream.h
 *
 * @brief Collects the edges of one barcode and decodes them.
 *
 * @par
 * The barcode task feeds every edge that the slicer found into this module.
 * It starts a capture at the first dark edge, remembers the edge times,
 * tells the task when to announce "barcode seen", when a full code is
 * ready, and how long to wait for the next edge before the capture is over.
 * The wait scales with the slowest element seen so far, so a slowly driving
 * car never has its code cut short and a fast one is not kept waiting.
 *
 * @par
 * When the capture ends, bc_stream_decode() removes glitches and searches
 * the collected edges for a valid code, so stray edges in front of or
 * behind the code do no harm.
 *
 * @par
 * Pure logic: no hardware, no kernel calls, unit tested on a PC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef BC_STREAM_H
#define BC_STREAM_H

#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "bc_decode.h"

/** What the caller should do after an edge was pushed. */
typedef enum
{
    BC_STREAM_EVENT_NONE = 0,   /* nothing special */
    BC_STREAM_EVENT_SEEN,       /* enough edges: announce "barcode seen" */
    BC_STREAM_EVENT_FULL,       /* a complete code worth of edges: try to
                                   decode now */
    BC_STREAM_EVENT_OVERFLOW    /* buffer full: the capture must end now */
} bc_stream_event_t;

/** State of the capture.  Treat as private; use the functions below. */
typedef struct
{
    uint32_t edge_us[LB_BC_MAX_EDGES];  /* edge times of this capture */
    uint32_t count;                     /* how many edges are stored */
    uint32_t last_us;                   /* time of the newest edge */
    uint32_t longest_us;                /* longest element so far */
    bool     b_active;                  /* a capture is running */
    bool     b_last_dark;               /* polarity of the newest edge */
    bool     b_seen_sent;               /* SEEN was already reported */
} bc_stream_t;

/*!
 * @brief Empty the capture and wait for the next dark edge.
 *
 * @param[out] p_stream  Capture to reset; must not be NULL.
 */
void bc_stream_reset(bc_stream_t * p_stream);

/*!
 * @brief Add one edge.
 *
 * Light edges are ignored while no capture is running, and an edge of the
 * same polarity as the previous one is ignored (edges must alternate).
 *
 * @param[in,out] p_stream  Capture; must not be NULL.
 * @param[in]     time_us   Time of the edge in microseconds.
 * @param[in]     b_dark    true if the sensor now sees a dark bar.
 *
 * @return What the caller should do next.
 */
bc_stream_event_t bc_stream_push(bc_stream_t * p_stream,
                                 uint32_t time_us,
                                 bool b_dark);

/*!
 * @brief Is a capture running?
 *
 * @param[in] p_stream  Capture; must not be NULL.
 *
 * @return true if at least one edge was collected and not yet decoded.
 */
bool bc_stream_is_active(bc_stream_t const * p_stream);

/*!
 * @brief Has "barcode seen" been announced for this capture?
 *
 * @param[in] p_stream  Capture; must not be NULL.
 *
 * @return true if bc_stream_push() returned BC_STREAM_EVENT_SEEN.
 */
bool bc_stream_seen_sent(bc_stream_t const * p_stream);

/*!
 * @brief Number of edges collected so far.
 *
 * @param[in] p_stream  Capture; must not be NULL.
 *
 * @return Edge count; 0 if p_stream is NULL.
 */
uint32_t bc_stream_count(bc_stream_t const * p_stream);

/*!
 * @brief How long to wait for the next edge before the capture is over.
 *
 * It is the larger of LB_BC_SILENCE_MIN_MS and LB_BC_SILENCE_FACTOR times
 * the longest element seen, limited to LB_BC_SILENCE_MAX_MS.
 *
 * @param[in] p_stream  Capture; must not be NULL.
 *
 * @return Time in milliseconds.
 */
uint32_t bc_stream_silence_ms(bc_stream_t const * p_stream);

/*!
 * @brief Decode the edges collected so far.
 *
 * Works on a copy: glitches are merged away and the edge list is searched
 * for a valid code.  The capture itself is not changed, so the caller may
 * keep collecting if the result is not BC_DECODE_OK.
 *
 * @param[in]  p_stream      Capture; must not be NULL.
 * @param[out] p_command     'A' to 'D' on success; the character read on
 *                           BC_DECODE_ERR_COMMAND.
 * @param[out] p_b_reversed  true if the code was read backwards.
 *
 * @return BC_DECODE_OK or the reason for the failure.
 */
bc_decode_status_t bc_stream_decode(bc_stream_t const * p_stream,
                                    uint8_t * p_command,
                                    bool * p_b_reversed);

#endif /* BC_STREAM_H */
/*** end of file ***/
