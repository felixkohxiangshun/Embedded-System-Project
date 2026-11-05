/** @file bc_decode.h
 *
 * @brief Code 39 decoder for the navigation barcodes "*A*" to "*D*".
 *
 * @par
 * Code 39: each character is 9 elements (5 bars and 4 spaces, starting with
 * a bar), exactly 3 of them wide: 2 wide bars and 1 wide space for every
 * character used here.  Characters are separated by one narrow space.  A
 * message "*X*" is 9 + 1 + 9 + 1 + 9 = 29 elements, which is 30 edges (the
 * first edge starts the first bar, the last edge ends the last bar).
 *
 * @par
 * The decoder works on edge TIMES, never on sample counts, so it does not
 * depend on the car's speed.  Inside every character the two widest bars and
 * the widest space are the wide ones, provided they are clearly wider than
 * the rest (LB_BC_WIDE_RATIO_X10).  A character that is not clear is
 * rejected, never guessed.  This rank-based rule also copes with ink that
 * prints bars wider than spaces (or the other way round), because bars and
 * spaces are compared among themselves.
 *
 * @par
 * Pure arithmetic: no hardware, no kernel calls, unit tested on a PC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef BC_DECODE_H
#define BC_DECODE_H

#include <stdint.h>
#include <stdbool.h>

/** Edges and elements of a complete "*X*" message. */
#define BC_DECODE_NUM_EDGES         30u
#define BC_DECODE_NUM_ELEMENTS      29u

/** Result of a decode attempt. */
typedef enum
{
    BC_DECODE_OK = 0,           /* valid "*X*", X is A, B, C or D */
    BC_DECODE_ERR_NULL,         /* a pointer argument was NULL */
    BC_DECODE_ERR_COUNT,        /* not exactly BC_DECODE_NUM_EDGES edges */
    BC_DECODE_ERR_WIDTH,        /* a character has no clear 2+1 wide pattern */
    BC_DECODE_ERR_FRAME,        /* the first or last character is not '*' */
    BC_DECODE_ERR_GAP,          /* an inter-character gap is the wrong size */
    BC_DECODE_ERR_SPEED,        /* the three characters differ too much in
                                   speed */
    BC_DECODE_ERR_COMMAND       /* well formed, but the character is not a
                                   navigation command (A to D) */
} bc_decode_status_t;

/*!
 * @brief Decode the 30 edges of one barcode.
 *
 * Tries the edges as read forward first.  If that fails it tries them as read
 * backwards (the car drove over the code in the other direction) and, if that
 * decodes, sets *p_b_reversed.  The caller decides what to do with a reversed
 * code (LB_BC_ACCEPT_REVERSED).
 *
 * @param[in]  p_edge_us     Edge times in microseconds, oldest first.  The
 *                           first edge starts a dark bar.  Wrap-around of
 *                           the 32-bit timer is handled.
 * @param[in]  num_edges     Number of edges; must be BC_DECODE_NUM_EDGES.
 * @param[out] p_command     'A' to 'D' when the result is BC_DECODE_OK.  When
 *                           the result is BC_DECODE_ERR_COMMAND it holds the
 *                           character that was read (0 if unknown).
 * @param[out] p_b_reversed  true when the code was read backwards.
 *
 * @return BC_DECODE_OK or the reason the edges were rejected.
 */
bc_decode_status_t bc_decode_edges(uint32_t const * p_edge_us,
                                   uint32_t num_edges,
                                   uint8_t * p_command,
                                   bool * p_b_reversed);

/*!
 * @brief Find and decode a code inside a longer list of edges.
 *
 * Stray edges before or after the code (a speck of dust, the edge of the
 * line) make the list longer than BC_DECODE_NUM_EDGES.  This function tries
 * every window of BC_DECODE_NUM_EDGES consecutive edges that starts with a
 * dark bar and returns the first one that decodes.
 *
 * @param[in]  p_edge_us     Edge times in microseconds, oldest first.
 * @param[in]  num_edges     Number of edges (at least BC_DECODE_NUM_EDGES).
 * @param[out] p_command     'A' to 'D' when the result is BC_DECODE_OK.  When
 *                           the result is BC_DECODE_ERR_COMMAND it holds the
 *                           character that was read (0 if unknown).
 * @param[out] p_b_reversed  true when the code was read backwards.
 *
 * @return BC_DECODE_OK, or the failure that came closest to a decode.
 */
bc_decode_status_t bc_decode_scan(uint32_t const * p_edge_us,
                                  uint32_t num_edges,
                                  uint8_t * p_command,
                                  bool * p_b_reversed);

/*!
 * @brief Remove glitches (very short elements) from a list of edges.
 *
 * An element shorter than LB_BC_GLITCH_X10 tenths of the median element is a
 * speck on the print or electrical noise.  Its two edges are deleted, which
 * merges the elements on both sides of it.  Works in place.
 *
 * @param[in,out] p_edge_us  Edge times, oldest first; compacted in place.
 * @param[in]     num_edges  Number of edges (at most LB_BC_MAX_EDGES).
 *
 * @return The number of edges that remain (num_edges if none was removed).
 */
uint32_t bc_decode_merge_glitches(uint32_t * p_edge_us, uint32_t num_edges);

#endif /* BC_DECODE_H */
/*** end of file ***/
