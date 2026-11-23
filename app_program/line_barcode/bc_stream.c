/** @file bc_stream.c
 *
 * @brief Collects the edges of one barcode and decodes them.
 *        See bc_stream.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "bc_decode.h"
#include "bc_stream.h"

#define MICROSECONDS_PER_MS     1000u

void
bc_stream_reset (bc_stream_t * p_stream)
{
    uint32_t idx = 0u;

    if (NULL != p_stream)
    {
        for (idx = 0u; idx < LB_BC_MAX_EDGES; idx++)
        {
            p_stream->edge_us[idx] = 0u;
        }

        p_stream->count       = 0u;
        p_stream->last_us     = 0u;
        p_stream->longest_us  = 0u;
        p_stream->b_active    = false;
        p_stream->b_last_dark = false;
        p_stream->b_seen_sent = false;
    }
    else
    {
        /* NULL pointer: nothing to reset. */
    }

    return;
}

bc_stream_event_t
bc_stream_push (bc_stream_t * p_stream, uint32_t time_us, bool b_dark)
{
    bc_stream_event_t event      = BC_STREAM_EVENT_NONE;
    uint32_t          element_us = 0u;

    if (NULL == p_stream)
    {
        /* NULL pointer: nothing to do. */
    }
    else if ((!p_stream->b_active) && (!b_dark))
    {
        /* A code always starts with a dark bar: ignore light edges while
         * waiting. */
    }
    else if (p_stream->b_active && (b_dark == p_stream->b_last_dark))
    {
        /* Two edges of the same polarity cannot follow each other. */
    }
    else
    {
        if (p_stream->b_active)
        {
            /* Unsigned subtraction stays correct when the 32-bit timer
             * wraps around between the two edges. */
            element_us = time_us - p_stream->last_us;

            if (element_us > p_stream->longest_us)
            {
                p_stream->longest_us = element_us;
            }
            else
            {
                /* Not the longest element. */
            }
        }
        else
        {
            p_stream->b_active = true;
        }

        p_stream->edge_us[p_stream->count] = time_us;
        p_stream->count++;
        p_stream->last_us     = time_us;
        p_stream->b_last_dark = b_dark;

        if (p_stream->count >= LB_BC_MAX_EDGES)
        {
            event = BC_STREAM_EVENT_OVERFLOW;
        }
        else if (BC_DECODE_NUM_EDGES == p_stream->count)
        {
            event = BC_STREAM_EVENT_FULL;
        }
        else if ((LB_BC_SEEN_EDGES == p_stream->count)
                 && (!p_stream->b_seen_sent))
        {
            p_stream->b_seen_sent = true;
            event                 = BC_STREAM_EVENT_SEEN;
        }
        else
        {
            /* Collecting: nothing to report yet. */
        }
    }

    return (event);
}

bool
bc_stream_is_active (bc_stream_t const * p_stream)
{
    bool b_active = false;

    if (NULL != p_stream)
    {
        b_active = p_stream->b_active;
    }
    else
    {
        /* NULL pointer: no capture. */
    }

    return (b_active);
}

bool
bc_stream_seen_sent (bc_stream_t const * p_stream)
{
    bool b_sent = false;

    if (NULL != p_stream)
    {
        b_sent = p_stream->b_seen_sent;
    }
    else
    {
        /* NULL pointer: nothing was sent. */
    }

    return (b_sent);
}

uint32_t
bc_stream_count (bc_stream_t const * p_stream)
{
    uint32_t count = 0u;

    if (NULL != p_stream)
    {
        count = p_stream->count;
    }
    else
    {
        /* NULL pointer: no edges. */
    }

    return (count);
}

uint32_t
bc_stream_silence_ms (bc_stream_t const * p_stream)
{
    uint32_t silence_ms = LB_BC_SILENCE_MIN_MS;
    uint32_t longest_ms = 0u;
    uint32_t scaled_ms  = 0u;

    if (NULL != p_stream)
    {
        /* Round up to whole milliseconds so that the wait is never shorter
         * than the element it is derived from. */
        longest_ms = (p_stream->longest_us + (MICROSECONDS_PER_MS - 1u))
                     / MICROSECONDS_PER_MS;

        if (longest_ms > (LB_BC_SILENCE_MAX_MS / LB_BC_SILENCE_FACTOR))
        {
            scaled_ms = LB_BC_SILENCE_MAX_MS;
        }
        else
        {
            scaled_ms = longest_ms * LB_BC_SILENCE_FACTOR;
        }

        if (scaled_ms > silence_ms)
        {
            silence_ms = scaled_ms;
        }
        else
        {
            /* The minimum wait is already the larger. */
        }
    }
    else
    {
        /* NULL pointer: use the minimum wait. */
    }

    return (silence_ms);
}

bc_decode_status_t
bc_stream_decode (bc_stream_t const * p_stream,
                  uint8_t * p_command,
                  bool * p_b_reversed)
{
    bc_decode_status_t status                  = BC_DECODE_ERR_NULL;
    uint32_t           work_us[LB_BC_MAX_EDGES] = { 0u };
    uint32_t           count                   = 0u;
    uint32_t           merged                  = 0u;
    uint32_t           idx                     = 0u;

    if ((NULL != p_stream) && (NULL != p_command) && (NULL != p_b_reversed))
    {
        count = p_stream->count;

        /* Work on a copy so that the capture can keep growing if this
         * attempt fails. */
        for (idx = 0u; idx < count; idx++)
        {
            work_us[idx] = p_stream->edge_us[idx];
        }

        /* Try the edges as captured first: a clean capture of exactly one
         * code must not lose a short (but real) element to the glitch
         * filter.  Only if that fails are the glitches merged away. */
        status = bc_decode_scan(work_us, count, p_command, p_b_reversed);

        if (BC_DECODE_OK != status)
        {
            merged = bc_decode_merge_glitches(work_us, count);

            if (merged != count)
            {
                status = bc_decode_scan(work_us, merged,
                                        p_command, p_b_reversed);
            }
            else
            {
                /* Nothing was merged: the first result stands. */
            }
        }
        else
        {
            /* Decoded as captured. */
        }
    }
    else
    {
        /* NULL pointer: status stays BC_DECODE_ERR_NULL. */
    }

    return (status);
}

/*** end of file ***/
