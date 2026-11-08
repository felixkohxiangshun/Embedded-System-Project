/** @file barcode_task.c
 *
 * @brief Barcode task.  See barcode_task.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stdint.h>
#include <stdbool.h>
#include "line_barcode.h"
#include "bc_decode.h"
#include "bc_stream.h"
#include "barcode_task.h"

/* The capture lives in static memory: it is about 200 bytes and the decoder
 * needs a copy of it, which would crowd a 2 KB task stack. */
static bc_stream_t g_stream;

/** What happened while one code was being captured. */
typedef struct
{
    bool b_posted;      /* BC_OK or BC_FAIL was already sent */
} capture_t;

/* Private function prototypes. */
static void capture_one_code(capture_t * p_capture);
static bool handle_edge(capture_t * p_capture, MSG_EDGE const * p_msg);
static void decode_and_post(capture_t * p_capture);
static void finish_capture(capture_t * p_capture);
static void post_message(uint8_t kind, uint8_t command);
static void record_trace(void);

void
barcode_task (INT stacd, void * p_exinf)
{
    capture_t capture;

    (void)stacd;
    (void)p_exinf;

    capture.b_posted     = false;
    g_lb_stats.bc_status = LB_BC_STATUS_NONE;

    for (;;)
    {
        capture_one_code(&capture);
    }
}

/*!
 * @brief Wait for one code, collect its edges and report the result.
 *
 * Blocks without a time-out while no capture is running.  Once the first
 * dark bar has arrived, the wait for each next edge is limited to the
 * "silence" time of the stream (several times the longest element seen), so
 * a code that ends, or a car that stops on it, closes the capture.
 */
static void
capture_one_code (capture_t * p_capture)
{
    MSG_EDGE msg;
    bool     b_done  = false;
    TMO      timeout = TMO_FEVR;
    INT      size    = 0;

    bc_stream_reset(&g_stream);
    p_capture->b_posted = false;
    g_lb_stats.bc_state = LB_BC_STATE_WAITING;

    while (!b_done)
    {
        /* Cast: the silence time is at most LB_BC_SILENCE_MAX_MS (6000),
         * so it fits the kernel's TMO. */
        timeout = bc_stream_is_active(&g_stream)
                  ? (TMO)bc_stream_silence_ms(&g_stream)
                  : TMO_FEVR;

        size = MBF_RECV(mbf_edge, msg, timeout);

        if (size > 0)
        {
            b_done = handle_edge(p_capture, &msg);
        }
        else if (bc_stream_is_active(&g_stream))
        {
            /* Silence (E_TMOUT) or any receive error: the capture is over. */
            b_done = true;
        }
        else
        {
            /* Receive error while idle: wait a little, so a broken buffer
             * cannot turn this loop into a busy loop. */
            (void)tk_dly_tsk(10);
        }
    }

    finish_capture(p_capture);

    return;
}

/*!
 * @brief Add one edge to the capture and react to what the stream reports.
 *
 * @return true if the capture is finished (result posted or buffer full).
 */
static bool
handle_edge (capture_t * p_capture, MSG_EDGE const * p_msg)
{
    bool              b_done = false;
    bc_stream_event_t event  = BC_STREAM_EVENT_NONE;

    g_lb_stats.bc_edges++;
    g_lb_stats.bc_state = LB_BC_STATE_CAPTURING;

    /* Cast: the kernel's UW is a 32-bit unsigned integer, as uint32_t. */
    event = bc_stream_push(&g_stream,
                           (uint32_t)p_msg->t_us,
                           (0u != p_msg->dark));

    if (BC_STREAM_EVENT_SEEN == event)
    {
        g_lb_stats.bc_seen++;
        post_message(BC_SEEN, 0u);
    }
    else if (BC_STREAM_EVENT_FULL == event)
    {
        /* A whole code worth of edges: decode at once, so the command does
         * not wait for the silence time-out.  If the decode fails the
         * capture simply goes on (late edges may complete the pattern). */
        decode_and_post(p_capture);
        b_done = p_capture->b_posted;
    }
    else if (BC_STREAM_EVENT_OVERFLOW == event)
    {
        b_done = true;
    }
    else
    {
        /* Edge stored, nothing to report. */
    }

    return (b_done);
}

/*!
 * @brief Try to decode the capture; on success post BC_OK.
 *
 * A code read backwards is rejected unless LB_BC_ACCEPT_REVERSED is set: a
 * reversed read means the car crossed the code the wrong way round, and the
 * turn command would be wrong.
 */
static void
decode_and_post (capture_t * p_capture)
{
    bc_decode_status_t status     = BC_DECODE_OK;
    uint8_t            command    = 0u;
    bool               b_reversed = false;
    bool               b_accept   = false;

    g_lb_stats.bc_state = LB_BC_STATE_DECODING;

    status   = bc_stream_decode(&g_stream, &command, &b_reversed);
    b_accept = (BC_DECODE_OK == status)
               && ((!b_reversed) || (0 != LB_BC_ACCEPT_REVERSED));

    /* Kept for the dashboard's debug frame: why the last attempt failed. */
    g_lb_stats.bc_count  = bc_stream_count(&g_stream);
    g_lb_stats.bc_status = ((BC_DECODE_OK == status) && (!b_accept))
                           ? LB_BC_STATUS_REVERSED
                           : (uint32_t)status;

    if (b_accept)
    {
        g_lb_stats.bc_state = LB_BC_STATE_POST;
        g_lb_stats.bc_last  = command;
        g_lb_stats.bc_decoded++;
        post_message(BC_OK, command);
        p_capture->b_posted = true;
    }
    else
    {
        /* Not decodable (yet); the caller decides whether it is final. */
    }

    return;
}

/*!
 * @brief Close the capture.  If nothing was posted yet, decode what was
 *        collected; a capture that was announced but cannot be decoded is
 *        reported as BC_FAIL so that Mission leaves BARCODE_SCAN.
 */
static void
finish_capture (capture_t * p_capture)
{
    if (!p_capture->b_posted)
    {
        if (bc_stream_seen_sent(&g_stream))
        {
            decode_and_post(p_capture);

            if (!p_capture->b_posted)
            {
                g_lb_stats.bc_state = LB_BC_STATE_FAILED;
                g_lb_stats.bc_failed++;
                post_message(BC_FAIL, 0u);
            }
            else
            {
                /* The late decode succeeded and was posted. */
            }
        }
        else
        {
            /* Only a few stray edges (dirt, a shadow): Mission was never
             * told, so there is nothing to withdraw.  Still shown on the
             * dashboard, so a capture that ends too early is visible. */
            g_lb_stats.bc_count  = bc_stream_count(&g_stream);
            g_lb_stats.bc_status = LB_BC_STATUS_SHORT;
        }
    }
    else
    {
        /* The result was posted while capturing. */
    }

    record_trace();

    return;
}

/*!
 * @brief Copy the edges of the finished capture for the dashboard.
 */
static void
record_trace (void)
{
    uint32_t idx   = 0u;
    uint32_t count = bc_stream_count(&g_stream);

    /* The stream is read directly: barcode_task owns it and nothing else
     * changes it while this runs. */
    for (idx = 0u; (idx < count) && (idx < LB_BC_MAX_EDGES); idx++)
    {
        g_lb_bc_trace.edge_us[idx] = g_stream.edge_us[idx];
    }

    g_lb_bc_trace.count  = idx;
    g_lb_bc_trace.status = g_lb_stats.bc_status;
    g_lb_bc_trace.seq++;

    return;
}

static void
post_message (uint8_t kind, uint8_t command)
{
    MSG_BCCMD msg;

    /* Casts: MSG_BCCMD uses the kernel's UB type (app_if.h). */
    msg.kind = (UB)kind;
    msg.cmd  = (UB)command;

    (void)MBF_SEND(mbf_bccmd, msg);

    return;
}

/*** end of file ***/
