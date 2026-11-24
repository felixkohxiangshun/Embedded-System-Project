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

/* Longest single wait for an edge while a capture runs or is on hold, so
 * that Mission's HOLD and ARM requests are seen within a few ms. */
#define POLL_MS                 20

/* A hold that Mission never ends (it went to IDLE or a fault) is dropped
 * after this long, so the reader cannot stay deaf. */
#define HOLD_MAX_MS             8000

/* The capture lives in static memory: it is about 200 bytes and the decoder
 * needs a copy of it, which would crowd a 2 KB task stack. */
static bc_stream_t g_stream;

/* true while Mission backs the car up: edges are dropped (EVT_BC_HOLD). */
static bool    g_b_hold   = false;
static int32_t g_hold_ms  = 0;

/* Forward travel at the first edge of the capture. */
static int32_t g_first_mm = 0;

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
static bool check_mission(void);
static int32_t travelled_mm(void);
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
 * dark bar has arrived, the capture ends when the sensor has seen white for
 * LB_BC_WHITE_EXIT_MS, or has sat on a dark bar for the stream's "silence"
 * time (several times the longest element seen).  The wait
 * is cut into POLL_MS steps so that Mission's requests are seen at once:
 * EVT_BC_HOLD drops the capture and ignores edges while the car backs up,
 * EVT_BC_ARM starts a fresh capture for the slow pass over the code.
 */
static void
capture_one_code (capture_t * p_capture)
{
    MSG_EDGE msg;
    bool     b_done   = false;
    TMO      timeout  = TMO_FEVR;
    INT      size     = 0;
    int32_t  quiet_ms = 0;

    bc_stream_reset(&g_stream);
    p_capture->b_posted = false;

    while (!b_done)
    {
        if (g_b_hold || bc_stream_is_active(&g_stream))
        {
            timeout = POLL_MS;
        }
        else
        {
            timeout = TMO_FEVR;
        }

        size = MBF_RECV(mbf_edge, msg, timeout);

        if (check_mission())
        {
            /* HOLD or ARM: whatever was collected is from the fast pass. */
            bc_stream_reset(&g_stream);
            p_capture->b_posted = false;
            quiet_ms            = 0;
        }
        else
        {
            /* No request from Mission. */
        }

        if (g_b_hold)
        {
            /* Backing up: drop the edge (or the time-out). */
            g_hold_ms += POLL_MS;

            if (g_hold_ms >= HOLD_MAX_MS)
            {
                g_b_hold = false;
            }
            else
            {
                /* Still waiting for EVT_BC_ARM. */
            }
        }
        else if (size > 0)
        {
            quiet_ms = 0;
            b_done   = handle_edge(p_capture, &msg);
        }
        else if (bc_stream_is_active(&g_stream))
        {
            quiet_ms += POLL_MS;

            /* Casts: both limits are at most 6000 ms, so they fit. */
            if ((!g_stream.b_last_dark)
                && (quiet_ms >= (int32_t)LB_BC_WHITE_EXIT_MS))
            {
                b_done = true;      /* white long enough: the code is over */
            }
            else if (quiet_ms >= (int32_t)bc_stream_silence_ms(&g_stream))
            {
                b_done = true;      /* stuck on a dark bar: give up */
            }
            else
            {
                /* Keep waiting for the next edge. */
            }
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
 * @brief Read Mission's barcode requests from flg_evt without waiting.
 *
 * @return true if HOLD or ARM arrived (the capture must start again).
 */
static bool
check_mission (void)
{
    UINT ptn       = 0u;
    bool b_restart = false;

    if (E_OK == tk_wai_flg(flg_evt, EVT_BC_HOLD | EVT_BC_ARM,
                           TWF_ORW | TWF_BITCLR, &ptn, TMO_POL))
    {
        b_restart = true;
        g_hold_ms = 0;

        /* If both arrived, ARM is the later one: the car drives forward. */
        g_b_hold  = (0u == (ptn & EVT_BC_ARM));
    }
    else
    {
        /* Nothing from Mission. */
    }

    return (b_restart);
}

/*!
 * @brief Forward travel of the car, mm (backwards counts negative).
 *
 * The mean of the two wheels, so a steering correction or a pivot adds
 * nothing (the path length, dist_mm, would count those).
 * Read without the state lock: fwd_mm is one aligned 32-bit word that the
 * Motor task writes, so the read is never torn.
 */
static int32_t
travelled_mm (void)
{
    /* Cast: a volatile read, because another task writes it; INT is the
     * kernel's 32-bit signed integer. */
    return ((int32_t)(*(volatile INT const *)&app_state.fwd_mm));
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
    uint32_t          before = bc_stream_count(&g_stream);
    int32_t           now_mm = travelled_mm();

    g_lb_stats.bc_edges++;

    /* Cast: LB_BC_FIRST_WINDOW_MM is a small constant. */
    if ((2u == before) && (0u != p_msg->dark)
        && ((now_mm - g_first_mm) > (int32_t)LB_BC_FIRST_WINDOW_MM))
    {
        /* One lone dark band (a junction line) and then, much later, a
         * dark bar: start again with this bar as the first. */
        bc_stream_reset(&g_stream);
        before = 0u;
    }
    else
    {
        /* Part of the same capture. */
    }

    /* Cast: the kernel's UW is a 32-bit unsigned integer, as uint32_t. */
    event = bc_stream_push(&g_stream,
                           (uint32_t)p_msg->t_us,
                           (0u != p_msg->dark));

    if ((0u == before) && (1u == bc_stream_count(&g_stream)))
    {
        g_first_mm = now_mm;
    }
    else if ((2u == before) && (3u == bc_stream_count(&g_stream)))
    {
        /* Second dark bar close behind the first: a code, not a junction.
         * Mission may back up and read it slowly. */
        post_message(BC_FIRST, 0u);
    }
    else
    {
        /* Not one of the edges that matter here. */
    }

    if (BC_STREAM_EVENT_SEEN == event)
    {
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
