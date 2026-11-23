/** @file line_barcode.h
 *
 * @brief Line following and barcode reading subsystem (Buddy 3): public
 *        interface, kernel object IDs and run-time counters.
 *
 * @par
 * What the subsystem does (design review section 6.1, FR-01 to FR-03):
 *  - follows the black line with the centre and right IR sensors, using
 *    a PD controller and a recovery routine for faded or lost lines, and
 *  - reads Code 39 "*X*" barcodes (X = A to Z, either direction) with the
 *    barcode IR sensor
 *    while the car keeps following the line.  The line sensors are
 *    analogue (ADC); the barcode sensor is digital (DO pin, GPIO level).
 *
 * @par
 * Tasks and handlers (ranks from app.h):
 *  - ir_sampler_isr  cyclic handler, 1 kHz; the ONLY user of the ADC.
 *  - line_task       PRI_LINE, woken every 10 ms by line_wakeup_isr.
 *  - barcode_task    PRI_BARCODE, blocks on the edge buffer mbf_edge.
 *
 * @par
 * Messages (app_if.h): MSG_STEER on mbf_steer and MSG_BCCMD on mbf_bccmd,
 * both to Mission.  This subsystem never commands the motors.
 *
 * @par
 * Data flow:
 * @verbatim
 *   ADC (centre, right) and GPIO level (barcode, DO pin)
 *       --> ir_sampler_isr --+--> filtered centre/right --> line_task
 *                            |                              |  line_ctrl
 *                            |                              |  line_recov
 *                            |                              v
 *                            |                         mbf_steer --> Mission
 *                            +--> bc_slicer --> mbf_edge --> barcode_task
 *                                                   bc_stream, bc_decode
 *                                                           |
 *                                                           v
 *                                                      mbf_bccmd --> Mission
 * @endverbatim
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "lb_config.h"

/** Barcode task states (design review statechart 6.1). */
#define LB_BC_STATE_WAITING     0u
#define LB_BC_STATE_CAPTURING   1u
#define LB_BC_STATE_DECODING    2u
#define LB_BC_STATE_POST        3u
#define LB_BC_STATE_FAILED      4u

/** Line task states (design review statechart 6.1). */
#define LB_LINE_STATE_SAMPLING  0u
#define LB_LINE_STATE_STEERING  1u
#define LB_LINE_STATE_LOST      2u

/** Run-time counters.  They are written by the tasks and read by the
 *  on-target tests, the trace output and the debugger; they are never used
 *  to make a control decision.  Each field is written by one context only. */
typedef struct
{
    volatile uint32_t sampler_runs;     /* runs of ir_sampler_isr */
    volatile uint32_t sampler_max_us;   /* longest run of ir_sampler_isr */
    volatile uint32_t edges_sent;       /* edges put on mbf_edge */
    volatile uint32_t edges_dropped;    /* edges lost, mbf_edge was full */
    volatile uint32_t line_ticks;       /* runs of line_task */
    volatile uint32_t line_dt_max_ms;   /* longest gap between two ticks */
    volatile uint32_t line_state;       /* LB_LINE_STATE_xxx */
    volatile int32_t  line_dark_centre; /* last darkness, centre (0 to 1000) */
    volatile int32_t  line_dark_right;  /* last darkness, right */
    volatile int32_t  line_err;         /* last position error */
    volatile int32_t  line_steer;       /* last steering sent */
    volatile uint32_t line_mode;        /* line_recov_mode_t */
    volatile uint32_t line_recoveries;  /* number of times tracking resumed */
    volatile uint32_t line_timeouts;    /* ticks sent with the TIMEOUT flag */
    volatile uint32_t line_junctions;   /* side branches found */
    volatile uint32_t bc_state;         /* LB_BC_STATE_xxx */
    volatile uint32_t bc_edges;         /* edges received by barcode_task */
    volatile uint32_t bc_seen;          /* codes announced (BC_SEEN) */
    volatile uint32_t bc_decoded;       /* codes decoded (BC_OK) */
    volatile uint32_t bc_failed;        /* codes rejected (BC_FAIL) */
    volatile uint32_t bc_last;          /* last decoded character, 0 = none */
    volatile uint32_t bc_status;        /* last decode: bc_decode_status_t,
                                           LB_BC_STATUS_REVERSED if refused */
    volatile uint32_t bc_count;         /* edges in the last decoded capture */
} lb_stats_t;

/* bc_status when a code was read backwards but LB_BC_ACCEPT_REVERSED is 0 */
#define LB_BC_STATUS_REVERSED   8u
/* bc_status when a capture ended with too few edges to be a code (ignored) */
#define LB_BC_STATUS_SHORT      9u
/* bc_status before the first capture has ended */
#define LB_BC_STATUS_NONE       15u

IMPORT lb_stats_t g_lb_stats;

/** Edge times of the last finished capture, for the dashboard's barcode
 *  topic.  seq changes after every capture; the rest is then stable until
 *  the next capture ends.  Written only by barcode_task. */
typedef struct
{
    volatile uint32_t seq;                      /* captures finished */
    volatile uint32_t status;                   /* as g_lb_stats.bc_status */
    volatile uint32_t count;                    /* edges in edge_us */
    volatile uint32_t edge_us[LB_BC_MAX_EDGES]; /* edge times */
} lb_bc_trace_t;

IMPORT lb_bc_trace_t g_lb_bc_trace;

IMPORT ID g_lb_tid_line;        /* line_task */
IMPORT ID g_lb_tid_barcode;     /* barcode_task */
IMPORT ID g_lb_cyc_ir;          /* 1 ms cyclic: ir_sampler_isr */
IMPORT ID g_lb_cyc_line;        /* 10 ms cyclic: line_wakeup_isr */

/*!
 * @brief Create this subsystem's kernel objects (mbf_edge and the cyclic
 *        handlers).  Called by usermain() before any task starts.
 *
 * @return E_OK, or a negative kernel error code.
 */
IMPORT ER line_barcode_init(void);

/*!
 * @brief Start the ADC sampler, the two tasks and both cyclic handlers.
 *
 * @return E_OK, or a negative kernel error code.
 */
IMPORT ER line_barcode_start(void);

#endif /* LINE_BARCODE_H */
/*** end of file ***/
