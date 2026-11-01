/** @file line_barcode.c
 *
 * @brief Creates the kernel objects of the line and barcode subsystem and
 *        starts its tasks and handlers.  See line_barcode.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stdint.h>
#include <stdbool.h>
#include "line_barcode.h"
#include "ir_driver.h"
#include "line_task.h"
#include "barcode_task.h"

/** Edge buffer: the sampler (a handler) sends, barcode_task receives.  Named
 *  by the team interface file app_if.h, hence no "g_" prefix. */
ID mbf_edge;

ID g_lb_tid_line;
ID g_lb_tid_barcode;
ID g_lb_cyc_ir;
ID g_lb_cyc_line;

lb_stats_t g_lb_stats;

/* Private function prototypes. */
static ID make_cyclic(FP handler, RELTIM period_ms);

ER
line_barcode_init (void)
{
    ER result = E_OK;

    /* A full code is 30 edges; the buffer holds LB_EDGE_BUFFER_COUNT of
     * them, so two codes fit while barcode_task is busy. */
    mbf_edge = app_make_mbf((INT)sizeof(MSG_EDGE), LB_EDGE_BUFFER_COUNT);

    if (mbf_edge < E_OK)
    {
        result = mbf_edge;
    }
    else
    {
        g_lb_cyc_ir = make_cyclic((FP)ir_sampler_isr, 1);

        if (g_lb_cyc_ir < E_OK)
        {
            result = g_lb_cyc_ir;
        }
        else
        {
            g_lb_cyc_line = make_cyclic((FP)line_wakeup_isr, LB_LINE_TICK_MS);

            if (g_lb_cyc_line < E_OK)
            {
                result = g_lb_cyc_line;
            }
            else
            {
                /* All objects created. */
            }
        }
    }

    return (result);
}

ER
line_barcode_start (void)
{
    ER result = E_OK;

    ir_driver_init();

    g_lb_tid_barcode = app_start_task(barcode_task, PRI_BARCODE, 0,
                                      "barcode_task");

    if (g_lb_tid_barcode < E_OK)
    {
        result = g_lb_tid_barcode;
    }
    else
    {
        g_lb_tid_line = app_start_task(line_task, PRI_LINE, 0, "line_task");

        if (g_lb_tid_line < E_OK)
        {
            result = g_lb_tid_line;
        }
        else
        {
            /* Tasks exist; now let the handlers run. */
            result = tk_sta_cyc(g_lb_cyc_ir);

            if (result >= E_OK)
            {
                result = tk_sta_cyc(g_lb_cyc_line);
            }
            else
            {
                /* Report the first error. */
            }
        }
    }

    return (result);
}

/*!
 * @brief Create a cyclic handler that starts after one period.
 *
 * @param[in] handler    Handler function.
 * @param[in] period_ms  Period in milliseconds.
 *
 * @return Cyclic handler ID, or a negative kernel error code.
 */
static ID
make_cyclic (FP handler, RELTIM period_ms)
{
    T_CCYC ccyc = {0};

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = handler;
    ccyc.cyctim = period_ms;
    ccyc.cycphs = period_ms;

    return (tk_cre_cyc(&ccyc));
}

/*** end of file ***/
