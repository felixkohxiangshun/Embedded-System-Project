/** @file bc_level.c
 *
 * @brief Debouncer for the barcode sensor's digital output.  See bc_level.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "bc_level.h"

/* Private function prototypes. */
static bool confirm(bc_level_t * p_level, uint32_t now_us,
                    bc_slicer_edge_t * p_edge);

void
bc_level_init (bc_level_t * p_level, bool b_dark, uint32_t now_us)
{
    if (NULL != p_level)
    {
        p_level->pend_us     = now_us;
        p_level->b_pend_dark = b_dark;
        p_level->b_sent_dark = b_dark;
    }
    else
    {
        /* NULL pointer: nothing to initialise. */
    }

    return;
}

bool
bc_level_change (bc_level_t * p_level, uint32_t now_us, bool b_dark,
                 bc_slicer_edge_t * p_edge)
{
    bool b_edge = false;

    if ((NULL != p_level) && (NULL != p_edge))
    {
        /* The level before this change may have held long enough. */
        b_edge = confirm(p_level, now_us, p_edge);

        if (b_dark != p_level->b_pend_dark)
        {
            p_level->b_pend_dark = b_dark;
            p_level->pend_us     = now_us;
        }
        else
        {
            /* Same level again (two edges latched at once): no change. */
        }
    }
    else
    {
        /* NULL pointer: nothing to record. */
    }

    return (b_edge);
}

bool
bc_level_poll (bc_level_t * p_level, uint32_t now_us,
               bc_slicer_edge_t * p_edge)
{
    bool b_edge = false;

    if ((NULL != p_level) && (NULL != p_edge))
    {
        b_edge = confirm(p_level, now_us, p_edge);
    }
    else
    {
        /* NULL pointer: nothing to check. */
    }

    return (b_edge);
}

/*!
 * @brief Report the pending level as an edge if it differs from the last
 *        edge and has held for LB_BC_GPIO_DEBOUNCE_US.
 */
static bool
confirm (bc_level_t * p_level, uint32_t now_us, bc_slicer_edge_t * p_edge)
{
    bool b_edge = false;

    /* Unsigned subtraction: correct across a timer wrap. */
    if ((p_level->b_pend_dark != p_level->b_sent_dark)
        && ((now_us - p_level->pend_us) >= LB_BC_GPIO_DEBOUNCE_US))
    {
        p_edge->time_us      = p_level->pend_us;
        p_edge->b_dark       = p_level->b_pend_dark;
        p_level->b_sent_dark = p_level->b_pend_dark;
        b_edge               = true;
    }
    else
    {
        /* Nothing new, or still inside the debounce time. */
    }

    return (b_edge);
}

/*** end of file ***/
