/*!
 * @file    lb_digital_test.c
 * @brief   PC check of the DIGITAL barcode path (module DO pin on GP28).
 *
 * @par
 * ir_driver.c turns the DO level into two fixed counts (LB_BC_GPIO_LIGHT_COUNT
 * and LB_BC_GPIO_DARK_COUNT).  This test feeds such a two-level signal, one
 * sample per millisecond, through the real 10 ms filter, the slicer and the
 * decoder, exactly as ir_sampler_isr does.
 *
 * @par
 * Cases: every command A to D, three car speeds, and the faults a digital
 * sensor really has:
 *  - threshold offset: the trimmer makes every bar wider (or narrower) than
 *    it is printed, and every space narrower by the same amount;
 *  - contact bounce: the DO level chatters for a few samples at each edge.
 *
 * @par
 * Run: make digital      (exit code 0 when every case passes)
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bc_decode.h"
#include "bc_slicer.h"
#include "ir_filter.h"
#include "lb_config.h"

#define NUM_CODES       4
#define NUM_SPEEDS      3
#define MAX_SAMPLES     200000
#define MAX_EDGES       128
#define LEAD_MS         300         /* bare paper before the code */
#define TRAIL_MS        1000        /* bare paper after the code  */

/* Element patterns, bar first: N narrow, W wide.  "*" then A to D. */
static char const * const g_star    = "NWNNWNWNN";
static char const * const g_pat[NUM_CODES] =
{
    "WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW"
};
static char const g_names[NUM_CODES + 1] = "ABCD";

/* One narrow element in milliseconds at the three test speeds. */
static int32_t const g_narrow_ms[NUM_SPEEDS] = { 25, 50, 100 };

static uint8_t  g_level[MAX_SAMPLES];
static uint32_t g_rng = 12345u;

/* Small repeatable pseudo-random bit for the bounce model. */
static bool
next_bit (void)
{
    g_rng = (g_rng * 1664525u) + 1013904223u;
    return (0u != ((g_rng >> 16) & 1u));
}

/*!
 * @brief Build the DO level (1 = bar) for one scan, one entry per ms.
 *
 * @param[in] code        0 to 3 for A to D.
 * @param[in] narrow_ms   Milliseconds per narrow element.
 * @param[in] offset_pct  Bar widening in percent of a narrow element;
 *                        spaces shrink by the same amount.
 * @param[in] bounce_ms   Samples at each edge that may flip at random.
 *
 * @return Number of samples written.
 */
static uint32_t
build_signal (int32_t code, int32_t narrow_ms, int32_t offset_pct,
              int32_t bounce_ms)
{
    char     elements[64] = "";
    uint32_t n = 0u;
    uint32_t i = 0u;
    int32_t  k = 0;

    (void)strcat(elements, g_star);
    (void)strcat(elements, "N");
    (void)strcat(elements, g_pat[code]);
    (void)strcat(elements, "N");
    (void)strcat(elements, g_star);

    for (k = 0; k < LEAD_MS; k++)
    {
        g_level[n] = 0u;
        n++;
    }

    for (i = 0u; i < strlen(elements); i++)
    {
        bool    b_bar = (0u == (i % 2u));
        int32_t w_ms  = ('W' == elements[i]) ? (3 * narrow_ms) : narrow_ms;
        int32_t shift = (offset_pct * narrow_ms) / 100;

        w_ms += b_bar ? shift : -shift;
        if (w_ms < 1)
        {
            w_ms = 1;
        }

        for (k = 0; k < w_ms; k++)
        {
            bool b_level = b_bar;

            if ((k < bounce_ms) && next_bit())
            {
                b_level = !b_level;
            }
            g_level[n] = b_level ? 1u : 0u;
            n++;
        }
    }

    for (k = 0; k < TRAIL_MS; k++)
    {
        g_level[n] = 0u;
        n++;
    }

    return (n);
}

/*!
 * @brief Run one scan through filter, slicer and decoder.
 *
 * @return true if the decoder returned the expected command, not reversed.
 */
static bool
run_case (int32_t code, int32_t narrow_ms, int32_t offset_pct,
          int32_t bounce_ms)
{
    static bc_slicer_t g_slicer;
    static ir_filter_t g_filter;
    uint32_t           edges[MAX_EDGES];
    uint32_t           num_edges = 0u;
    uint32_t           count     = build_signal(code, narrow_ms, offset_pct,
                                                bounce_ms);
    uint32_t           i         = 0u;
    uint32_t           t_us      = 0u;
    uint8_t            cmd       = 0u;
    bool               b_rev     = false;
    bc_slicer_edge_t   edge;
    bc_decode_status_t status;

    bc_slicer_init(&g_slicer);
    ir_filter_init(&g_filter);

    for (i = 0u; i < count; i++)
    {
        int32_t dark = ir_filter_push(&g_filter,
                           (0u != g_level[i]) ? LB_BC_GPIO_DARK_COUNT
                                              : LB_BC_GPIO_LIGHT_COUNT);

        t_us += 1000u;
        if (bc_slicer_step(&g_slicer, dark, t_us, &edge)
            && (num_edges < MAX_EDGES))
        {
            edges[num_edges] = edge.time_us;
            num_edges++;
        }
    }

    status = bc_decode_scan(edges, num_edges, &cmd, &b_rev);

    return ((BC_DECODE_OK == status) && (cmd == (uint8_t)g_names[code])
            && !b_rev);
}

/*!
 * @brief Run all codes and speeds for one fault setting.
 *
 * @return Number of passing cases out of NUM_CODES * NUM_SPEEDS.
 */
static int32_t
run_set (int32_t offset_pct, int32_t bounce_ms)
{
    int32_t passed = 0;
    int32_t code   = 0;
    int32_t speed  = 0;

    for (code = 0; code < NUM_CODES; code++)
    {
        for (speed = 0; speed < NUM_SPEEDS; speed++)
        {
            if (run_case(code, g_narrow_ms[speed], offset_pct, bounce_ms))
            {
                passed++;
            }
        }
    }

    return (passed);
}

int
main (void)
{
    int32_t const total = NUM_CODES * NUM_SPEEDS;
    int32_t       fails = 0;
    int32_t       ok    = 0;

    /* name, offset in percent of a narrow element, bounce samples */
    struct { char const * name; int32_t offset; int32_t bounce; } const
    cases[] =
    {
        { "ideal level",                  0, 0 },
        { "bars 20 % too wide",          20, 0 },
        { "bars 40 % too wide",          40, 0 },
        { "bars 40 % too narrow",       -40, 0 },
        { "bounce 1 sample at each edge", 0, 1 },
        { "bounce 2 samples at each edge", 0, 2 },
        { "20 % wide + bounce 2",        20, 2 },
    };
    size_t i = 0u;

    for (i = 0u; i < (sizeof(cases) / sizeof(cases[0])); i++)
    {
        ok = run_set(cases[i].offset, cases[i].bounce);
        printf("[TEST] digital  %-32s %2d/%d %s\n", cases[i].name,
               (int)ok, (int)total, (ok == total) ? "PASS" : "FAIL");
        if (ok != total)
        {
            fails++;
        }
    }

    printf("[TEST] digital SUMMARY %d failed\n", (int)fails);

    return ((0 == fails) ? 0 : 1);
}

/*** end of file ***/
