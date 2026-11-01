/** @file line_barcode_test.c
 *
 * @brief On-target tests of the line and barcode subsystem.  Built only with
 *        "make TEST=1".  Results go to testing/line_barcode/TEST_LOG.md.
 *
 * @par
 * What runs here (the pure logic is tested on a PC, see
 * testing/line_barcode/lb_host_tests.c):
 *  - S2-U09 the sampler runs at 1 kHz            (NFR-04)
 *  - S2-U10 the line task runs every 10 ms       (NFR-03)
 *  - S2-T03 the sampler fits its time budget
 *  - S2-T04 the sensors deliver plausible values (ADC for centre and right,
 *           GPIO level for the barcode sensor)
 *  - INT-01 30 edges injected into mbf_edge are decoded and no edge is lost
 *
 * @par
 * Optional helpers, switched on in lb_config.h:
 *  - LB_TEST_CALIBRATE     guided calibration (white, line, barcode ink)
 *  - LB_TEST_TRACE_SECONDS live trace for tuning while the car drives
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stdint.h>
#include <stdbool.h>
#include "line_barcode.h"
#include "ir_driver.h"
#include "ir_calib.h"
#include "../test_report.h"

#if APP_TEST

/* Edge timing of the injected code: one narrow unit of 50 ms is a car speed
 * of 60 mm/s over 3 mm bars. */
#define NARROW_US       50000u
#define WIDE_US         (3u * NARROW_US)
#define SAMPLE_WINDOW_MS    1000

/* The sampler must finish well inside its 1 ms period. */
#define SAMPLER_BUDGET_US   200u

/* Private function prototypes. */
static void make_code(char const * p_data, uint32_t * p_edge_us);
static void inject_code(char const * p_data);
#if (LB_TEST_CALIBRATE != 0)
static void run_guided_calibration(void);
static void print_probe(char const * p_name, ir_calib_probe_t const * p_probe);
static void collect(uint32_t channel, int32_t seconds,
                    ir_calib_probe_t * p_probe);
#endif
#if (LB_TEST_TRACE_SECONDS > 0)
static void run_trace(void);
#endif

/*!
 * @brief Build the 30 edge times of "*X*" at the injected speed.
 *
 * @param[in]  p_data     Nine letters N/W: the pattern of the middle char.
 * @param[out] p_edge_us  Room for 30 edge times.
 */
static void
make_code (char const * p_data, uint32_t * p_edge_us)
{
    char const * p_part[3] = { "NWNNWNWNN", NULL, "NWNNWNWNN" };
    uint32_t     now  = 1000u;
    uint32_t     num  = 0u;
    uint32_t     part = 0u;
    uint32_t     elem = 0u;

    p_part[1] = p_data;
    p_edge_us[num++] = now;                     /* first bar begins */

    for (part = 0u; part < 3u; part++)
    {
        for (elem = 0u; elem < 9u; elem++)
        {
            now += ('W' == p_part[part][elem]) ? WIDE_US : NARROW_US;
            p_edge_us[num++] = now;
        }

        if (part < 2u)
        {
            now += NARROW_US;                   /* inter-character gap */
            p_edge_us[num++] = now;
        }
    }

    return;
}

/*!
 * @brief Put the edges of "*X*" on mbf_edge, as the sampler would.
 */
static void
inject_code (char const * p_data)
{
    uint32_t edge_us[30];
    MSG_EDGE msg;
    uint32_t idx = 0u;

    make_code(p_data, edge_us);

    for (idx = 0u; idx < 30u; idx++)
    {
        /* Casts: the kernel's UW and UB are unsigned 32 and 8 bit. */
        msg.t_us = (UW)edge_us[idx];
        msg.dark = (UB)(0u == (idx % 2u));      /* bars start dark */
        (void)MBF_SEND(mbf_edge, msg);
    }

    return;
}

void
line_barcode_test (void)
{
    uint32_t start_runs   = 0u;
    uint32_t start_ticks  = 0u;
    uint32_t start_decoded = 0u;
    uint32_t start_drops  = 0u;
    int32_t  dark_c       = 0;
    int32_t  dark_r       = 0;
    int32_t  dark_b       = 0;
    bool     b_alive      = false;

    TEST_INIT();

#if (LB_TEST_CALIBRATE != 0)
    run_guided_calibration();
#endif

    /* S2-U09: the sampler runs at 1 kHz (about 1000 runs in 1 s). */
    start_runs = g_lb_stats.sampler_runs;
    (void)tk_dly_tsk(SAMPLE_WINDOW_MS);
    TEST_CHECK("S2-U09",
               ((g_lb_stats.sampler_runs - start_runs) >= 950u)
               && ((g_lb_stats.sampler_runs - start_runs) <= 1050u),
               "IR sampler runs at 1 kHz");

    /* S2-U10: the line task runs every 10 ms (about 100 ticks in 1 s). */
    start_ticks = g_lb_stats.line_ticks;
    (void)tk_dly_tsk(SAMPLE_WINDOW_MS);
    TEST_CHECK("S2-U10",
               ((g_lb_stats.line_ticks - start_ticks) >= 95u)
               && ((g_lb_stats.line_ticks - start_ticks) <= 105u),
               "line task runs every 10 ms");

    /* S2-T03: the longest sampler run stays far below the 1 ms period. */
    TEST_CHECK("S2-T03", g_lb_stats.sampler_max_us < SAMPLER_BUDGET_US,
               "sampler run time is below 200 us");
    tm_printf((UB*)"       (longest sampler run: %d us)\n",
              (INT)g_lb_stats.sampler_max_us);

    /* S2-T04: the sensors are alive.  A converter or wiring fault shows as
     * every sensor stuck at 0 or at 4095.  The barcode sensor is digital and
     * reports one of two fixed counts, so it never reads exactly 0 or 4095. */
    dark_c = ir_driver_get_dark(IR_CHANNEL_CENTRE);
    dark_r = ir_driver_get_dark(IR_CHANNEL_RIGHT);
    dark_b = ir_driver_get_dark(IR_CHANNEL_BARCODE);
    b_alive = ((dark_c > 20) && (dark_c < 4075))
              || ((dark_r > 20) && (dark_r < 4075));
#if (LB_BC_USE_GPIO_LEVEL == 0)
    /* Only in analogue mode: the digital level is always one of the two
     * fixed counts, so including it would make this check always pass. */
    b_alive = b_alive || ((dark_b > 20) && (dark_b < 4075));
#endif
    TEST_CHECK("S2-T04", b_alive, "sensor readings are not stuck at 0 or 4095");
    tm_printf((UB*)"       (dark scale now: centre %d, right %d, barcode %d)\n",
              (INT)dark_c, (INT)dark_r, (INT)dark_b);

    /* INT-01 (IF-02): 30 edges injected into mbf_edge are decoded by the
     * barcode task.  Keep the barcode sensor over white paper meanwhile. */
    start_decoded = g_lb_stats.bc_decoded;
    start_drops   = g_lb_stats.edges_dropped;
    inject_code("NNWNNWNNW");                   /* the command 'B' */
    (void)tk_dly_tsk(300);
    TEST_CHECK("INT-01",
               (g_lb_stats.bc_decoded == (start_decoded + 1u))
               && ('B' == g_lb_stats.bc_last),
               "30 edges on mbf_edge -> barcode task decodes 'B'");
    TEST_CHECK("INT-01b", g_lb_stats.edges_dropped == start_drops,
               "no edge was lost between the sampler and the barcode task");

    /* A code that is not a command must be reported as a failure. */
    start_decoded = g_lb_stats.bc_decoded;
    inject_code("WNNNWWNNN");                   /* the character 'F' */
    (void)tk_dly_tsk(300);
    TEST_CHECK("INT-01c", g_lb_stats.bc_decoded == start_decoded,
               "a valid Code 39 character that is not A-D is not a command");

    TEST_SUMMARY("line_barcode");

#if (LB_TEST_TRACE_SECONDS > 0)
    run_trace();
#endif

    return;
}

#if (LB_TEST_CALIBRATE != 0)
/* A still sensor stays within about 20 counts; far more means it moved. */
#define CAL_NOISE_MAX   200

/*!
 * @brief Collect statistics of one sensor for a number of seconds.
 */
static void
collect (uint32_t channel, int32_t seconds, ir_calib_probe_t * p_probe)
{
    int32_t ms = 0;

    ir_calib_probe_init(p_probe);

    for (ms = 0; ms < (seconds * 1000); ms++)
    {
        ir_calib_probe_push(p_probe, ir_driver_get_dark(channel));
        (void)tk_dly_tsk(1);
    }

    return;
}

static void
print_probe (char const * p_name, ir_calib_probe_t const * p_probe)
{
    tm_printf((UB*)"  %s: mean %d, noise (max-min) %d\n", p_name,
              (INT)ir_calib_probe_mean(p_probe),
              (INT)ir_calib_probe_spread(p_probe));

    return;
}

/*!
 * @brief Guided calibration.  The console tells the person what to do and
 *        prints the constants to paste into lb_config.h.
 */
static void
run_guided_calibration (void)
{
    ir_calib_probe_t white_c;
    ir_calib_probe_t white_r;
    ir_calib_probe_t white_b;
    ir_calib_probe_t line_c;
    ir_calib_probe_t line_r;
    ir_calib_probe_t ink_b;

    tm_printf((UB*)"\n=== CALIBRATION (about 50 s) ===\n");
    tm_printf((UB*)"Step 1/4: put the car so ALL THREE sensors\n");
    tm_printf((UB*)"are over bare white paper. Measuring in 5 s ...\n");
    (void)tk_dly_tsk(5000);
    collect(IR_CHANNEL_CENTRE, 2, &white_c);
    collect(IR_CHANNEL_RIGHT, 2, &white_r);
    collect(IR_CHANNEL_BARCODE, 2, &white_b);

    /* One sensor at a time: at the working spacing a plain line cannot fill
     * both spots at once. */
    tm_printf((UB*)"Step 2/4: CENTRE sensor over the middle of the line.\n");
    tm_printf((UB*)"Then hands off. Measuring in 10 s ...\n");
    (void)tk_dly_tsk(10000);
    collect(IR_CHANNEL_CENTRE, 2, &line_c);

    tm_printf((UB*)"Step 3/4: RIGHT sensor over the middle of the line.\n");
    tm_printf((UB*)"Then hands off. Measuring in 10 s ...\n");
    (void)tk_dly_tsk(10000);
    collect(IR_CHANNEL_RIGHT, 2, &line_r);

    tm_printf((UB*)"Step 4/4: BARCODE sensor over a black barcode bar\n");
    tm_printf((UB*)"(its DO output must flip; check the module LED).\n");
    tm_printf((UB*)"Measuring in 8 s ...\n");
    (void)tk_dly_tsk(8000);
    collect(IR_CHANNEL_BARCODE, 2, &ink_b);

    tm_printf((UB*)"\nMeasured (dark scale, higher = darker):\n");
    print_probe("centre white", &white_c);
    print_probe("centre line ", &line_c);
    print_probe("right  white", &white_r);
    print_probe("right  line ", &line_r);
    print_probe("barcode white", &white_b);
    print_probe("barcode ink  ", &ink_b);

    tm_printf((UB*)"\nPaste into app_program/line_barcode/lb_config.h:\n");
    tm_printf((UB*)"  #define LB_WHITE_CENTER %d\n",
              (INT)ir_calib_probe_mean(&white_c));
    tm_printf((UB*)"  #define LB_BLACK_CENTER %d\n",
              (INT)ir_calib_probe_mean(&line_c));
    tm_printf((UB*)"  #define LB_WHITE_RIGHT  %d\n",
              (INT)ir_calib_probe_mean(&white_r));
    tm_printf((UB*)"  #define LB_BLACK_RIGHT  %d\n",
              (INT)ir_calib_probe_mean(&line_r));
#if (LB_BC_USE_GPIO_LEVEL == 0)
    tm_printf((UB*)"  #define LB_BC_WHITE_NOMINAL %d\n",
              (INT)ir_calib_probe_mean(&white_b));
    tm_printf((UB*)"  #define LB_BC_BLACK_NOMINAL %d\n",
              (INT)ir_calib_probe_mean(&ink_b));
#else
    /* The barcode sensor is digital: nothing to paste.  The mean shows
     * which of the two fixed counts the DO level maps to. */
    tm_printf((UB*)"  (barcode sensor is digital: no constants to paste;\n");
    tm_printf((UB*)"   white should read %d and ink %d. If they are\n",
              (INT)LB_BC_GPIO_LIGHT_COUNT, (INT)LB_BC_GPIO_DARK_COUNT);
    tm_printf((UB*)"   swapped, flip LB_BC_GPIO_DARK_IS_HIGH.)\n");
#endif

    if ((ir_calib_probe_spread(&line_c) > CAL_NOISE_MAX)
        || (ir_calib_probe_spread(&line_r) > CAL_NOISE_MAX))
    {
        tm_printf((UB*)"WARNING: a sensor moved while measured on the\n");
        tm_printf((UB*)"line (noise above %d). Calibrate again.\n",
                  (INT)CAL_NOISE_MAX);
    }
    else
    {
        /* The car stayed still on the line. */
    }

    if (ir_calib_probe_mean(&line_c) <= ir_calib_probe_mean(&white_c))
    {
        tm_printf((UB*)"WARNING: black reads LOWER than white. Flip\n");
        tm_printf((UB*)"LB_IR_DARK_IS_HIGH in lb_config.h, calibrate again.\n");
    }
    else
    {
        tm_printf((UB*)"Polarity looks right (black reads higher).\n");
    }

    tm_printf((UB*)"Rule: the black-minus-white span of each sensor must be\n");
    tm_printf((UB*)"at least %d counts, and many times the noise above.\n",
              (INT)LB_CAL_SPAN_MIN);
    tm_printf((UB*)"=== END OF CALIBRATION ===\n\n");

    return;
}
#endif /* LB_TEST_CALIBRATE */

#if (LB_TEST_TRACE_SECONDS > 0)
/*!
 * @brief Print one status line every 100 ms.  Save it with "screen -L" and
 *        plot it, or read it while pushing the car by hand.
 *
 * Columns: time(ms) dark-centre dark-right dark-barcode barcode-edges
 *          decoded failed last
 */
static void
run_trace (void)
{
    int32_t step = 0;

    tm_printf((UB*)"TRACE t_ms darkC darkR bcDark bcEdges ok fail last\n");

    for (step = 0; step < (LB_TEST_TRACE_SECONDS * 10); step++)
    {
        tm_printf((UB*)"TRACE %d %d %d %d %d %d %d %c\n",
                  (INT)(step * 100),
                  (INT)g_lb_stats.line_dark_centre,
                  (INT)g_lb_stats.line_dark_right,
                  (INT)ir_driver_get_dark(IR_CHANNEL_BARCODE),
                  (INT)g_lb_stats.bc_edges,
                  (INT)g_lb_stats.bc_decoded,
                  (INT)g_lb_stats.bc_failed,
                  (0u != g_lb_stats.bc_last) ? (INT)g_lb_stats.bc_last : '-');
        (void)tk_dly_tsk(100);
    }

    return;
}
#endif /* LB_TEST_TRACE_SECONDS */

#endif /* APP_TEST */

/*** end of file ***/
