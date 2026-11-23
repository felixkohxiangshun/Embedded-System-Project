/** @file lb_host_tests.c
 *
 * @brief PC test runner for the line and barcode subsystem (Buddy 3).
 *
 * @par
 * Runs the REAL firmware modules (no copies) on a PC:
 *  - bc_decode, bc_stream and bc_slicer with synthetic edge lists and with
 *    the optics simulation (lb_sim_bc.c),
 *  - ir_filter, ir_calib, line_ctrl and line_recov with hand-made inputs,
 *  - the closed-loop line simulation (lb_sim_line.c) for the whole chain.
 *
 * @par
 * Build and run: cd testing/line_barcode && make run
 * Exit status is 0 only if every check passed.  Test IDs match TEST_LOG.md.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "lb_config.h"
#include "bc_decode.h"
#include "bc_stream.h"
#include "bc_slicer.h"
#include "bc_level.h"
#include "ir_calib.h"
#include "ir_filter.h"
#include "line_ctrl.h"
#include "line_recov.h"
#include "lb_sim.h"

static uint32_t g_pass = 0u;
static uint32_t g_fail = 0u;

#define CHECK(id, cond, text)                                              \
    do                                                                     \
    {                                                                      \
        if (cond)                                                          \
        {                                                                  \
            g_pass++;                                                      \
            printf("[TEST] %s PASS  %s\n", (id), (text));                  \
        }                                                                  \
        else                                                               \
        {                                                                  \
            g_fail++;                                                      \
            printf("[TEST] %s FAIL  %s   (line %d)\n", (id), (text),       \
                   __LINE__);                                              \
        }                                                                  \
    } while (0)

/* ---------------------------------------------------------------------
 * Edge list builder: the 30 edges of "*X*" for a given narrow unit.
 * --------------------------------------------------------------------- */

#define NUM_EDGES   30u

/* Widths of the 29 elements of "*X*" in narrow units (wide = ratio). */
static void
element_widths (char symbol, double ratio, double * p_width)
{
    char const * p_pat[3];
    uint32_t     part = 0u;
    uint32_t     elem = 0u;
    uint32_t     n = 0u;

    p_pat[0] = lb_sim_pattern('*');
    p_pat[1] = lb_sim_pattern(symbol);
    p_pat[2] = lb_sim_pattern('*');

    for (part = 0u; part < 3u; part++)
    {
        for (elem = 0u; elem < 9u; elem++)
        {
            p_width[n++] = ('W' == p_pat[part][elem]) ? ratio : 1.0;
        }

        if (part < 2u)
        {
            p_width[n++] = 1.0;     /* inter-character gap */
        }
    }
}

static void
make_edges (char symbol, double unit_us, uint32_t t0, uint32_t * p_edge)
{
    double   width[29];
    double   now = 0.0;
    uint32_t idx = 0u;

    element_widths(symbol, 3.0, width);
    p_edge[0] = t0;

    for (idx = 0u; idx < 29u; idx++)
    {
        now += width[idx] * unit_us;
        p_edge[idx + 1u] = t0 + (uint32_t)(now + 0.5);
    }
}

static bc_decode_status_t
decode_symbol (char symbol, double unit_us, uint8_t * p_cmd, bool * p_rev)
{
    uint32_t edge[NUM_EDGES];

    make_edges(symbol, unit_us, 1000u, edge);

    return (bc_decode_edges(edge, NUM_EDGES, p_cmd, p_rev));
}

/* ---------------------------------------------------------------------
 * Tests: decoder
 * --------------------------------------------------------------------- */

static void
test_decoder (void)
{
    uint8_t  cmd = 0u;
    bool     rev = false;
    uint32_t edge[NUM_EDGES];
    uint32_t idx = 0u;
    char     ch = 0;
    bool     b_all = true;
    char     id[16];

    CHECK("S2-U01", (BC_DECODE_OK == decode_symbol('A', 50000.0, &cmd, &rev))
                    && ('A' == cmd) && (!rev), "decode *A*");
    CHECK("S2-U02", (BC_DECODE_OK == decode_symbol('B', 50000.0, &cmd, &rev))
                    && ('B' == cmd), "decode *B*");
    CHECK("S2-U03", (BC_DECODE_OK == decode_symbol('C', 50000.0, &cmd, &rev))
                    && ('C' == cmd), "decode *C*");
    CHECK("S2-U04", (BC_DECODE_OK == decode_symbol('D', 50000.0, &cmd, &rev))
                    && ('D' == cmd), "decode *D*");

    /* Speed independence: unit from 3 ms (very fast) to 200 ms (crawling). */
    b_all = true;
    for (idx = 0u; idx < 6u; idx++)
    {
        static double const unit[6] = {3000.0, 12500.0, 25000.0, 50000.0,
                                       100000.0, 200000.0};

        b_all = b_all && (BC_DECODE_OK == decode_symbol('C', unit[idx],
                                                        &cmd, &rev))
                && ('C' == cmd);
    }
    CHECK("S2-U05", b_all, "same code decodes at 3 ms to 200 ms per unit");

    /* Rejects: never decoded wrongly. */
    make_edges('A', 50000.0, 1000u, edge);
    CHECK("S2-U06a", BC_DECODE_ERR_COUNT == bc_decode_edges(edge, 29u, &cmd,
                                                            &rev),
          "29 edges are rejected");
    edge[5] = edge[4] + 5000u;      /* squash one element to 0.1 unit */
    CHECK("S2-U06b", BC_DECODE_OK != bc_decode_edges(edge, NUM_EDGES, &cmd,
                                                     &rev),
          "a squashed element is rejected");
    CHECK("S2-U06c", BC_DECODE_ERR_COMMAND == decode_symbol('*', 50000.0, &cmd, &rev),
          "a valid frame with '*' in the middle does not give a command");
    make_edges('A', 50000.0, 1000u, edge);
    edge[10] += 120000u;            /* inter-character gap far too long */
    CHECK("S2-U06d", BC_DECODE_OK != bc_decode_edges(edge, NUM_EDGES, &cmd,
                                                     &rev),
          "a wrong inter-character gap is rejected");

    /* Every letter A to Z decodes, in both directions (the demo's
     * mapping can use any four letters). */
    b_all = true;
    for (ch = 'A'; ch <= 'Z'; ch++)
    {
        b_all = b_all
                && (BC_DECODE_OK == decode_symbol(ch, 50000.0, &cmd, &rev))
                && ((uint8_t)ch == cmd) && (!rev);
    }
    CHECK("S2-U30", b_all, "every letter A to Z decodes");

    b_all = true;
    for (ch = 'A'; ch <= 'Z'; ch++)
    {
        uint32_t back[NUM_EDGES];

        make_edges(ch, 50000.0, 1000u, edge);
        for (idx = 0u; idx < NUM_EDGES; idx++)
        {
            /* Read from the other end: last edge first, times mirrored. */
            back[idx] = 3000000u - edge[NUM_EDGES - 1u - idx];
        }
        b_all = b_all
                && (BC_DECODE_OK == bc_decode_edges(back, NUM_EDGES, &cmd, &rev))
                && ((uint8_t)ch == cmd) && rev;
    }
    CHECK("S2-U31", b_all, "every letter A to Z decodes when read backwards");

    /* Timer wrap in the middle of a code. */
    make_edges('B', 50000.0, 0xFFFFFFFFu - 400000u, edge);
    CHECK("S2-U11", (BC_DECODE_OK == bc_decode_edges(edge, NUM_EDGES, &cmd,
                                                     &rev)) && ('B' == cmd),
          "timer wrap inside a code is harmless");

    /* Reversed read is detected. */
    make_edges('A', 50000.0, 1000u, edge);
    for (idx = 0u; idx < 15u; idx++)
    {
        uint32_t tmp = edge[idx];

        edge[idx] = edge[NUM_EDGES - 1u - idx];
        edge[NUM_EDGES - 1u - idx] = tmp;
    }
    /* Mirror the times so they increase again. */
    for (idx = 0u; idx < NUM_EDGES; idx++)
    {
        edge[idx] = 2000000u - edge[idx];
    }
    snprintf(id, sizeof(id), "S2-U14");
    CHECK(id, (BC_DECODE_OK == bc_decode_edges(edge, NUM_EDGES, &cmd, &rev))
              && rev, "a code driven over backwards is flagged as reversed");
}

/* Timing noise: every edge moves by up to +-pct of a unit. */
static void
test_decoder_noise (void)
{
    uint32_t edge[NUM_EDGES];
    uint8_t  cmd = 0u;
    bool     rev = false;
    uint32_t seed = 12345u;
    uint32_t trial = 0u;
    uint32_t idx = 0u;
    uint32_t ok = 0u;
    uint32_t wrong = 0u;
    char const syms[4] = {'A', 'B', 'C', 'D'};

    for (trial = 0u; trial < 400u; trial++)
    {
        make_edges(syms[trial % 4u], 50000.0, 1000u, edge);

        for (idx = 1u; idx < NUM_EDGES; idx++)
        {
            seed = (seed * 1664525u) + 1013904223u;
            /* +-0.30 unit uniform shift: a 3 mm bar edge off by 0.9 mm */
            edge[idx] = (uint32_t)((int32_t)edge[idx]
                                   + ((int32_t)((seed >> 8) % 30001u)
                                      - 15000));
        }

        if (BC_DECODE_OK == bc_decode_edges(edge, NUM_EDGES, &cmd, &rev))
        {
            if ((uint8_t)syms[trial % 4u] == cmd)
            {
                ok++;
            }
            else
            {
                wrong++;
            }
        }
    }

    printf("       (edge timing noise +-0.3 unit: %u of 400 correct, %u wrong)\n",
           ok, wrong);
    CHECK("S2-U15", (0u == wrong) && (ok >= 380u),
          "edges off by +-0.3 unit: >= 95 percent read, none read wrong");
}

static void
test_stream (void)
{
    bc_stream_t stream;
    uint32_t    edge[NUM_EDGES];
    uint32_t    idx = 0u;
    uint8_t     cmd = 0u;
    bool        rev = false;
    uint32_t    seen = 0u;
    uint32_t    full = 0u;
    bc_stream_event_t ev;

    bc_stream_reset(&stream);
    make_edges('D', 40000.0, 5000u, edge);

    for (idx = 0u; idx < NUM_EDGES; idx++)
    {
        ev = bc_stream_push(&stream, edge[idx], 0u == (idx % 2u));
        if (BC_STREAM_EVENT_SEEN == ev) { seen++; }
        if (BC_STREAM_EVENT_FULL == ev) { full++; }
    }

    CHECK("S2-U16a", (1u == seen) && (1u == full),
          "stream announces SEEN once and FULL at 30 edges");
    CHECK("S2-U16b", (BC_DECODE_OK == bc_stream_decode(&stream, &cmd, &rev))
                     && ('D' == cmd), "stream decodes *D*");

    /* Stray edges before the code (dust, the line's own edge). */
    bc_stream_reset(&stream);
    (void)bc_stream_push(&stream, 100u, true);
    (void)bc_stream_push(&stream, 4000u, false);
    for (idx = 0u; idx < NUM_EDGES; idx++)
    {
        (void)bc_stream_push(&stream, edge[idx], 0u == (idx % 2u));
    }
    CHECK("S2-U16c", (BC_DECODE_OK == bc_stream_decode(&stream, &cmd, &rev))
                     && ('D' == cmd),
          "two stray edges in front of the code do not matter");

    /* Light edges are ignored while idle. */
    bc_stream_reset(&stream);
    (void)bc_stream_push(&stream, 100u, false);
    CHECK("S2-U16d", !bc_stream_is_active(&stream),
          "a light edge does not start a capture");

    /* A complete capture with one narrow space squeezed to 0.3 x (the DO
     * threshold widens the bars): the glitch filter would delete it. */
    make_edges('K', 40000.0, 5000u, edge);
    edge[4] = edge[3] + 12000u;     /* 2nd narrow space of the "*" */
    bc_stream_reset(&stream);
    for (idx = 0u; idx < NUM_EDGES; idx++)
    {
        (void)bc_stream_push(&stream, edge[idx], 0u == (idx % 2u));
    }
    {
        /* Widths from a real hand swipe (2026-11-07, ms): narrow bars read
         * far too thin, which the old narrow-bar speed check rejected. */
        static double const b[15] = { 0.4, 1.8, 16.8, 18.4, 3.3, 2.9, 22.6,
                                      18.8, 6.9, 7.8, 5.8, 5.7, 19.1, 19.6,
                                      1.8 };
        static double const s[14] = { 20.7, 8.1, 6.7, 7.6, 8.5, 7.5, 5.3,
                                      18.8, 5.0, 5.6, 21.3, 5.6, 4.9, 6.9 };
        uint32_t real[NUM_EDGES];
        double   now = 1000.0;

        real[0] = 1000u;
        for (idx = 0u; idx < 29u; idx++)
        {
            now += 1000.0 * ((0u == (idx % 2u)) ? b[idx / 2u] : s[idx / 2u]);
            real[idx + 1u] = (uint32_t)now;
        }
        CHECK("S2-U32", BC_DECODE_OK == bc_decode_edges(real, NUM_EDGES,
                                                        &cmd, &rev),
              "real swipe with thin narrow bars decodes (speed by char time)");
    }
    CHECK("S2-U16e", (BC_DECODE_OK == bc_stream_decode(&stream, &cmd, &rev))
                     && ('K' == cmd),
          "a 30-edge capture is decoded before glitches are merged");
}

/* DO debouncer (digital mode): edges from the GPIO interrupt. */
static void
test_level (void)
{
    bc_level_t       lv;
    bc_slicer_edge_t e;
    uint32_t         n_edges = 0u;
    uint32_t         t_dark  = 0u;
    uint32_t         t_light = 0u;
    bool             b_ok    = true;

    /* A clean 1.3 ms bar (the narrowest seen at driving speed). */
    bc_level_init(&lv, false, 0u);
    b_ok &= !bc_level_change(&lv, 10000u, true, &e);
    if (bc_level_change(&lv, 11300u, false, &e)) { n_edges++; t_dark = e.time_us; b_ok &= e.b_dark; }
    if (bc_level_poll(&lv, 11400u, &e)) { b_ok = false; }      /* too soon */
    if (bc_level_poll(&lv, 11600u, &e)) { n_edges++; t_light = e.time_us; b_ok &= !e.b_dark; }
    CHECK("S2-U33a", b_ok && (2u == n_edges) && (10000u == t_dark)
                     && (11300u == t_light),
          "DO: a 1.3 ms bar gives two edges at the exact interrupt times");

    /* Chatter: three extra toggles within 60 us at each edge. */
    bc_level_init(&lv, false, 0u);
    n_edges = 0u;
    {
        static uint32_t const t[8] = { 20000u, 20020u, 20040u, 20060u,
                                       23000u, 23015u, 23030u, 23045u };
        static bool const d[8] = { true, false, true, true,
                                   false, true, false, false };
        uint32_t k = 0u;

        for (k = 0u; k < 8u; k++)
        {
            if (bc_level_change(&lv, t[k], d[k], &e)) { n_edges++; t_dark = e.time_us; }
        }
        if (bc_level_poll(&lv, 24000u, &e)) { n_edges++; t_light = e.time_us; }
    }
    CHECK("S2-U33b", (2u == n_edges) && (20040u == t_dark)
                     && (23030u == t_light),
          "DO: chatter at both edges of a bar is reduced to two edges");

    /* A 100 us speck on white paper is ignored. */
    bc_level_init(&lv, false, 0u);
    b_ok = !bc_level_change(&lv, 5000u, true, &e)
           && !bc_level_change(&lv, 5100u, false, &e)
           && !bc_level_poll(&lv, 9000u, &e);
    CHECK("S2-U33c", b_ok, "DO: a 100 us speck is not an edge");

    /* Timer wrap between the two edges. */
    bc_level_init(&lv, false, 0u);
    (void)bc_level_change(&lv, 0xFFFFFF00u, true, &e);
    b_ok = bc_level_change(&lv, 0x00000800u, false, &e) && e.b_dark
           && (0xFFFFFF00u == e.time_us);
    CHECK("S2-U33d", b_ok, "DO: timer wrap inside a bar is harmless");
}

static void
test_glitch (void)
{
    uint32_t edge[NUM_EDGES + 2u];
    uint32_t idx = 0u;
    uint32_t num = 0u;
    uint8_t  cmd = 0u;
    bool     rev = false;
    uint32_t base[NUM_EDGES];

    make_edges('B', 50000.0, 1000u, base);

    /* Insert a 1 ms dust speck inside the first wide space. */
    for (idx = 0u; idx < 2u; idx++)
    {
        edge[idx] = base[idx];
    }
    edge[2] = base[1] + 40000u;
    edge[3] = edge[2] + 1000u;
    for (idx = 2u; idx < NUM_EDGES; idx++)
    {
        edge[idx + 2u] = base[idx];
    }

    num = bc_decode_merge_glitches(edge, NUM_EDGES + 2u);
    CHECK("S2-U17", (NUM_EDGES == num)
                    && (BC_DECODE_OK == bc_decode_edges(edge, num, &cmd, &rev))
                    && ('B' == cmd), "a 1 ms speck inside an element is merged");
}

/* ---------------------------------------------------------------------
 * Tests: slicer in the optics simulation
 * --------------------------------------------------------------------- */

static void
test_optics (void)
{
    static double const speed[4]    = {60.0, 90.0, 120.0, 160.0};
    static double const spot[3]     = {3.0, 4.0, 5.0};
    static double const contrast[3] = {1.0, 0.8, 0.7};
    char const syms[4] = {'A', 'B', 'C', 'D'};
    uint32_t total = 0u;
    uint32_t good = 0u;
    uint32_t bad = 0u;
    uint32_t a = 0u, b = 0u, c = 0u, n = 0u;
    lb_sim_bc_cfg_t cfg;

    lb_sim_seed(1u);

    for (a = 0u; a < 4u; a++)
    {
        for (b = 0u; b < 3u; b++)
        {
            for (c = 0u; c < 3u; c++)
            {
                cfg.speed_mmps    = speed[a];
                cfg.speed_wobble  = 0.15;
                cfg.spot_mm       = spot[b];
                cfg.contrast      = contrast[c];
                cfg.noise_counts  = 25.0;
                cfg.flicker       = 0.04;
                cfg.ambient_shift = 150.0;
                cfg.wide_ratio    = 3.0;
                cfg.jitter_us     = 30.0;

                for (n = 0u; n < 40u; n++)
                {
                    lb_sim_bc_result_t r = lb_sim_bc_run(&cfg, syms[n % 4u]);

                    total++;
                    if (LB_SIM_BC_CORRECT == r) { good++; }
                    if (LB_SIM_BC_WRONG == r)   { bad++; }
                }
            }
        }
    }

    printf("       (optics: %u passes, %u correct, %u wrong)\n",
           total, good, bad);
    CHECK("S2-U18", 0u == bad,
          "optics sim: no pass is ever decoded to a wrong command");
    CHECK("S2-U19", (good * 100u) >= (total * 98u),
          "optics sim: >= 98 percent correct, 60-160 mm/s, spot 3-5 mm");

    /* Printed wide/narrow ratio 2.5 instead of 3. */
    cfg.speed_mmps = 90.0; cfg.spot_mm = 4.0; cfg.contrast = 1.0;
    cfg.wide_ratio = 2.5;
    good = 0u;
    for (n = 0u; n < 100u; n++)
    {
        if (LB_SIM_BC_CORRECT == lb_sim_bc_run(&cfg, syms[n % 4u])) { good++; }
    }
    CHECK("S2-U20", good >= 95u,
          "optics sim: a code printed with wide/narrow = 2.5 is read");
}

/* ---------------------------------------------------------------------
 * Tests: filter, calibration, control, recovery
 * --------------------------------------------------------------------- */

static void
test_filter_calib (void)
{
    ir_filter_t filt;
    ir_calib_t  cal = { 400, 3200 };
    int32_t     out = 0;
    int32_t     lo = 99999, hi = -99999;
    uint32_t    idx = 0u;

    ir_filter_init(&filt);
    for (idx = 0u; idx < 300u; idx++)
    {
        /* 100 Hz flicker of +-400 counts sampled at 1 kHz */
        int32_t in = 2000 + (((idx % 10u) < 5u) ? 400 : -400);

        out = ir_filter_push(&filt, in);
        if (idx > 20u)
        {
            if (out < lo) { lo = out; }
            if (out > hi) { hi = out; }
        }
    }
    CHECK("S2-U21", (hi - lo) <= 2, "10 ms average removes 100 Hz flicker");

    CHECK("S2-U12a", 0 == ir_calib_darkness(&cal, 100), "below white -> 0");
    CHECK("S2-U12b", 1000 == ir_calib_darkness(&cal, 4000),
          "above black -> 1000");
    CHECK("S2-U12c", (ir_calib_darkness(&cal, 1800) > 480)
                     && (ir_calib_darkness(&cal, 1800) < 520),
          "half way -> about 500");
}

/* A plain line, and the "one sensor dark" partner reading, in each mode:
 * edge mode sees half the line on both sensors; straddle mode sees white. */
#if (LB_LINE_STRADDLE != 0)
#define JCT_PLAIN   50
#define JCT_LOW     100
#else
#define JCT_PLAIN   600
#define JCT_LOW     500
#endif

static void
test_line_junction (void)
{
    line_junction_t jct;

    line_junction_init(&jct);
    CHECK("S2-U27a", !line_junction_step(&jct, 1000, 1000, 0)
                     && !line_junction_step(&jct, 1000, 1000, 50),
          "both dark from the start, no plain line seen: nothing reported");

    (void)line_junction_step(&jct, JCT_PLAIN, JCT_PLAIN, 100);
    CHECK("S2-U27b", !line_junction_step(&jct, 1000, 1000, 100)
                     && !line_junction_step(&jct, 1000, 1000, 107)
                     && !line_junction_step(&jct, JCT_PLAIN, JCT_PLAIN, 108),
          "both dark for only 7 mm is not a junction");

    CHECK("S2-U27c", !line_junction_step(&jct, 1000, JCT_LOW, 110)
                     && !line_junction_step(&jct, 1000, JCT_LOW, 150)
                     && !line_junction_step(&jct, JCT_LOW, 1000, 151)
                     && !line_junction_step(&jct, JCT_LOW, 1000, 190),
          "one sensor dark alone is not a junction, however far");

    (void)line_junction_step(&jct, 1000, 1000, 200);
    CHECK("S2-U27d", !line_junction_step(&jct, 1000, 1000, 207)
                     && line_junction_step(&jct, 1000, 1000, 208),
          "both dark for 8 mm: junction reported");

    CHECK("S2-U27e", !line_junction_step(&jct, 1000, 1000, 300),
          "the same junction is reported only once");

    (void)line_junction_step(&jct, JCT_PLAIN, JCT_PLAIN, 301);
    (void)line_junction_step(&jct, 1000, 1000, 400);
    CHECK("S2-U27f", line_junction_step(&jct, 1000, 1000, 408),
          "plain line in between: the next junction is reported");
}

static void
test_line_ctrl (void)
{
    line_ctrl_t     ctrl;
    line_ctrl_out_t out;
    uint32_t        tick = 0u;

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 1000, 0, &out);
    CHECK("S2-U07a", (out.err < 0) && out.b_seen,
          "line under the centre sensor -> error < 0 (steer left)");

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 0, 1000, &out);
    CHECK("S2-U07b", (out.err > 0) && out.b_seen,
          "line under the right sensor -> error > 0 (steer right)");

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 600, 600, &out);
    CHECK("S2-U07c", (out.err > -30) && (out.err < 30),
          "line half way between the sensors -> error about 0");

#if (LB_LINE_STRADDLE == 0) && (LB_LINE_ERR_DIFF != 0)
    /* The car's own reading (2026-11-09): centre on white, right a third on
     * the line.  The share formula gave +450 (steer +700) and zigzagged. */
    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 20, 350, &out);
    CHECK("S2-U07e", (out.err == 165) && out.b_seen,
          "difference error: a light touch on one sensor asks for little steering");
#endif

    line_ctrl_init(&ctrl);
    for (tick = 0u; tick < 5u; tick++)
    {
        line_ctrl_update(&ctrl, 5, 5, &out);
    }
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-U07d", !out.b_seen, "both sensors on paper -> line not seen");
#endif

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 0, 1000, &out);
    line_ctrl_update(&ctrl, 1000, 0, &out);
    CHECK("S2-U08", (out.steer >= -LB_STEER_MAX) && (out.steer <= LB_STEER_MAX),
          "steering stays inside +-STEER_MAX");

    /* Slew limit: a sudden full error cannot jump the steering. */
    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 0, 600, &out);
    line_ctrl_update(&ctrl, 600, 0, &out);
    CHECK("S2-U22", out.steer >= -(LB_STEER_SLEW_MAX + 400),
          "steering change per tick is limited");

    /* Faded line: ratio estimate is unchanged when the ink is half as dark. */
    {
        line_ctrl_out_t a, b;

        uint32_t t = 0u;

        line_ctrl_init(&ctrl);
        for (t = 0u; t < 6u; t++) { line_ctrl_update(&ctrl, 700, 300, &a); }
        line_ctrl_init(&ctrl);
        for (t = 0u; t < 6u; t++) { line_ctrl_update(&ctrl, 350, 150, &b); }
#if (LB_LINE_STRADDLE == 0) && (LB_LINE_ERR_DIFF == 0)
        CHECK("S2-U23", (a.err - b.err <= 5) && (b.err - a.err <= 5)
                        && b.b_seen && b.b_faded,
              "half the ink: same position error, flagged as faded");
#endif
    }
}

static void
test_line_recov (void)
{
    line_recov_t     rec;
    line_recov_in_t  in;
    line_recov_out_t out;
    uint32_t         tick = 0u;
    uint32_t         first_lost = 0u;
    uint32_t         first_sweep = 0u;
    uint32_t         first_timeout = 0u;
    bool             b_lost_seen = false;

    line_recov_init(&rec);
    in.b_seen = true; in.err = 20; in.steer = 100; in.dt_ms = 10;
    in.b_timeout_active = true;

    for (tick = 0u; tick < 30u; tick++)
    {
        line_recov_step(&rec, &in, &out);
    }
    CHECK("S2-U24a", (100 == out.steer) && (0u == out.flags)
                     && (LINE_RECOV_TRACK == out.mode),
          "line seen: steering passes through unchanged");

    in.b_seen = false;
    for (tick = 1u; tick <= 400u; tick++)
    {
        line_recov_step(&rec, &in, &out);
        if ((0u != (out.flags & LINE_RECOV_F_LOST)) && (!b_lost_seen))
        {
            b_lost_seen = true;
            first_lost = tick;
        }
        if ((LINE_RECOV_SWEEP == out.mode) && (0u == first_sweep))
        {
            first_sweep = tick;
        }
        if ((0u != (out.flags & LINE_RECOV_F_TIMEOUT)) && (0u == first_timeout))
        {
            first_timeout = tick;
        }
    }
    printf("       (line vanishes: lost flag tick %u, sweep tick %u, "
           "timeout tick %u; 1 tick = 10 ms)\n",
           first_lost, first_sweep, first_timeout);
    CHECK("S2-U24b", (first_lost >= 2u) && (first_lost <= 3u),
          "a single missing tick is bridged, the second reports LOST");
    CHECK("S2-U24c", (first_sweep > first_lost) && (first_sweep <= 60u),
          "after a short coast the search (sweep) starts");
    CHECK("S2-U24d", (first_timeout >= 295u) && (first_timeout <= 305u),
          "TIMEOUT is raised after LB_TIMEOUT_MS (3 s) without the line");

    /* Re-acquire needs LB_REACQ_TICKS good ticks in a row. */
    line_recov_init(&rec);
    in.b_seen = true;
    for (tick = 0u; tick < 20u; tick++) { line_recov_step(&rec, &in, &out); }
    in.b_seen = false;
    for (tick = 0u; tick < 12u; tick++) { line_recov_step(&rec, &in, &out); }
    in.b_seen = true;
    line_recov_step(&rec, &in, &out);
    CHECK("S2-U25a", LINE_RECOV_TRACK != out.mode,
          "one good tick does not end the search");
    in.b_seen = false;
    line_recov_step(&rec, &in, &out);
    in.b_seen = true;
    for (tick = 0u; tick < (uint32_t)LB_REACQ_TICKS; tick++)
    {
        line_recov_step(&rec, &in, &out);
    }
    CHECK("S2-U25b", (LINE_RECOV_TRACK == out.mode) || out.b_resumed,
          "enough good ticks in a row resume normal tracking");

    /* The timeout does not run while Mission is not following the line. */
    line_recov_init(&rec);
    in.b_seen = true;
    for (tick = 0u; tick < 20u; tick++) { line_recov_step(&rec, &in, &out); }
    in.b_seen = false;
    in.b_timeout_active = false;
    first_timeout = 0u;
    for (tick = 1u; tick <= 600u; tick++)
    {
        line_recov_step(&rec, &in, &out);
        if (0u != (out.flags & LINE_RECOV_F_TIMEOUT)) { first_timeout = tick; }
    }
    CHECK("S2-U26", 0u == first_timeout,
          "no TIMEOUT while Mission is turning, idle or bypassing");
}

/* ---------------------------------------------------------------------
 * Tests: closed loop
 * --------------------------------------------------------------------- */

#define COUNT_OF(a)  (sizeof(a) / sizeof((a)[0]))

static lb_sim_seg_t const g_straight[] = { {1500.0, 0.0} };
static lb_sim_seg_t const g_bends[] =
{
    {300.0, 0.0}, {314.0, 90.0}, {200.0, 0.0}, {314.0, -90.0}, {200.0, 0.0}
};

static void
test_closed_loop (void)
{
    lb_sim_line_cfg_t    cfg;
    lb_sim_line_result_t res;
    lb_sim_fade_t        fade;
    lb_sim_veer_t        veer;

    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_straight;
    cfg.num_segments = COUNT_OF(g_straight);
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-L01", res.b_finished && (res.max_offset_mm < 1.0),
          "straight line, no disturbance: stays centred");
#endif

    cfg.start_offset_mm = 8.0;
    cfg.start_heading_deg = 5.0;
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-L02", res.b_finished && (!res.b_timeout)
                    && (res.max_offset_mm < 12.0)
                    && (res.final_offset_mm > -2.0)
                    && (res.final_offset_mm < 2.0),
          "start 8 mm off and 5 degrees off: converges, no overshoot");
#endif

    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_bends;
    cfg.num_segments = COUNT_OF(g_bends);
    cfg.speed_mmps = 120.0;
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-L03", res.b_finished && (res.max_offset_mm < 20.0)
                    && (res.max_lost_ms < 1.0),
          "two 90 degree bends at 120 mm/s: followed, line never lost");
#endif
    cfg.speed_mmps = 60.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-L04", res.b_finished && (res.max_offset_mm < 20.0),
          "two 90 degree bends at 60 mm/s: followed");

    /* Car dynamics a factor of two off from the model.  At 60 mm/s, the
     * car's normal speed since 2026-11-09: with the gentler difference error
     * 120 mm/s is beyond what these slow motors can follow. */
    cfg.speed_mmps = 60.0;
    cfg.lag_s = 0.15;
    cfg.delay_ms = 50.0;
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-L05", res.b_finished && (res.max_offset_mm < 25.0),
          "slow motors (lag 150 ms, delay 50 ms): still followed");
#endif

    /* The line suddenly loses ink and the car is pushed sideways. */
    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_straight; cfg.num_segments = COUNT_OF(g_straight);
    fade.start_mm = 400.0; fade.length_mm = 250.0; fade.level = 0.25;
    fade.ramp_mm = 20.0;
    veer.start_mm = 380.0; veer.length_mm = 300.0; veer.yaw_dps = 4.0;
    cfg.p_fades = &fade; cfg.num_fades = 1;
    cfg.p_veers = &veer; cfg.num_veers = 1;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-R01", res.b_finished && (!res.b_timeout)
                    && (res.max_offset_mm < 25.0),
          "ink drops to 25 percent + 4 deg/s veer: car stays on the line");

    fade.level = 0.0; fade.length_mm = 100.0; veer.yaw_dps = 4.0;
    veer.length_mm = 140.0;
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-R02", res.b_finished && (!res.b_timeout)
                    && (res.recoveries >= 1u) && (res.max_offset_mm < 40.0),
          "100 mm without ink + veer: coast, search, line re-acquired");
#endif

    fade.level = 0.0; fade.length_mm = 200.0; veer.yaw_dps = 0.0;
    veer.length_mm = 0.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-R03", res.b_finished && (!res.b_timeout),
          "200 mm of missing line on a straight: crossed");

    fade.level = 0.0; fade.length_mm = 700.0;
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-R04", res.b_timeout && (res.max_lost_ms >= 2900.0),
          "line gone for good: TIMEOUT reported so Mission can take over");
#endif

    /* Fade inside a bend, at the car's normal 60 mm/s (see S2-L05). */
    lb_sim_line_defaults(&cfg);
    cfg.speed_mmps = 60.0;
    cfg.p_segments = g_bends; cfg.num_segments = COUNT_OF(g_bends);
    fade.start_mm = 480.0; fade.length_mm = 200.0; fade.level = 0.3;
    fade.ramp_mm = 30.0;
    cfg.p_fades = &fade; cfg.num_fades = 1;
    lb_sim_line_run(&cfg, &res);
#if (LB_LINE_STRADDLE == 0)
    CHECK("S2-R05", res.b_finished && (!res.b_timeout)
                    && (res.max_offset_mm < 25.0),
          "ink drops to 30 percent inside a bend: bend is still followed");
#endif
}

#if (LB_LINE_STRADDLE != 0)
/* Straddle mode: the line runs between the sensors, both normally white. */
static void
test_straddle (void)
{
    line_ctrl_t          ctrl;
    line_ctrl_out_t      out;
    lb_sim_line_cfg_t    cfg;
    lb_sim_line_result_t res;
    uint32_t             tick = 0u;
    bool                 b_ok = true;

    line_ctrl_init(&ctrl);
    for (tick = 0u; tick < 50u; tick++)
    {
        line_ctrl_update(&ctrl, 5, 5, &out);
    }
    CHECK("S2-S01a", out.b_seen && (0 == out.err) && (0 == out.steer),
          "straddle: both sensors white -> on the line, steer straight");

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 1000, 0, &out);
    b_ok = (-500 == out.err) && (out.steer < 0);
    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 0, 1000, &out);
    b_ok = b_ok && (500 == out.err) && (out.steer > 0);
    CHECK("S2-S01b", b_ok,
          "straddle: centre black -> steer left, right black -> steer right");

    /* after an obstacle bypass Mission says which side the line is on */
    line_ctrl_init(&ctrl);
    line_ctrl_restart(&ctrl, 0);
    line_ctrl_seek(&ctrl, 1);
    line_ctrl_update(&ctrl, 5, 5, &out);
    b_ok = (out.err > 0);
    line_ctrl_init(&ctrl);
    line_ctrl_seek(&ctrl, -1);
    line_ctrl_update(&ctrl, 5, 5, &out);
    b_ok = b_ok && (out.err < 0);
    line_ctrl_init(&ctrl);
    line_ctrl_seek(&ctrl, 0);
    line_ctrl_update(&ctrl, 5, 5, &out);
    CHECK("S2-S09", b_ok && (0 == out.err),
          "straddle: told the line is right/left, both white seeks that way; no hint, straight");

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 1000, 1000, &out);
    CHECK("S2-S01c", (0 == out.err),
          "straddle: both black (junction) -> straight on");

    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_straight;
    cfg.num_segments = COUNT_OF(g_straight);
    cfg.speed_mmps = 80.0;
    cfg.start_offset_mm = 8.0;
    cfg.start_heading_deg = 5.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-S02", res.b_finished && (!res.b_off_track)
                    && (res.max_offset_mm < 12.0)
                    && (res.final_offset_mm < 6.0),
          "straddle: straight at 80 mm/s from 8 mm off stays between sensors");

    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_bends;
    cfg.num_segments = COUNT_OF(g_bends);
    cfg.speed_mmps = 80.0;
    lb_sim_line_run(&cfg, &res);
    b_ok = res.b_finished && (!res.b_off_track) && (res.max_offset_mm < 15.0);
    cfg.speed_mmps = 18.0;
    lb_sim_line_run(&cfg, &res);
    b_ok = b_ok && res.b_finished && (!res.b_off_track)
           && (res.max_offset_mm < 18.0);
    CHECK("S2-S03", b_ok,
          "straddle: two 90 degree bends at 80 and 18 mm/s are followed");

    /* A junction (both sensors at 900): found, crossed straight, and no
     * "line went out" memory is left behind. */
    {
        line_junction_t jct;
        uint32_t        mm = 0u;
        bool            b_found = false;

        line_ctrl_init(&ctrl);
        line_junction_init(&jct);
        (void)line_junction_step(&jct, 20, 20, 0u);
        for (mm = 0u; mm < 20u; mm++)
        {
            line_ctrl_update(&ctrl, 900, 900, &out);
            b_found = line_junction_step(&jct, 900, 900, mm) || b_found;
        }
        b_ok = out.b_junction && b_found && (0 == out.err);
        line_ctrl_update(&ctrl, 20, 20, &out);
        line_ctrl_update(&ctrl, 20, 20, &out);
        CHECK("S2-S08", b_ok && (0 == ctrl.out_side) && (0 == out.err),
              "straddle: junction (both 900) found, crossed straight");
    }

    /* The line slides out past the centre sensor (it went black, then
     * white): keep steering left; after SEEK_MS of white report it lost. */
    line_ctrl_init(&ctrl);
    {
        static int32_t const dark[8] = { 300, 600, 800, 800, 600, 300, 0, 0 };
        uint32_t k = 0u;

        for (k = 0u; k < 8u; k++)
        {
            line_ctrl_update(&ctrl, dark[k], 0, &out);
        }
    }
    b_ok = out.b_seen && (out.steer < 0) && (-1 == ctrl.out_side);
    for (tick = 0u; tick < ((uint32_t)LB_STRADDLE_SEEK_MS / (uint32_t)LB_LINE_TICK_MS) + 2u; tick++)
    {
        line_ctrl_update(&ctrl, 0, 0, &out);
    }
    CHECK("S2-S04", b_ok && !out.b_seen,
          "straddle: line gone out past a sensor -> steer back that way, then lost");

    /* A light touch (peak below FULL once rescaled) means the line came back
     * inside. */
    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 0, 200, &out);
    line_ctrl_update(&ctrl, 0, 300, &out);
    line_ctrl_update(&ctrl, 0, 0, &out);
    for (tick = 0u; tick < 300u; tick++)
    {
        line_ctrl_update(&ctrl, 0, 0, &out);
    }
    CHECK("S2-S05", out.b_seen && (0 == out.err) && (0 == ctrl.out_side),
          "straddle: a light touch then white -> line inside, straight, never lost");

    /* A tight 100 mm radius bend at the car's 60 mm/s is followed.  (At
     * 80 mm/s with the old gains this needed the side memory; with the smooth
     * straddle gains, slew 40, 80 mm/s is too fast for this bend.) */
    {
        static lb_sim_seg_t const tight[] =
        {
            {300.0, 0.0}, {157.0, 90.0}, {200.0, 0.0}, {157.0, -90.0},
            {200.0, 0.0}
        };

        lb_sim_line_defaults(&cfg);
        cfg.p_segments = tight;
        cfg.num_segments = COUNT_OF(tight);
        cfg.speed_mmps = 60.0;
        lb_sim_line_run(&cfg, &res);
    }
    {
        bool b_s06 = res.b_finished && (!res.b_off_track);

        /* White-paper noise (up to 250, below LB_STRADDLE_BLACK_MIN) on one
         * sensor never counts as black: no "line went out" memory, never
         * lost, only a small steering wobble. */
        {
            static int32_t const noise[10] = { 0, 120, 250, 60, 200,
                                               10, 250, 90, 180, 0 };
            uint32_t k = 0u;
            bool     b_white = true;

            line_ctrl_init(&ctrl);
            for (k = 0u; k < 300u; k++)
            {
                line_ctrl_update(&ctrl, noise[k % 10u], 0, &out);
                b_white = b_white && out.b_seen && (0 == ctrl.out_side)
                          && (!ctrl.b_episode);
            }
            CHECK("S2-S07", b_white,
                  "straddle: white-paper noise is never taken for black");
        }
        res.b_finished = b_s06;
        res.b_off_track = !b_s06;
    }
    CHECK("S2-S06", res.b_finished && (!res.b_off_track),
          "straddle: tight 100 mm radius bend at 60 mm/s is followed");

    return;
}
#endif /* LB_LINE_STRADDLE */

int
main (void)
{
    printf("Line and barcode host tests (Buddy 3)\n\n");

    test_decoder();
    test_decoder_noise();
    test_stream();
    test_level();
    test_glitch();
    test_filter_calib();
    test_line_ctrl();
    test_line_junction();
    test_line_recov();
    test_optics();
    test_closed_loop();
#if (LB_LINE_STRADDLE != 0)
    test_straddle();
#endif

    printf("\n[TEST] line_barcode_host SUMMARY %u passed, %u failed\n",
           g_pass, g_fail);

    return ((0u == g_fail) ? 0 : 1);
}

/*** end of file ***/
