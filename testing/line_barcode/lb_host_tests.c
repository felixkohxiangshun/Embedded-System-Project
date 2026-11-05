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
    CHECK("S2-U06c", BC_DECODE_OK != decode_symbol('Q', 50000.0, &cmd, &rev),
          "a valid but wrong frame/char (Q) does not give a command");
    make_edges('A', 50000.0, 1000u, edge);
    edge[10] += 120000u;            /* inter-character gap far too long */
    CHECK("S2-U06d", BC_DECODE_OK != bc_decode_edges(edge, NUM_EDGES, &cmd,
                                                     &rev),
          "a wrong inter-character gap is rejected");

    /* Valid Code 39 characters that are not commands are ignored. */
    b_all = true;
    for (idx = 0u; idx < 5u; idx++)
    {
        static char const other[5] = {'F', 'W', 'X', 'Y', 'Z'};

        ch = other[idx];
        b_all = b_all
                && (BC_DECODE_ERR_COMMAND
                    == decode_symbol(ch, 50000.0, &cmd, &rev))
                && ((uint8_t)ch == cmd);
    }
    CHECK("S2-U13", b_all, "F, W, X, Y, Z are read but not accepted");

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

static void
test_line_ctrl (void)
{
    line_ctrl_t     ctrl;
    line_ctrl_out_t out;
    uint32_t        tick = 0u;

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 1000, 0, &out);
    CHECK("S2-U07a", (out.err < 0) && out.b_seen,
          "line under the left sensor -> error < 0 (steer left)");

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 0, 1000, &out);
    CHECK("S2-U07b", (out.err > 0) && out.b_seen,
          "line under the centre sensor -> error > 0 (steer right)");

    line_ctrl_init(&ctrl);
    line_ctrl_update(&ctrl, 600, 600, &out);
    CHECK("S2-U07c", (out.err > -30) && (out.err < 30),
          "line half way between the sensors -> error about 0");

    line_ctrl_init(&ctrl);
    for (tick = 0u; tick < 5u; tick++)
    {
        line_ctrl_update(&ctrl, 5, 5, &out);
    }
    CHECK("S2-U07d", !out.b_seen, "both sensors on paper -> line not seen");

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
        CHECK("S2-U23", (a.err - b.err <= 5) && (b.err - a.err <= 5)
                        && b.b_seen && b.b_faded,
              "half the ink: same position error, flagged as faded");
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
    CHECK("S2-L01", res.b_finished && (res.max_offset_mm < 1.0),
          "straight line, no disturbance: stays centred");

    cfg.start_offset_mm = 8.0;
    cfg.start_heading_deg = 5.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-L02", res.b_finished && (!res.b_timeout)
                    && (res.max_offset_mm < 12.0)
                    && (res.final_offset_mm > -2.0)
                    && (res.final_offset_mm < 2.0),
          "start 8 mm off and 5 degrees off: converges, no overshoot");

    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_bends;
    cfg.num_segments = COUNT_OF(g_bends);
    cfg.speed_mmps = 120.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-L03", res.b_finished && (res.max_offset_mm < 20.0)
                    && (res.max_lost_ms < 1.0),
          "two 90 degree bends at 120 mm/s: followed, line never lost");
    cfg.speed_mmps = 60.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-L04", res.b_finished && (res.max_offset_mm < 20.0),
          "two 90 degree bends at 60 mm/s: followed");

    /* Car dynamics a factor of two off from the model. */
    cfg.speed_mmps = 120.0;
    cfg.lag_s = 0.15;
    cfg.delay_ms = 50.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-L05", res.b_finished && (res.max_offset_mm < 25.0),
          "slow motors (lag 150 ms, delay 50 ms): still followed");

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
    CHECK("S2-R02", res.b_finished && (!res.b_timeout)
                    && (res.recoveries >= 1u) && (res.max_offset_mm < 40.0),
          "100 mm without ink + veer: coast, search, line re-acquired");

    fade.level = 0.0; fade.length_mm = 200.0; veer.yaw_dps = 0.0;
    veer.length_mm = 0.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-R03", res.b_finished && (!res.b_timeout),
          "200 mm of missing line on a straight: crossed");

    fade.level = 0.0; fade.length_mm = 700.0;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-R04", res.b_timeout && (res.max_lost_ms >= 2900.0),
          "line gone for good: TIMEOUT reported so Mission can take over");

    /* Fade inside a bend. */
    lb_sim_line_defaults(&cfg);
    cfg.p_segments = g_bends; cfg.num_segments = COUNT_OF(g_bends);
    fade.start_mm = 480.0; fade.length_mm = 200.0; fade.level = 0.3;
    fade.ramp_mm = 30.0;
    cfg.p_fades = &fade; cfg.num_fades = 1;
    lb_sim_line_run(&cfg, &res);
    CHECK("S2-R05", res.b_finished && (!res.b_timeout)
                    && (res.max_offset_mm < 25.0),
          "ink drops to 30 percent inside a bend: bend is still followed");
}

int
main (void)
{
    printf("Line and barcode host tests (Buddy 3)\n\n");

    test_decoder();
    test_decoder_noise();
    test_stream();
    test_glitch();
    test_filter_calib();
    test_line_ctrl();
    test_line_recov();
    test_optics();
    test_closed_loop();

    printf("\n[TEST] line_barcode_host SUMMARY %u passed, %u failed\n",
           g_pass, g_fail);

    return ((0u == g_fail) ? 0 : 1);
}

/*** end of file ***/
