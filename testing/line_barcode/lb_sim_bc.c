/** @file lb_sim_bc.c
 *
 * @brief PC simulation of the barcode path: optics, sampler, slicer, edge
 *        stream and decoder, driven by the real firmware modules.
 *
 * @par
 * Model: the code is a row of bars on paper.  The sensor sees the average
 * reflectance over a spot of finite width (this blurs narrow bars), plus
 * noise, 100 Hz mains flicker and a 12-bit quantiser.  The car drives at a
 * given speed, optionally changing speed while it crosses the code.  The
 * signal is sampled once per millisecond, exactly like ir_sampler_isr().
 *
 * @par
 * What this proves: the LOGIC (slicer, stream, decoder) reads the code
 * under the stated conditions.  It does not prove the real sensor behaves
 * like the model; testing/line_barcode/TEST_LOG.md has the manual tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "lb_config.h"
#include "ir_filter.h"
#include "bc_slicer.h"
#include "bc_stream.h"
#include "bc_decode.h"
#include "lb_sim.h"

/* 12-bit counts per LB_ADC_BITS count */
#define ADC_SCALE ((double)(1 << (12 - LB_ADC_BITS)))

#define PROFILE_STEP_MM     0.05    /* resolution of the printed pattern */
#define PROFILE_LEAD_MM     60.0    /* paper in front of the code */
#define PROFILE_TAIL_MM     60.0    /* paper behind the code */
#define NARROW_MM           3.0
#define PI_VALUE            3.14159265358979

/* Code 39 element patterns of the symbols the simulation prints. */
static char const * const g_pattern_star = "NWNNWNWNN";

typedef struct
{
    uint8_t * p_dark;       /* 1 where a bar is printed, per PROFILE_STEP_MM */
    uint32_t  length;
} profile_t;

static uint32_t g_rng_state = 12345u;

/* Small, repeatable random numbers: the results must not change between
 * runs or between machines. */
static uint32_t
rng_next (void)
{
    g_rng_state = (g_rng_state * 1664525u) + 1013904223u;

    return (g_rng_state);
}

static double
rng_uniform (void)
{
    return ((double)(rng_next() >> 8) / 16777216.0);
}

static double
rng_gauss (void)
{
    double u1 = rng_uniform() + 1.0e-12;
    double u2 = rng_uniform();

    return (sqrt(-2.0 * log(u1)) * cos(2.0 * PI_VALUE * u2));
}

void
lb_sim_seed (uint32_t seed)
{
    g_rng_state = seed;

    return;
}

/* Pattern string of one symbol; NULL if the simulation does not know it. */
char const *
lb_sim_pattern (char symbol)
{
    static char const * const p_table[] =
    {
        "WNNNNWNNW", "NNWNNWNNW", "WNWNNWNNN", "NNNNWWNNW",  /* A B C D */
        "WNNNWWNNN", "NNWNWWNNN", "NNNNNWWNW", "WNNNNWWNN",  /* E F G H */
        "NNWNNWWNN", "NNNNWWWNN", "WNNNNNNWW", "NNWNNNNWW",  /* I J K L */
        "WNWNNNNWN", "NNNNWNNWW", "WNNNWNNWN", "NNWNWNNWN",  /* M N O P */
        "NNNNNNWWW", "WNNNNNWWN", "NNWNNNWWN", "NNNNWNWWN",  /* Q R S T */
        "WWNNNNNNW", "NWWNNNNNW", "WWWNNNNNN", "NWNNWNNNW",  /* U V W X */
        "WWNNWNNNN", "NWWNWNNNN"                             /* Y Z */
    };
    char const * p_result = NULL;

    if ((symbol >= 'A') && (symbol <= 'Z'))
    {
        p_result = p_table[symbol - 'A'];
    }
    else if ('*' == symbol)
    {
        p_result = g_pattern_star;
    }
    else
    {
        /* Unknown symbol. */
    }

    return (p_result);
}

/* Print "*X*" into a bar profile.  Returns the code length in mm. */
static double
print_code (profile_t * p_profile, char symbol, double wide_ratio)
{
    char const * p_parts[3];
    double       x_mm   = PROFILE_LEAD_MM;
    double       width  = 0.0;
    uint32_t     part   = 0u;
    uint32_t     elem   = 0u;
    uint32_t     step   = 0u;
    uint32_t     first  = 0u;
    uint32_t     last   = 0u;

    p_parts[0] = g_pattern_star;
    p_parts[1] = lb_sim_pattern(symbol);
    p_parts[2] = g_pattern_star;

    for (part = 0u; part < 3u; part++)
    {
        for (elem = 0u; elem < 9u; elem++)
        {
            width = ('W' == p_parts[part][elem]) ? (NARROW_MM * wide_ratio)
                                                  : NARROW_MM;
            first = (uint32_t)(x_mm / PROFILE_STEP_MM);
            last  = (uint32_t)((x_mm + width) / PROFILE_STEP_MM);

            for (step = first; (step < last) && (0u == (elem % 2u)); step++)
            {
                p_profile->p_dark[step] = 1u;
            }

            x_mm += width;
        }

        x_mm += NARROW_MM;      /* inter-character gap */
    }

    return (x_mm - PROFILE_LEAD_MM - NARROW_MM);
}

/* Reflectance seen by a spot centred at x_mm: the mean of the printed
 * pattern over the spot width. */
static double
spot_signal (profile_t const * p_profile, double x_mm, double spot_mm)
{
    int32_t first = (int32_t)((x_mm - (spot_mm / 2.0)) / PROFILE_STEP_MM);
    int32_t last  = (int32_t)((x_mm + (spot_mm / 2.0)) / PROFILE_STEP_MM);
    int32_t step  = 0;
    int32_t sum   = 0;
    int32_t num   = 0;

    for (step = first; step <= last; step++)
    {
        if ((step >= 0) && ((uint32_t)step < p_profile->length))
        {
            sum += (int32_t)p_profile->p_dark[step];
        }
        else
        {
            /* Outside the printed strip: bare paper. */
        }

        num++;
    }

    return ((double)sum / (double)num);
}

lb_sim_bc_result_t
lb_sim_bc_run (lb_sim_bc_cfg_t const * p_cfg, char symbol)
{
    lb_sim_bc_result_t result    = LB_SIM_BC_NO_DECODE;
    profile_t          profile   = { NULL, 0u };
    bc_slicer_t        slicer;
    ir_filter_t        filter;
    bc_stream_t        stream;
    bc_slicer_edge_t   edge;
    bc_stream_event_t  event     = BC_STREAM_EVENT_NONE;
    bc_decode_status_t status    = BC_DECODE_OK;
    uint8_t            command   = 0u;
    bool               b_rev     = false;
    bool               b_done    = false;
    double             x_mm      = 0.0;
    double             speed     = 0.0;
    double             code_mm   = 0.0;
    double             total_mm  = 0.0;
    double             t_ms      = 0.0;
    double             count     = 0.0;
    double             phase     = rng_uniform() * 2.0 * PI_VALUE;
    double             lead_ms   = 0.0;
    uint32_t           now_us    = 0u;
    uint32_t           last_edge_ms = 0u;
    uint32_t           ms        = 0u;
    int32_t            dark      = 0;

    total_mm = PROFILE_LEAD_MM + 141.0 * (p_cfg->wide_ratio / 3.0)
               + PROFILE_TAIL_MM + 40.0;
    profile.length = (uint32_t)(total_mm / PROFILE_STEP_MM) + 1u;
    profile.p_dark = (uint8_t *)calloc(profile.length, 1u);

    if (NULL != profile.p_dark)
    {
        code_mm = print_code(&profile, symbol, p_cfg->wide_ratio);

        bc_slicer_init(&slicer);
        ir_filter_init(&filter);
        bc_stream_reset(&stream);

        /* The car starts standing before the code long enough for the
         * slicer to warm up, then drives at the chosen speed. */
        lead_ms = (double)(LB_BC_WARMUP_SAMPLES + 150);
        x_mm    = PROFILE_LEAD_MM - (p_cfg->speed_mmps * lead_ms / 1000.0);

        for (ms = 0u; (!b_done)
             && (x_mm < (PROFILE_LEAD_MM + code_mm + 80.0)
                 || bc_stream_is_active(&stream))
             && (ms < 20000u); ms++)
        {
            speed = p_cfg->speed_mmps;
            t_ms  = (double)ms;

            if (x_mm > (PROFILE_LEAD_MM - 10.0))
            {
                /* Speed wobble while crossing the code. */
                speed *= 1.0 + (p_cfg->speed_wobble
                               * sin(2.0 * PI_VALUE * 0.8 * t_ms / 1000.0));
            }
            else
            {
                /* Approach at the nominal speed. */
            }

            x_mm += speed / 1000.0;

            count = (double)LB_BC_WHITE_NOMINAL
                    + (p_cfg->contrast
                       * (double)(LB_BC_BLACK_NOMINAL - LB_BC_WHITE_NOMINAL)
                       * spot_signal(&profile, x_mm, p_cfg->spot_mm));
            count += p_cfg->ambient_shift / ADC_SCALE;   /* 12-bit counts */
            count *= 1.0 + (p_cfg->flicker
                           * sin((2.0 * PI_VALUE * 100.0 * t_ms / 1000.0)
                                 + phase));
            count += (p_cfg->noise_counts / ADC_SCALE) * rng_gauss();

            dark = (int32_t)floor(count + 0.5);
            if (dark < 0)
            {
                dark = 0;
            }
            if (dark > LB_ADC_MAX_COUNT)
            {
                dark = LB_ADC_MAX_COUNT;
            }

            now_us = (ms * 1000u)
                     + (uint32_t)(rng_uniform() * p_cfg->jitter_us);

            /* The sampler averages the barcode channel over 10 ms, exactly
             * like the line channels, to cancel the 100 Hz mains flicker. */
            dark = ir_filter_push(&filter, dark);

            if (bc_slicer_step(&slicer, dark, now_us, &edge))
            {
                event = bc_stream_push(&stream, edge.time_us, edge.b_dark);
                last_edge_ms = ms;

                if (BC_STREAM_EVENT_FULL == event)
                {
                    status = bc_stream_decode(&stream, &command, &b_rev);
                    if (BC_DECODE_OK == status)
                    {
                        b_done = true;
                    }
                }
                else if (BC_STREAM_EVENT_OVERFLOW == event)
                {
                    b_done = true;
                }
            }
            else if (bc_stream_is_active(&stream)
                     && ((ms - last_edge_ms) >= bc_stream_silence_ms(&stream)))
            {
                b_done = true;
            }
        }

        status = bc_stream_decode(&stream, &command, &b_rev);

        if ((BC_DECODE_OK == status) && (!b_rev))
        {
            result = ((uint8_t)symbol == command) ? LB_SIM_BC_CORRECT
                                                  : LB_SIM_BC_WRONG;
        }
        else if (BC_DECODE_ERR_COMMAND == status)
        {
            result = LB_SIM_BC_IGNORED;
        }
        else
        {
            result = LB_SIM_BC_NO_DECODE;
        }

        free(profile.p_dark);
    }

    return (result);
}

/*** end of file ***/
