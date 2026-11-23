/** @file lb_sim_line.c
 *
 * @brief PC simulation of line following in a closed loop, driven by the real
 *        firmware modules (ir_filter, ir_calib, line_ctrl, line_recov).
 *
 * @par
 * Model:
 *  - The track centre line is a chain of straight and circular sections,
 *    stored as points 1 mm apart.
 *  - The car is a two-wheel differential drive.  Steering -1000 to +1000
 *    gives a wheel speed difference of up to wheel_diff_mmps, so the yaw
 *    rate is (difference / track width) at full steer.  The yaw rate follows
 *    the command with a first-order lag and a pure delay.
 *  - Two sensors sit ahead of the axle, a gap apart, symmetric about the car
 *    axis.  Each sees the average darkness over a spot of finite width.
 *  - A weak or missing stretch of ink and a sideways push (veer) can be
 *    placed anywhere on the line.
 *  - Sensors are sampled at 1 kHz with noise, 100 Hz flicker and the 10 ms
 *    average of the sampler; the control runs every 10 ms, as on the car.
 *
 * @par
 * What this proves: the CONTROL LOGIC copes with the stated conditions.  It
 * does not prove that the real car's dynamics, sensors and floor match the
 * model; the track tests in docs/BUDDY3_GUIDE.md do that.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "lb_config.h"
#include "ir_calib.h"
#include "ir_filter.h"
#include "line_ctrl.h"
#include "line_recov.h"
#include "lb_sim.h"

/* 12-bit counts per LB_ADC_BITS count */
#define ADC_SCALE ((double)(1 << (12 - LB_ADC_BITS)))

#define PI_VALUE        3.14159265358979
#define DEG2RAD         (PI_VALUE / 180.0)
#define POINT_STEP_MM   1.0
#define MAX_POINTS      20000u
#define SPOT_SAMPLES    7u
#define OFF_TRACK_MM    80.0
#define DELAY_MAX_MS    128u
#define TAIL_MM         60.0

typedef struct
{
    double x;       /* world position, mm; x forward at the start */
    double y;       /* + = left of the start direction */
    double dx;      /* unit tangent */
    double dy;
} point_t;

typedef struct
{
    point_t  pt[MAX_POINTS];
    uint32_t count;
} path_t;

static uint32_t g_rng = 777u;

static double
rng_uniform (void)
{
    g_rng = (g_rng * 1664525u) + 1013904223u;

    return ((double)(g_rng >> 8) / 16777216.0);
}

static double
rng_gauss (void)
{
    double u1 = rng_uniform() + 1.0e-12;
    double u2 = rng_uniform();

    return (sqrt(-2.0 * log(u1)) * cos(2.0 * PI_VALUE * u2));
}

/* Build the centre line from the sections, plus a straight tail. */
static void
build_path (path_t * p_path, lb_sim_line_cfg_t const * p_cfg)
{
    double   x = 0.0;
    double   y = 0.0;
    double   ang = 0.0;
    double   dang = 0.0;
    uint32_t seg = 0u;
    uint32_t n = 0u;
    uint32_t i = 0u;
    uint32_t steps = 0u;

    p_path->count = 0u;

    for (seg = 0u; seg <= p_cfg->num_segments; seg++)
    {
        double len  = TAIL_MM;
        double turn = 0.0;

        if (seg < p_cfg->num_segments)
        {
            len  = p_cfg->p_segments[seg].length_mm;
            turn = p_cfg->p_segments[seg].turn_deg * DEG2RAD;
        }

        steps = (uint32_t)(len / POINT_STEP_MM);
        dang  = (steps > 0u) ? (turn / (double)steps) : 0.0;

        for (i = 0u; (i < steps) && (n < MAX_POINTS); i++)
        {
            p_path->pt[n].x  = x;
            p_path->pt[n].y  = y;
            p_path->pt[n].dx = cos(ang);
            p_path->pt[n].dy = sin(ang);
            n++;
            ang += dang;
            x   += POINT_STEP_MM * cos(ang);
            y   += POINT_STEP_MM * sin(ang);
        }
    }

    p_path->count = n;
}

/* Nearest path point to (px, py); returns its index, and the signed
 * lateral distance (+ = the point is left of the line). */
static uint32_t
nearest (path_t const * p_path, uint32_t hint, double px, double py,
         double * p_lat)
{
    uint32_t lo = (hint > 150u) ? (hint - 150u) : 0u;
    uint32_t hi = ((hint + 150u) < p_path->count) ? (hint + 150u)
                                                  : (p_path->count - 1u);
    uint32_t best = lo;
    double   best_d2 = 1.0e18;
    uint32_t i = 0u;
    double   rx = 0.0;
    double   ry = 0.0;

    for (i = lo; i <= hi; i++)
    {
        rx = px - p_path->pt[i].x;
        ry = py - p_path->pt[i].y;

        if (((rx * rx) + (ry * ry)) < best_d2)
        {
            best_d2 = (rx * rx) + (ry * ry);
            best    = i;
        }
    }

    rx = px - p_path->pt[best].x;
    ry = py - p_path->pt[best].y;

    /* Cross product tangent x radius: + when the point is left of the line. */
    *p_lat = (p_path->pt[best].dx * ry) - (p_path->pt[best].dy * rx);

    return (best);
}

/* Ink strength (0 to 1) at distance s along the line. */
static double
ink_strength (lb_sim_line_cfg_t const * p_cfg, double s_mm)
{
    double   level = 1.0;
    uint32_t i = 0u;
    double   a = 0.0;
    double   t = 0.0;
    double   ramp = 0.0;

    for (i = 0u; i < p_cfg->num_fades; i++)
    {
        lb_sim_fade_t const * p_f = &p_cfg->p_fades[i];

        ramp = (p_f->ramp_mm > 0.1) ? p_f->ramp_mm : 0.1;
        a    = p_f->start_mm;

        if ((s_mm >= (a - ramp)) && (s_mm < a))
        {
            t = (s_mm - (a - ramp)) / ramp;
            level *= 1.0 - (t * (1.0 - p_f->level));
        }
        else if ((s_mm >= a) && (s_mm < (a + p_f->length_mm)))
        {
            level *= p_f->level;
        }
        else if ((s_mm >= (a + p_f->length_mm))
                 && (s_mm < (a + p_f->length_mm + ramp)))
        {
            t = (s_mm - (a + p_f->length_mm)) / ramp;
            level *= p_f->level + (t * (1.0 - p_f->level));
        }
    }

    return (level);
}

void
lb_sim_line_defaults (lb_sim_line_cfg_t * p_cfg)
{
    memset(p_cfg, 0, sizeof(*p_cfg));

    p_cfg->speed_mmps        = 120.0;   /* Motor: normal speed */
    p_cfg->axle_to_sensor_mm = 80.0;    /* assumed; measure on the car */
#if (LB_LINE_STRADDLE != 0)
    p_cfg->sensor_gap_mm     = 30.0;    /* straddle: wider than the line */
#else
    p_cfg->sensor_gap_mm     = 18.0;    /* about one line width */
#endif
    p_cfg->spot_mm           = 6.0;
    p_cfg->line_mm           = 18.0;    /* Barcode & Line Specification */
    p_cfg->track_mm          = 130.0;   /* Motor: wheel base */
    p_cfg->wheel_diff_mmps   = 120.0;   /* Motor: 60 mm/s per wheel */
    p_cfg->lag_s             = 0.07;
    p_cfg->delay_ms          = 20.0;
    p_cfg->noise_counts      = 25.0;
    p_cfg->flicker           = 0.04;
    p_cfg->ambient_shift     = 100.0;
    p_cfg->contrast          = 1.0;
}

/* Darkness count seen by one sensor. */
static double
sensor_count (path_t const * p_path, uint32_t * p_hint,
              lb_sim_line_cfg_t const * p_cfg,
              double sx, double sy, double nx, double ny, double * p_s_mm)
{
    double   cover = 0.0;
    double   lat = 0.0;
    double   off = 0.0;
    uint32_t k = 0u;
    uint32_t idx = *p_hint;
    double   white = (double)LB_WHITE_CENTER;
    double   black = (double)LB_BLACK_CENTER;

    for (k = 0u; k < SPOT_SAMPLES; k++)
    {
        off = ((((double)k + 0.5) / (double)SPOT_SAMPLES) - 0.5)
              * p_cfg->spot_mm;
        idx = nearest(p_path, idx, sx + (off * nx), sy + (off * ny), &lat);

        if (fabs(lat) < (p_cfg->line_mm / 2.0))
        {
            cover += 1.0;
        }
    }

    *p_hint = idx;
    *p_s_mm = (double)idx * POINT_STEP_MM;
    cover  /= (double)SPOT_SAMPLES;

    return (white + (p_cfg->contrast * ink_strength(p_cfg, *p_s_mm) * cover
                     * (black - white)));
}

void
lb_sim_line_run (lb_sim_line_cfg_t const * p_cfg, lb_sim_line_result_t * p_res)
{
    static path_t     path;
    static double     delay_line[DELAY_MAX_MS];
    ir_filter_t       filt_c;
    ir_filter_t       filt_r;
    ir_calib_t        cal_c = { LB_WHITE_CENTER, LB_BLACK_CENTER };
    ir_calib_t        cal_r = { LB_WHITE_RIGHT, LB_BLACK_RIGHT };
    line_ctrl_t       ctrl;
    line_recov_t      recov;
    line_ctrl_out_t   co;
    line_recov_out_t  ro;
    line_recov_in_t   ri;
    double  px = 0.0, py = 0.0, psi = 0.0;     /* car axle centre and heading */
    double  omega = 0.0;                       /* yaw rate, rad/s, + = left */
    double  steer_cmd = 0.0;
    double  travelled = 0.0;
    double  lost_ms = 0.0;
    double  sum_sq = 0.0;
    double  offset = 0.0;
    double  lat = 0.0;
    double  s_mm = 0.0;
    double  dt = 0.001;
    uint32_t hint_c = 0u, hint_r = 0u, hint_car = 0u;
    uint32_t ms = 0u;
    uint32_t dpos = 0u;
    uint32_t n_off = 0u;
    uint32_t i = 0u;
    bool     b_done = false;
    bool     b_was_lost = false;
    double   path_len = 0.0;
    double   phase = rng_uniform() * 2.0 * PI_VALUE;

    memset(p_res, 0, sizeof(*p_res));
    memset(delay_line, 0, sizeof(delay_line));
    build_path(&path, p_cfg);
    path_len = (double)path.count * POINT_STEP_MM;

    ir_filter_init(&filt_c);
    ir_filter_init(&filt_r);
    line_ctrl_init(&ctrl);
    line_recov_init(&recov);

    /* Start pose: offset to the left of the line start, with a heading. */
    px  = 0.0 - (0.0);
    py  = p_cfg->start_offset_mm;
    psi = p_cfg->start_heading_deg * DEG2RAD;

    for (ms = 0u; (!b_done) && (ms < 120000u); ms++)
    {
        double cs = cos(psi), sn = sin(psi);
        double lx = -sn, ly = cs;          /* unit vector pointing left */
        double ahead = p_cfg->axle_to_sensor_mm;
        double gap = p_cfg->sensor_gap_mm / 2.0;
        double cnt_c, cnt_r, flick, veer = 0.0, target = 0.0;

        /* Sensors: centre at +gap, right at -gap (see line_ctrl.h). */
        cnt_c = sensor_count(&path, &hint_c, p_cfg,
                             px + (ahead * cs) + (gap * lx),
                             py + (ahead * sn) + (gap * ly), lx, ly, &s_mm);
        cnt_r = sensor_count(&path, &hint_r, p_cfg,
                             px + (ahead * cs) - (gap * lx),
                             py + (ahead * sn) - (gap * ly), lx, ly, &s_mm);

        flick = 1.0 + (p_cfg->flicker
                       * sin((2.0 * PI_VALUE * 100.0 * (double)ms / 1000.0)
                             + phase));
        /* ambient_shift and noise_counts are given in 12-bit counts (as
         * measured); scale them to the LB_ADC_BITS readings. */
        cnt_c = ((cnt_c + (p_cfg->ambient_shift / ADC_SCALE)) * flick)
                + ((p_cfg->noise_counts / ADC_SCALE) * rng_gauss());
        cnt_r = ((cnt_r + (p_cfg->ambient_shift / ADC_SCALE)) * flick)
                + ((p_cfg->noise_counts / ADC_SCALE) * rng_gauss());
        cnt_c = fmin(fmax(cnt_c, 0.0), (double)LB_ADC_MAX_COUNT);
        cnt_r = fmin(fmax(cnt_r, 0.0), (double)LB_ADC_MAX_COUNT);

        {
            int32_t fc = ir_filter_push(&filt_c, (int32_t)floor(cnt_c + 0.5));
            int32_t fr = ir_filter_push(&filt_r, (int32_t)floor(cnt_r + 0.5));

            /* Control tick every 10 ms. */
            if ((ms % (uint32_t)LB_LINE_TICK_MS) == ((uint32_t)LB_LINE_TICK_MS - 1u))
            {
                line_ctrl_update(&ctrl, ir_calib_darkness(&cal_c, fc),
                                 ir_calib_darkness(&cal_r, fr), &co);
                ri.b_seen = co.b_seen;
                ri.err = co.err;
                ri.steer = co.steer;
                ri.dt_ms = LB_LINE_TICK_MS;
                ri.b_timeout_active = true;
                line_recov_step(&recov, &ri, &ro);
                steer_cmd = (double)ro.steer;

                p_res->ticks_total++;
                if (LINE_RECOV_COAST == ro.mode) { p_res->ticks_coast++; }
                if (LINE_RECOV_SWEEP == ro.mode) { p_res->ticks_sweep++; }
                if (ro.b_resumed) { p_res->recoveries++; }
                if (0u != (ro.flags & LINE_RECOV_F_TIMEOUT))
                {
                    p_res->b_timeout = true;
                    b_done = true;
                }
                if (0u != (ro.flags & LINE_RECOV_F_LOST))
                {
                    lost_ms += (double)LB_LINE_TICK_MS;
                    b_was_lost = true;
                    if (lost_ms > p_res->max_lost_ms)
                    {
                        p_res->max_lost_ms = lost_ms;
                    }
                }
                else
                {
                    lost_ms = 0.0;
                }
            }
        }

        /* Command delay (Mission, buffers, motor loop), then the lag. */
        delay_line[ms % DELAY_MAX_MS] = steer_cmd;
        dpos = (uint32_t)(p_cfg->delay_ms);
        if (dpos >= DELAY_MAX_MS) { dpos = DELAY_MAX_MS - 1u; }
        target = delay_line[(ms + DELAY_MAX_MS - dpos) % DELAY_MAX_MS];
        /* Positive steer = turn right = negative (clockwise) yaw. */
        target = -(target / (double)LB_STEER_MAX)
                 * (p_cfg->wheel_diff_mmps / p_cfg->track_mm);

        for (i = 0u; i < p_cfg->num_veers; i++)
        {
            lb_sim_veer_t const * p_v = &p_cfg->p_veers[i];

            if ((travelled >= p_v->start_mm)
                && (travelled < (p_v->start_mm + p_v->length_mm)))
            {
                veer += p_v->yaw_dps * DEG2RAD;
            }
        }

        omega += ((target + veer) - omega) * (dt / p_cfg->lag_s);
        psi   += omega * dt;
        px    += p_cfg->speed_mmps * dt * cos(psi);
        py    += p_cfg->speed_mmps * dt * sin(psi);
        travelled += p_cfg->speed_mmps * dt;

        /* Where is the car relative to the line? */
        hint_car = nearest(&path, hint_car, px, py, &lat);
        offset   = lat;
        sum_sq  += offset * offset;
        n_off++;
        if (fabs(offset) > p_res->max_offset_mm)
        {
            p_res->max_offset_mm = fabs(offset);
        }
        if (fabs(offset) > OFF_TRACK_MM)
        {
            p_res->b_off_track = true;
            b_done = true;
        }
        if (((double)hint_car * POINT_STEP_MM) >= (path_len - TAIL_MM))
        {
            p_res->b_finished = true;
            b_done = true;
        }
    }

    p_res->distance_mm     = travelled;
    p_res->final_offset_mm = offset;
    p_res->rms_offset_mm   = (n_off > 0u) ? sqrt(sum_sq / (double)n_off) : 0.0;
    (void)b_was_lost;
}

/*** end of file ***/
