/** @file line_task.c
 *
 * @brief Line tracking task.  See line_task.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stdint.h>
#include <stdbool.h>
#include "line_barcode.h"
#include "ir_calib.h"
#include "ir_driver.h"
#include "line_ctrl.h"
#include "line_recov.h"
#include "line_task.h"

/* The recovery module reports its flags with its own constants so that it
 * can be tested on a PC without the kernel headers.  They must equal the
 * team interface values in app_if.h, or Mission would misread them. */
#if (LINE_RECOV_F_LOST != STEER_F_LOST)
#error "LINE_RECOV_F_LOST differs from STEER_F_LOST in app_if.h"
#endif

#if (LINE_RECOV_F_TIMEOUT != STEER_F_TIMEOUT)
#error "LINE_RECOV_F_TIMEOUT differs from STEER_F_TIMEOUT in app_if.h"
#endif

#if (LB_STEER_MAX != STEER_MAX)
#error "LB_STEER_MAX differs from STEER_MAX in app_if.h"
#endif

/** Everything the task remembers between two ticks. */
typedef struct
{
    line_ctrl_t  ctrl;          /* position error and PD steering */
    line_recov_t recov;         /* faded / lost / veered recovery */
    line_junction_t junction;   /* side branch under the line sensors */
    uint32_t     dist_mm;       /* last distance read from app_state */
    bool         b_jct_pending; /* branch found, Mission not told yet */
    uint32_t     last_us;       /* time of the previous tick */
    bool         b_first;       /* no previous tick yet */
    bool         b_following;   /* Mission follows the line right now */
} line_run_t;

static ir_calib_t const g_cal_centre = { LB_WHITE_CENTER, LB_BLACK_CENTER };
static ir_calib_t const g_cal_right  = { LB_WHITE_RIGHT, LB_BLACK_RIGHT };

static line_run_t g_run;

/* Private function prototypes. */
static void     run_init(line_run_t * p_run);
static void     run_tick(line_run_t * p_run);
static int32_t  elapsed_ms(line_run_t * p_run);
static bool     mission_follows_line(bool b_previous);
static uint32_t read_distance(uint32_t previous_mm);
static void     post_steer(line_run_t * p_run,
                           line_recov_out_t const * p_recov,
                           int32_t dark_centre,
                           int32_t dark_right);
static void     update_stats(line_ctrl_out_t const * p_ctrl,
                             line_recov_out_t const * p_recov,
                             int32_t dark_centre,
                             int32_t dark_right,
                             int32_t dt_ms);

void
line_wakeup_isr (void * p_exinf)
{
    (void)p_exinf;

    (void)tk_wup_tsk(g_lb_tid_line);

    return;
}

void
line_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    run_init(&g_run);

    for (;;)
    {
        /* Sleep until line_wakeup_isr() wakes us (every 10 ms).  Extra
         * wake-ups that piled up while a tick was late are discarded so
         * the task never runs two ticks back to back. */
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);

        run_tick(&g_run);
    }
}

static void
run_init (line_run_t * p_run)
{
    line_ctrl_init(&p_run->ctrl);
    line_recov_init(&p_run->recov);
    p_run->last_us      = 0u;
    p_run->b_first      = true;
    p_run->b_following  = false;

    return;
}

/*!
 * @brief One 10 ms tick: sample, control, recover, post.
 */
static void
run_tick (line_run_t * p_run)
{
    line_ctrl_out_t  ctrl_out;
    line_recov_out_t recov_out;
    line_recov_in_t  recov_in;
    bool             b_follow = false;
    int32_t          dt_ms    = elapsed_ms(p_run);
    int32_t          dark_c   = 0;
    int32_t          dark_r   = 0;

    /* 1. SAMPLING: 10 ms averages, normalised to 0 (paper) to 1000 (line). */
    dark_c = ir_calib_darkness(&g_cal_centre,
                               ir_driver_get_dark(IR_CHANNEL_CENTRE));
    dark_r = ir_calib_darkness(&g_cal_right,
                               ir_driver_get_dark(IR_CHANNEL_RIGHT));

    /* A fresh start whenever Mission begins to follow the line again (after
     * idle, a turn, an obstacle bypass): old memories would steer wrongly. */
    b_follow = mission_follows_line(p_run->b_following);
    if (b_follow && (!p_run->b_following))
    {
        line_ctrl_restart(&p_run->ctrl, 0);
        line_recov_init(&p_run->recov);
        line_junction_init(&p_run->junction);
        p_run->b_jct_pending = false;
    }
    else
    {
        /* No change in what Mission is doing. */
    }
    p_run->b_following = b_follow;

    /* 2. STEERING: position error and PD steering. */
    line_ctrl_update(&p_run->ctrl, dark_c, dark_r, &ctrl_out);

    /* Side branch: only while the car follows the line. */
    p_run->dist_mm = read_distance(p_run->dist_mm);
    if (b_follow
        && line_junction_step(&p_run->junction,
                              dark_c,
                              dark_r,
                              p_run->dist_mm))
    {
        p_run->b_jct_pending = true;
    }
    else
    {
        /* No new branch. */
    }

    /* 3. Recovery: pass the steering through, or coast / search. */
    recov_in.b_seen           = ctrl_out.b_seen;
    recov_in.err              = ctrl_out.err;
    recov_in.steer            = ctrl_out.steer;
    recov_in.dt_ms            = dt_ms;
    recov_in.b_timeout_active = (0 == LB_PAUSE_TIMEOUT_OFF_LINE) || b_follow;
    line_recov_step(&p_run->recov, &recov_in, &recov_out);

    /* 4. Post to Mission and publish the counters. */
    post_steer(p_run, &recov_out, dark_c, dark_r);
    update_stats(&ctrl_out, &recov_out, dark_c, dark_r, dt_ms);

    return;
}

/*!
 * @brief Milliseconds since the previous tick, limited to a sane range.
 *
 * The real elapsed time is used (not the nominal 10 ms) so that the recovery
 * timers stay correct even if a tick runs late.  The first tick, and any
 * absurd value, count as one nominal tick.
 */
static int32_t
elapsed_ms (line_run_t * p_run)
{
    /* Cast: APP_NOW_US() is the kernel's 32-bit unsigned UW. */
    uint32_t now_us = (uint32_t)APP_NOW_US();
    uint32_t diff   = 0u;
    int32_t  dt_ms  = LB_LINE_TICK_MS;

    if (!p_run->b_first)
    {
        /* Unsigned subtraction stays correct when the timer wraps. */
        diff  = now_us - p_run->last_us;

        /* Cast: diff / 1000 is below 4.3 million, so it fits int32_t. */
        dt_ms = (int32_t)((diff + 500u) / 1000u);

        if (dt_ms < 1)
        {
            dt_ms = 1;
        }
        else if (dt_ms > LB_DT_MAX_MS)
        {
            dt_ms = LB_DT_MAX_MS;
        }
        else
        {
            /* Normal range. */
        }
    }
    else
    {
        p_run->b_first = false;
    }

    p_run->last_us = now_us;

    return (dt_ms);
}

/*!
 * @brief Is Mission in a state where it forwards the steering to the motors?
 *
 * Those are the states in which losing the line is a real problem.  While
 * Mission turns for a barcode command or bypasses an obstacle, the line is
 * not expected under the car and the lost-line timer must not run.
 *
 * The shared state is read without waiting: if another task holds the lock
 * the previous answer is kept.
 *
 * @param[in] b_previous  The answer of the previous tick.
 *
 * @return true if Mission follows the line.
 */
static bool
mission_follows_line (bool b_previous)
{
    bool    b_follow = b_previous;
    uint8_t state    = 0u;

    if (E_OK == state_lock(TMO_POL))
    {
        /* Cast: sys_state is the kernel's UB, an unsigned 8-bit value. */
        state = (uint8_t)app_state.sys_state;
        state_unlock();

        b_follow = (SYS_LINE_FOLLOW == state)
                   || (SYS_BARCODE_SCAN == state)
                   || (SYS_HUMP_TRAVERSE == state);
    }
    else
    {
        /* Lock busy: keep the previous answer. */
    }

    return (b_follow);
}

/*!
 * @brief Distance driven so far.  If the lock is busy the last value is kept.
 *
 * @param[in] previous_mm  The value of the previous tick.
 *
 * @return Distance in millimetres.
 */
static uint32_t
read_distance (uint32_t previous_mm)
{
    uint32_t dist_mm = previous_mm;

    if (E_OK == state_lock(TMO_POL))
    {
        /* Cast: dist_mm is the kernel's UW, an unsigned 32-bit value. */
        dist_mm = (uint32_t)app_state.dist_mm;
        state_unlock();
    }
    else
    {
        /* Lock busy: keep the previous value. */
    }

    return (dist_mm);
}

/*!
 * @brief Send MSG_STEER to Mission.  A full buffer drops the message: Mission
 *        is behind and the next tick brings a newer value anyway.  A branch
 *        report is kept and sent again until it is delivered.
 */
static void
post_steer (line_run_t * p_run,
            line_recov_out_t const * p_recov,
            int32_t dark_centre,
            int32_t dark_right)
{
    MSG_STEER msg;

    /* Casts: MSG_STEER uses the kernel's INT and UB types (app_if.h). */
    msg.steer = (INT)p_recov->steer;
    msg.flags = (UB)p_recov->flags;

    /* Black under each sensor right now, whatever the car is doing:
     * Mission uses them to find the line again after an obstacle bypass. */
    if (dark_centre >= LB_STEER_DARK_MIN)
    {
        msg.flags |= STEER_F_DARK_C;
    }
    else
    {
        /* Centre sensor on white. */
    }

    if (dark_right >= LB_STEER_DARK_MIN)
    {
        msg.flags |= STEER_F_DARK_R;
    }
    else
    {
        /* Right sensor on white. */
    }

    if (p_run->b_jct_pending)
    {
        msg.flags |= STEER_F_JUNCTION;
    }
    else
    {
        /* No branch to report. */
    }

    if (E_OK == MBF_SEND(mbf_steer, msg))
    {
        p_run->b_jct_pending = false;
    }
    else
    {
        /* Full: a pending branch report goes out with the next tick. */
    }

    return;
}

static void
update_stats (line_ctrl_out_t const * p_ctrl,
              line_recov_out_t const * p_recov,
              int32_t dark_centre,
              int32_t dark_right,
              int32_t dt_ms)
{
    g_lb_stats.line_ticks++;
    g_lb_stats.line_dark_centre   = dark_centre;
    g_lb_stats.line_dark_right = dark_right;
    g_lb_stats.line_err         = p_ctrl->err;
    g_lb_stats.line_steer       = p_recov->steer;
    /* Casts: the enum value is 0 to 2 and dt_ms is 1 to LB_DT_MAX_MS, so both
     * are non-negative and fit uint32_t. */
    g_lb_stats.line_mode        = (uint32_t)p_recov->mode;

    if ((uint32_t)dt_ms > g_lb_stats.line_dt_max_ms)
    {
        g_lb_stats.line_dt_max_ms = (uint32_t)dt_ms;
    }
    else
    {
        /* Not a new maximum. */
    }


    return;
}

/*** end of file ***/
