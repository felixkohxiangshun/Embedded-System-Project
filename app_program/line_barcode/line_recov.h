/** @file line_recov.h
 *
 * @brief Recovery when the line fades, vanishes, or the car veers off it.
 *
 * @par
 * Normal tracking (mode TRACK) passes the PD steering of line_ctrl.c
 * through.  When the line is no longer seen the module reacts in stages:
 *
 *  1. BRIDGE  - a single missing tick is covered by holding the last
 *               steering (LB_LOST_MISS_TICKS).
 *  2. COAST   - the line is declared lost.  The car keeps its course: the
 *               last smoothed steering is held (so a bend is still
 *               followed) and fades out at the end.  The coast is
 *               LONG (LB_COAST_FADE_MS) if the error was small when the
 *               line vanished, because then the LINE faded while the car
 *               was still on its path.  It is SHORT (LB_COAST_VEER_MS) if
 *               the error was large, because then the CAR left the line.
 *  3. SWEEP   - the car swings toward the side where the line was last
 *               seen, on top of the remembered steering.  The swing ramps
 *               up; if nothing is found the
 *               sweep reverses and each new leg is stronger, so the car
 *               zig-zags across the place where the line must be.
 *  4. REACQUIRE - the line must be seen on LB_REACQ_TICKS consecutive ticks
 *               before normal tracking resumes (one dark speck is ignored).
 *  5. TIMEOUT - if the line is missing for LB_TIMEOUT_MS the TIMEOUT flag
 *               is raised and Mission takes over (FAULT_RECOVERY).
 *
 * @par
 * The module only produces a steering value and flags.  It never commands
 * the motors; Mission decides what to forward (TEAM.md section 5, IF-01).
 *
 * @par
 * Pure arithmetic: no hardware, no kernel calls, unit tested on a PC.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef LINE_RECOV_H
#define LINE_RECOV_H

#include <stdint.h>
#include <stdbool.h>

/** Flag bits in line_recov_out_t.flags.  They have the same values as
 *  STEER_F_LOST and STEER_F_TIMEOUT in app_if.h (checked in line_task.c). */
#define LINE_RECOV_F_LOST       0x01u
#define LINE_RECOV_F_TIMEOUT    0x02u

/** What the recovery is doing now. */
typedef enum
{
    LINE_RECOV_TRACK = 0,       /* line seen, normal PD steering */
    LINE_RECOV_COAST,           /* line lost, keeping the course */
    LINE_RECOV_SWEEP            /* line lost, searching for it */
} line_recov_mode_t;

/** State of the recovery.  Treat as private; use the functions below. */
typedef struct
{
    line_recov_mode_t mode;
    int32_t           miss_ticks;   /* consecutive ticks without the line */
    int32_t           seen_ticks;   /* consecutive ticks with the line */
    int32_t           err_mem;      /* smoothed error while tracking */
    int32_t           steer_mem;    /* smoothed steering while tracking */
    int32_t           steer_out;    /* steering sent on the previous tick */
    int32_t           side;         /* search side: -1 left, +1 right */
    int32_t           coast_ms;     /* planned length of the coast */
    int32_t           mode_ms;      /* time spent in the current mode */
    int32_t           leg_index;    /* number of the current sweep leg */
    int32_t           leg_ms;       /* time spent in the current leg */
    int32_t           lost_ms;      /* time the line has been missing */
} line_recov_t;

/** Input of one tick. */
typedef struct
{
    bool    b_seen;             /* line_ctrl says the line is visible */
    int32_t err;                /* position error from line_ctrl */
    int32_t steer;              /* PD steering from line_ctrl */
    int32_t dt_ms;              /* time since the previous tick */
    bool    b_timeout_active;   /* false: do not count lost time (the car is
                                   not expected to follow a line now) */
} line_recov_in_t;

/** Output of one tick. */
typedef struct
{
    int32_t           steer;        /* steering to send to Mission */
    uint8_t           flags;        /* LINE_RECOV_F_xxx */
    line_recov_mode_t mode;         /* mode after this tick */
    bool              b_resumed;    /* true on the tick tracking resumes */
} line_recov_out_t;

/*!
 * @brief Reset the recovery to normal tracking.
 *
 * @param[out] p_recov  Recovery to initialise; must not be NULL.
 */
void line_recov_init(line_recov_t * p_recov);

/*!
 * @brief Run one tick (every LB_LINE_TICK_MS).
 *
 * @param[in,out] p_recov  Recovery state; must not be NULL.
 * @param[in]     p_in     Input of this tick; must not be NULL.
 * @param[out]    p_out    Output of this tick; must not be NULL.
 */
void line_recov_step(line_recov_t * p_recov,
                     line_recov_in_t const * p_in,
                     line_recov_out_t * p_out);

#endif /* LINE_RECOV_H */
/*** end of file ***/
