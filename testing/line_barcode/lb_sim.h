/** @file lb_sim.h
 *
 * @brief Interface of the PC simulations (barcode path and line following).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef LB_SIM_H
#define LB_SIM_H

#include <stdint.h>
#include <stdbool.h>

/** Conditions of one simulated barcode pass. */
typedef struct
{
    double speed_mmps;      /* car speed over the code */
    double speed_wobble;    /* relative speed change while crossing (0.2) */
    double spot_mm;         /* width of the sensor spot along the travel */
    double contrast;        /* ink contrast, 1.0 = nominal black on white */
    double noise_counts;    /* ADC noise, standard deviation in counts */
    double flicker;         /* 100 Hz mains flicker, relative amplitude */
    double ambient_shift;   /* constant offset added to every reading */
    double wide_ratio;      /* printed wide / narrow ratio (ideal 3.0) */
    double jitter_us;       /* sample time jitter, microseconds */
} lb_sim_bc_cfg_t;

/** Outcome of one simulated pass. */
typedef enum
{
    LB_SIM_BC_CORRECT = 0,  /* decoded to the printed command */
    LB_SIM_BC_WRONG,        /* decoded to a DIFFERENT command (dangerous) */
    LB_SIM_BC_IGNORED,      /* valid Code 39, but not a command: ignored */
    LB_SIM_BC_NO_DECODE     /* nothing decoded (reported as a failure) */
} lb_sim_bc_result_t;

void lb_sim_seed(uint32_t seed);

/** Code 39 element pattern of a symbol ('A' to 'Z' or '*'): nine letters,
 *  N = narrow, W = wide, bar first.  NULL for any other character. */
char const * lb_sim_pattern(char symbol);
lb_sim_bc_result_t lb_sim_bc_run(lb_sim_bc_cfg_t const * p_cfg, char symbol);


/* ---------------------------------------------------------------------
 * Closed-loop line following (lb_sim_line.c)
 * --------------------------------------------------------------------- */

/** One section of the track centre line. */
typedef struct
{
    double length_mm;       /* length along the line */
    double turn_deg;        /* total turn over this section, + = left */
} lb_sim_seg_t;

/** A stretch of the line where the ink is weak or missing. */
typedef struct
{
    double start_mm;        /* distance along the line where it starts */
    double length_mm;       /* length of the full-strength weak part */
    double level;           /* ink strength there: 1.0 normal, 0.0 none */
    double ramp_mm;         /* length of the fade in and the fade out */
} lb_sim_fade_t;

/** A stretch where the car is pushed sideways (wheel slip, uneven floor). */
typedef struct
{
    double start_mm;        /* distance travelled where it starts */
    double length_mm;       /* length of the disturbance */
    double yaw_dps;         /* yaw rate added, deg/s, + = veer to the left */
} lb_sim_veer_t;

/** Conditions of one closed-loop run. */
typedef struct
{
    double speed_mmps;      /* forward speed (Motor: 120 normal, 60 slow) */
    double axle_to_sensor_mm;   /* sensors are this far ahead of the axle */
    double sensor_gap_mm;   /* distance between left and centre sensor */
    double spot_mm;         /* width of the sensor spot across the line */
    double line_mm;         /* printed line width (18 mm) */
    double track_mm;        /* distance between the wheels (130 mm) */
    double wheel_diff_mmps; /* wheel speed difference at full steer (120) */
    double lag_s;           /* yaw response time constant of the car */
    double delay_ms;        /* command delay: Mission, buffers, motor loop */
    double noise_counts;    /* ADC noise (standard deviation) */
    double flicker;         /* 100 Hz flicker, relative amplitude */
    double ambient_shift;   /* constant offset added to every reading */
    double contrast;        /* ink contrast of the whole line (1.0) */
    double start_offset_mm; /* starting position, + = car left of the line */
    double start_heading_deg;   /* starting heading, + = pointing left */
    lb_sim_seg_t const *   p_segments;  /* track */
    uint32_t               num_segments;
    lb_sim_fade_t const *  p_fades;     /* weak or missing ink */
    uint32_t               num_fades;
    lb_sim_veer_t const *  p_veers;     /* sideways pushes */
    uint32_t               num_veers;
} lb_sim_line_cfg_t;

/** Outcome of one closed-loop run. */
typedef struct
{
    double   max_offset_mm;     /* largest distance from car axis to line */
    double   rms_offset_mm;     /* rms of that distance over the whole run */
    double   final_offset_mm;   /* distance at the end */
    double   max_lost_ms;       /* longest time the line was not seen */
    double   distance_mm;       /* distance driven */
    uint32_t ticks_total;
    uint32_t ticks_coast;
    uint32_t ticks_sweep;
    uint32_t recoveries;        /* times normal tracking resumed after a loss */
    bool     b_timeout;         /* TIMEOUT flag was raised (Mission FAULT) */
    bool     b_off_track;       /* car left the line by more than 80 mm */
    bool     b_finished;        /* the end of the track was reached */
} lb_sim_line_result_t;

/** Fill in the defaults for this car (see lb_sim_line.c for the sources). */
void lb_sim_line_defaults(lb_sim_line_cfg_t * p_cfg);

/** Run one closed-loop simulation. */
void lb_sim_line_run(lb_sim_line_cfg_t const * p_cfg,
                     lb_sim_line_result_t * p_res);

#endif /* LB_SIM_H */
/*** end of file ***/
