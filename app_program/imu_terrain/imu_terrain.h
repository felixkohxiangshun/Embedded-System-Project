/**
 * @file imu_terrain.h
 *      IMU AND TERRAIN subsystem -- owner Lau Sook Yee (2503178).
 *      Design review: section 6.2, FR-04..FR-06, NFR-06, NFR-07.
 *      Our humps are narrow: only one drive wheel climbs them.  The car leans,
 *      and the wheel's height is track width x tan(roll) (IMU_HUMP_MODE, see
 *      imu_logic.h; the Tribunal 1 pitch x encoder-distance method is kept for
 *      humps across the whole track).
 *
 *      Does:         samples the LSM303DLHC accelerometer (all 3 axes) every
 *                    10 ms, removes the board's mounting tilt (measured at rest),
 *                    works out the lifted wheel's height from the roll, reports
 *                    each hump's peak height and the
 *                    tallest of the run, and reports the car being LIFTED
 *                    (picked up: an upward speed of about 0.4 m/s, or turned
 *                    upside down for 100 ms; Mission then stops the motors).  IMU collision (shock) detection was
 *                    dropped from the requirement (2026-11-17).  A failing I2C
 *                    read is a SENSOR_FAULT that is retried; it never stops the
 *                    car (FR-19).
 *                    States: CALIBRATING, MONITORING, HUMP_TRACKING, HUMP_COMPLETE,
 *                    LIFTED, SENSOR_FAULT.
 *      Task:         imu_task (PRI_IMU, woken by cyc_imu every 10 ms)
 *      Peripheral:   I2C0 on GP4 (SDA) / GP5 (SCL) through the kernel's I2C driver
 *                    (device "iica"); sensor address 0x19.  Only imu_task uses it.
 *      Reads:        app_state.enc_left, enc_right, motor_state (IF-11, written
 *                    by Motor), under state_lock(TMO_POL): it never waits.
 *      Publishes:    MSG_TERRAIN on mbf_terrain (IF-04) to Mission
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include "../app.h"
#include "../motor_control/motor_ctrl.h"        /* ENC_COUNTS_PER_REV, WHEEL_MM_PER_REV (read only) */
#include "imu_logic.h"

/* ---- Mounting (test S3-M01) ----
   Which LSM303 axis points forward / sideways / up on the car.
   MEASURED on our car (recording 2026-11-01, testing/imu_terrain/logs):
     Z up (+880 mg at rest), Y forward (it carries the motors' fore-aft
     shaking), X sideways, and X goes POSITIVE when the RIGHT wheel climbs
     the hump.  If the board is remounted, check again with S3_TEST_LOG:
     on a hump run the side axis must move most, positive with the wheel up. */
#define IMU_AXIS_FWD            1       /* 0 = X, 1 = Y, 2 = Z */
#define IMU_SIGN_FWD            (+1)    /* only used by the full-width pitch mode [check] */
#define IMU_AXIS_UP             2
#define IMU_SIGN_UP             (+1)
#define IMU_SIGN_SIDE           (+1)    /* + = hump-side (right) wheel up */

/* ---- Fault handling ---- */
#define IMU_FAULT_READS         3       /* 3 failed reads in a row (30 ms) -> SENSOR_FAULT */
#define IMU_RETRY_TICKS         10      /* while faulty, re-open the sensor every 100 ms */
#define IMU_LOG                 1       /* 1: print each hump on the console (S3-M02) */

IMPORT ID       tid_imu;
IMPORT ID       cyc_imu;

/* Observable values for the tests */
IMPORT volatile UW  s3_ticks;
IMPORT volatile UB  s3_state;           /* IMU_xxx */
IMPORT volatile INT s3_pitch_deg;       /* pitch now, whole degrees */
IMPORT volatile INT s3_h_um;            /* height of the hump under way, um */
IMPORT volatile INT s3_residual_um;     /* left over after the last hump (~0 is good) */
IMPORT volatile INT s3_raw_mg[3];       /* last raw X, Y, Z (S3-M01) */
IMPORT volatile UB  s3_run_cmd;         /* test -> task: S3_CMD_xxx */
#define S3_CMD_NONE             0
#define S3_CMD_RESET            1       /* clear the per-run statistics */
#define S3_CMD_FINISH           2       /* car stopped: close a hump under way */
IMPORT volatile INT s3_run_peak_um;     /* highest lean height this run, um */
IMPORT volatile INT s3_run_dev[3];      /* largest change from rest this run: fwd, side, up, mg */
IMPORT volatile BOOL s3_calibrated;     /* mounting tilt measured at rest */
IMPORT volatile INT s3_rest[3];         /* rest vector fwd, side, up, mg */
IMPORT volatile UW  s3_humps;           /* humps completed since power-up */
IMPORT volatile INT s3_rise;            /* lift detector, mg x samples (LIFT_RISE_MGS trips it) */
#define S3_LOG_MAX              1500    /* 15 s of 10 ms samples, ~13 KB, test builds only */
#if APP_TEST
IMPORT volatile BOOL s3_log_on;
IMPORT volatile INT  s3_log_n;
IMPORT H  s3_log_raw[S3_LOG_MAX][3];
IMPORT H  s3_log_ds[S3_LOG_MAX];
IMPORT H  s3_log_h[S3_LOG_MAX];
IMPORT UB s3_log_st[S3_LOG_MAX];
#endif
IMPORT volatile INT s3_last_mm;         /* height of the last hump */
IMPORT volatile INT s3_max_mm;          /* tallest of the run */

/* drv_imu.c: the only code that touches the I2C device */
IMPORT ER   imu_hw_open(void);                  /* open + configure sensor */
IMPORT ER   imu_read_accel(INT mg[3]);          /* X, Y, Z acceleration in milli-g */

IMPORT void imu_task(INT stacd, void *exinf);
IMPORT void imu_cyclic_handler(void *exinf);

#endif /* IMU_TERRAIN_H */

/*** end of file ***/
