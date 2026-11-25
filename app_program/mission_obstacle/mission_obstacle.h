/**
 * @file mission_obstacle.h
 *      MISSION AND OBSTACLE AVOIDANCE subsystem (and integration) -- owner Felix Koh
 *      (2500256).  Design review: sections 5 and 6.4, FR-07..FR-10, FR-17,
 *      FR-19, NFR-08..NFR-11.
 *
 *      Does:         runs the system statechart (INIT, IDLE, RUNNING{LINE_FOLLOW,
 *                    BARCODE_SCAN, BARCODE_CMD, HUMP_TRAVERSE, OBSTACLE_SCAN,
 *                    OBSTACLE_BYPASS, LINE_REACQUIRE}, FAULT_RECOVERY);
 *                    detects and profiles obstacles with the ultrasonic sensor on
 *                    the servo (5 degree steps, -55..+55 degrees), measures their
 *                    width and takes the car round them on a 45 degree dog-leg.
 *      Tasks:        mission_task   (PRI_MISSION,   every 5 ms, polls its inputs)
 *                    scan_task      (PRI_SCAN,      ping timer, commands from Mission)
 *                      states MONITORING, OBSTACLE_DETECTED, MOVE_SERVO, MEASURE,
 *                      EVALUATE, POST_RESULT, SCAN_FAULT
 *      Peripherals:  HC-SR04 (GP16 echo, GP17 trigger) and servo PWM slice 6 (GP12),
 *                    both owned by scan_task; status LEDs (NeoPixels on GP18, drv_rgb.c) owned by mission_task
 *      Objects:      mbf_steer, mbf_bccmd, mbf_terrain, mbf_obs, mbf_scancmd,
 *                    mbf_mqttcmd, mtx_state + app_state (IF-01 .. IF-11)
 *
 *      Mission is the ONE place where events from every sensor task are
 *      combined.  It never drives the motors itself: it sends commands to the
 *      Motor task on mbf_nav.
 */

#ifndef MISSION_OBSTACLE_H
#define MISSION_OBSTACLE_H

#include "../app.h"
#include "mission_logic.h"
#include "scan_logic.h"

IMPORT ID       tid_mission;
IMPORT ID       tid_scan;
IMPORT ID       flg_echo;       /* private: echo ISR -> scan_task, bit ECHO_FLG_DONE */
#define ECHO_FLG_DONE           0x01
#define ECHO_TIMEOUT_MS         30      /* no falling edge by then = no echo */

#define MISSION_TICK_MS         5

/* Scan task states (statechart 6.4), visible for the tests */
#define SCAN_MONITORING         0
#define SCAN_OBSTACLE_DETECTED  1
#define SCAN_MOVE_SERVO         2
#define SCAN_MEASURE            3
#define SCAN_EVALUATE           4
#define SCAN_POST_RESULT        5
#define SCAN_FAULT              6

IMPORT volatile UB  s5_scan_state;
IMPORT volatile INT s5_scan_last_mm;
/* last sweep's result, for the dashboard */
IMPORT volatile UB  s5_scan_side;       /* SCAN_SIDE_xxx */
IMPORT volatile INT s5_scan_width_mm;   /* last measured width, 0 = none */
IMPORT volatile INT s5_scan_half_mm[2]; /* left, right half widths */
#define SCAN_SIDE_NONE          0       /* no sweep yet */
#define SCAN_SIDE_LEFT          1
#define SCAN_SIDE_RIGHT         2
#define SCAN_SIDE_FAIL          3       /* no centre echo or no room either side */
IMPORT volatile UW  s5_mission_ticks;
IMPORT volatile UB  s5_mission_state;
IMPORT volatile UB  s5_bc_exit;         /* BCX_xxx (mission_logic.h) */
IMPORT volatile UB  s5_byp_result;      /* BYPR_xxx (mission_logic.h) */

/* drv_ultra.c: only scan_task calls these */
IMPORT void ultra_hw_init(void);
IMPORT INT  ultra_ping_mm(void);
IMPORT INT  ultra_ping_raw_mm(void);    /* without the 40 mm / 2 m limits */
IMPORT void servo_set_us(INT us);

/* 1: servo calibration.  The car never drives: GP20 moves the servo -10 us,
   GP21 +10 us (hold for repeat), and the pulse width in us shows
   on the dashboard where the ultrasonic reading normally is.  Restore 0. */
#ifndef S5_TEST_SERVO_CAL
#define S5_TEST_SERVO_CAL       0
#endif

IMPORT void mission_task(INT stacd, void *exinf);
IMPORT void scan_task(INT stacd, void *exinf);

#endif /* MISSION_OBSTACLE_H */

/*** end of file ***/
