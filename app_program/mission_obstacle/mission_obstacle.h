/*
 *	mission_obstacle.h
 *	MISSION AND OBSTACLE AVOIDANCE subsystem (and integration) -- owner Felix Koh
 *	(2500256).  Design review: sections 5 and 6.4, FR-07..FR-10, FR-17,
 *	FR-19, NFR-08..NFR-11.
 *
 *	Does:         runs the system statechart (INIT, IDLE, RUNNING{LINE_FOLLOW,
 *	              BARCODE_SCAN, BARCODE_CMD, HUMP_TRAVERSE, OBSTACLE_SCAN,
 *	              OBSTACLE_BYPASS, LINE_REACQUIRE}, FAULT_RECOVERY, E_STOP);
 *	              detects and profiles obstacles with the ultrasonic sensor on
 *	              the servo (five angles, -60..+60 degrees).  (The telemetry
 *	              task moved to the dashboard subsystem.)
 *	Tasks:        mission_task   (PRI_MISSION,   every 5 ms, polls its inputs)
 *	              scan_task      (PRI_SCAN,      ping timer, commands from Mission)
 *	                states MONITORING, OBSTACLE_DETECTED, MOVE_SERVO, MEASURE,
 *	                EVALUATE, POST_RESULT, SCAN_FAULT
 *	Peripherals:  HC-SR04 (GP16 echo, GP17 trigger) and servo PWM slice 6 (GP12),
 *	              both owned by scan_task; status LED (GP18) owned by mission_task
 *	Objects:      mbf_steer, mbf_bccmd, mbf_terrain, mbf_obs, mbf_scancmd,
 *	              mbf_mqttcmd, mtx_state + app_state (IF-01 .. IF-11)
 *
 *	Mission is the ONE place where events from every sensor task are
 *	combined.  It never drives the motors itself: it sends commands to the
 *	Motor task on mbf_nav.
 */

#ifndef MISSION_OBSTACLE_H
#define MISSION_OBSTACLE_H

#include "../app.h"
#include "mission_logic.h"
#include "scan_logic.h"

IMPORT ID	tid_mission;
IMPORT ID	tid_scan;
IMPORT ID	flg_echo;	/* private: echo ISR -> scan_task, bit ECHO_FLG_DONE */
#define ECHO_FLG_DONE		0x01
#define ECHO_TIMEOUT_MS		30	/* no falling edge by then = no echo */

#define MISSION_TICK_MS		5

/* Scan task states (statechart 6.4), visible for the tests */
#define SCAN_MONITORING		0
#define SCAN_OBSTACLE_DETECTED	1
#define SCAN_MOVE_SERVO		2
#define SCAN_MEASURE		3
#define SCAN_EVALUATE		4
#define SCAN_POST_RESULT	5
#define SCAN_FAULT		6

IMPORT volatile UB  s5_scan_state;
IMPORT volatile INT s5_scan_last_mm;
/* last sweep, for the dashboard: readings in mm (0 = no echo) and result */
IMPORT volatile INT s5_scan_mm[5];	/* far left .. far right (SCAN_xxx order) */
IMPORT volatile UB  s5_scan_side;	/* SCAN_SIDE_xxx */
#define SCAN_SIDE_NONE		0	/* no sweep yet */
#define SCAN_SIDE_LEFT		1
#define SCAN_SIDE_RIGHT		2
#define SCAN_SIDE_FAIL		3	/* no centre echo or no room either side */
IMPORT volatile UW  s5_mission_ticks;
IMPORT volatile UB  s5_mission_state;
IMPORT volatile UB  s5_bc_exit;		/* BCX_xxx (mission_logic.h) */

/* drv_ultra.c: only scan_task calls these */
IMPORT void ultra_hw_init(void);
IMPORT INT  ultra_ping_mm(void);
IMPORT INT  ultra_ping_raw_mm(void);	/* sensor test: no limits */
IMPORT void servo_set_us(INT us);

/* 1: servo calibration.  The car never drives: GP20 moves the servo -10 us,
   GP21 +10 us (hold for repeat), and the pulse width in us shows
   on the dashboard where the ultrasonic reading normally is.  Restore 0. */
#ifndef S5_TEST_SERVO_CAL
#define S5_TEST_SERVO_CAL	0
#endif

/* 1: stationary obstacle test.  The car never drives and GP20 does nothing.
   The ultrasonic pings all the time; an object within 30 cm (two readings
   in a row) runs the left / centre / right sweep, and the readings and the
   chosen side go to the dashboard.  The sweep repeats every few seconds
   while the object stays there.  Restore 0. */
#ifndef S5_TEST_STATIONARY
#define S5_TEST_STATIONARY	0
#endif

/* 1: ultrasonic sensor test.  The car never drives.  The sensor pings
   every 100 ms at one servo position and the distance goes to the dashboard
   ("Ultrasonic now", and the sweep field of that position) WITHOUT the
   40 mm / 2 m limits, so self-echoes show too.  GP20 moves one position
   left, GP21 one right (far left .. far right); it starts at centre.
   Restore 0. */
#ifndef S5_TEST_ULTRA
#define S5_TEST_ULTRA		0
#endif

IMPORT void mission_task(INT stacd, void *exinf);
IMPORT void scan_task(INT stacd, void *exinf);

#endif /* MISSION_OBSTACLE_H */
