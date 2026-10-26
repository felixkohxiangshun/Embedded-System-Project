/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/<subsystem>/<subsystem>.h).
 *
 *	Group 11 robo car, micro T-Kernel 3.0 on Raspberry Pi Pico W + Robo Pico.
 *	Design reference: Week 6 Design Review (final).  Ownership: TEAM.md.
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */
#include <sys/machine.h>	/* in_w/out_w/set_w/clr_w and RP2040 registers */

/* ---------------------------------------------------------------------
 *  Pins (design review section 2, "Hardware Integration Pin Layout").
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  GP0/GP1 stay free (they are UART0 only with "make CRASH_REPORT=1").
 *  Every pin is claimed in TEAM.md section 2.
 * --------------------------------------------------------------------- */
#define PIN_ENC_LEFT_A		2	/* quadrature encoder, left motor  */
#define PIN_ENC_LEFT_B		3
#define PIN_ENC_RIGHT_A		6	/* quadrature encoder, right motor */
#define PIN_ENC_RIGHT_B		7
#define PIN_MOT_LEFT_FWD	8	/* PWM slice 4 channel A */
#define PIN_MOT_LEFT_BWD	9	/* PWM slice 4 channel B */
#define PIN_MOT_RIGHT_FWD	10	/* PWM slice 5 channel A */
#define PIN_MOT_RIGHT_BWD	11	/* PWM slice 5 channel B */
#define PIN_SERVO_PWM		12	/* PWM slice 6 channel A */
#define PIN_IMU_SDA		4	/* I2C0 (set in i2c_cnf_sysdep.h) */
#define PIN_IMU_SCL		5
#define PIN_ULTRA_ECHO		16
#define PIN_ULTRA_TRIG		17
#define PIN_LED_STATUS		18
#define PIN_BTN_START		20	/* active low */
#define PIN_BTN_2		21	/* unused so far */
#define PIN_IR_LEFT		26	/* ADC0 */
#define PIN_IR_CENTER		27	/* ADC1 */
#define PIN_IR_RIGHT		28	/* ADC2 (barcode) */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task, so application
 *  priorities start at 12.  The design review ranks the tasks 1..5; the
 *  same order is kept, shifted into the application band:
 *
 *      review "Highest (1)"  -> 12   Motor
 *      review "High (2)"     -> 13   Line Tracking
 *      review "Medium (3)"   -> 14   Mission
 *      review "Medium (4)"   -> 15   IMU          (10 ms)
 *                               16   Barcode      (event, drains mbf_edge)
 *                               17   Obstacle Scan (60 ms)
 *      review "Low (5)"      -> 18   Telemetry    (500 ms)
 *                               19   Dashboard
 *
 *  Every task has its own level (Tribunal 1): the kernel does not
 *  time-slice tasks of equal priority, so a tie would let one task hold
 *  the CPU until it blocks.  Inside one review rank the shorter period
 *  goes first (rate monotonic).  TEAM.md section 4 has the full table.
 * --------------------------------------------------------------------- */
#define PRI_MOTOR		12
#define PRI_LINE		13
#define PRI_MISSION		14
#define PRI_IMU			15
#define PRI_BARCODE		16
#define PRI_SCAN		17
#define PRI_TELEMETRY		18
#define PRI_DASHBOARD		19

#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    <subsystem>_init()   create this subsystem's kernel objects
 *    <subsystem>_start()  create and start this subsystem's tasks
 *    <subsystem>_test()   on-target tests, only in test builds (make TEST=1)
 *
 *    motor_control/     Motor, encoders, button ISR      (Dave)
 *    line_barcode/      IR sampler, Line, Barcode        (Vania)
 *    imu_terrain/       IMU hump and shock               (Sook Yee)
 *    dashboard/         Telemetry, WiFi / MQTT status   (Jun Kai)
 *    mission_obstacle/  Mission, Obstacle scan          (Felix)
 * --------------------------------------------------------------------- */
IMPORT ER motor_control_init(void);
IMPORT ER line_barcode_init(void);
IMPORT ER imu_terrain_init(void);
IMPORT ER dashboard_init(void);
IMPORT ER mission_obstacle_init(void);

IMPORT ER motor_control_start(void);
IMPORT ER line_barcode_start(void);
IMPORT ER imu_terrain_start(void);
IMPORT ER dashboard_start(void);
IMPORT ER mission_obstacle_start(void);

#if APP_TEST
IMPORT void motor_control_test(void);
IMPORT void line_barcode_test(void);
IMPORT void imu_terrain_test(void);
IMPORT void dashboard_test(void);
IMPORT void mission_obstacle_test(void);
#endif

/* GPIO bank 0 hook.  The RP2040 has ONE interrupt line for all of bank 0, owned
   by motor_control/gpio_isr.c (encoders, button).  That handler calls this
   function for the ultrasonic ECHO pin (GP16); it is defined in
   mission_obstacle/drv_ultra.c.  Runs in interrupt context: no blocking
   calls.  now_us = APP_NOW_US() taken at handler entry; edges = ECHO_EDGE_xxx. */
#define ECHO_EDGE_RISE		0x1
#define ECHO_EDGE_FALL		0x2
IMPORT void ultra_echo_isr(UW now_us, UW edges);

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

/* Helpers for the IPC objects (defined in app_main.c).  Message buffers are
   created by the subsystem that receives; 'count' messages of 'maxmsz'
   bytes fit before a send is refused. */
IMPORT ID app_make_mbf(INT maxmsz, INT count);
IMPORT ID app_make_flg(void);

/* Send/receive one struct.  A send never blocks (TMO_POL), so it is safe in
   handlers and cannot stall a control loop; a full buffer returns E_TMOUT. */
#define MBF_SEND(id, m)		tk_snd_mbf((id), (void*)&(m), (INT)sizeof(m), TMO_POL)
#define MBF_RECV(id, m, tmo)	tk_rcv_mbf((id), (void*)&(m), (tmo))

/* Hardware timer, 1 MHz free running (microseconds, wraps every 71 min) */
#define APP_NOW_US()		((UW)in_w(TIMER_TIMELR))

/* The kernel releases the PWM block from reset only when it uses PWM for its
   own tick (USE_PTMR=1).  It does not, so every PWM user releases it here.
   Doing it twice is harmless. */
static inline void app_pwm_unreset(void)
{
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0) {
		;
	}
}

#include "app_if.h"

#endif /* APP_H */
