/*
 *	app_main.c
 *	START HERE.  usermain() below is the program's entry point.
 *
 *	How micro T-Kernel starts the code
 *	----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing created here runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (<subsystem>_init)
 *	  2. every subsystem starts its tasks            (<subsystem>_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The robo car: one subsystem per folder (design review, final)
 *	-------------------------------------------------------------
 *	  motor_control/  Motor        PID speed loop, encoders, start button ISR
 *	  line_barcode/  IR sensors   1 kHz ADC sampler, Line task, Barcode task
 *	  imu_terrain/  IMU          hump detection, lifted (upside down) stop
 *	  dashboard/  Dashboard    telemetry task, WiFi / MQTT status, JSON, commands
 *	  mission_obstacle/  Mission      system statechart, Obstacle Scan
 *
 *	They talk only through kernel objects (message buffers, one event flag,
 *	one mutex-protected shared state); see app_if.h and TEAM.md section 5.
 *	The owner of the data creates the object; the other side just uses it.
 *
 *	Output appears on the console (USB serial, or UART with CONSOLE=uart),
 *	115200 8N1, in any terminal.
 */

#include "app.h"

/* 1: TEMP bench build (2026-11-17) for the ultrasonic test.  Every
   subsystem still creates its kernel objects (no hardware is touched in
   _init), but only Dashboard (WiFi / MQTT) and Mission/obstacle start their
   tasks.  Motor, Line/Barcode and IMU never run, so no motor PWM, no
   1 kHz IR sampler and no I2C.  The one GPIO bank-0 interrupt handler
   (motor_control/gpio_isr.c) is still installed, because the ultrasonic
   ECHO edges arrive through it; encoder counts and button presses it sees
   go nowhere.  Restore 0. */
#ifndef APP_ONLY_OBSTACLE
#define APP_ONLY_OBSTACLE	0
#endif

#if APP_ONLY_OBSTACLE
IMPORT ER gpio_isr_start(void);		/* motor_control/gpio_isr.c */
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

/* tk_cre_mbf wrapper: room for 'count' messages of 'maxmsz' bytes.  Each
   message also stores a small header, hence the +8. */
EXPORT ID app_make_mbf(INT maxmsz, INT count)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.maxmsz = maxmsz;
	cmbf.bufsz  = count * (((maxmsz + 3) & ~3) + 8);
	return tk_cre_mbf(&cmbf);
}

/* tk_cre_flg wrapper: several tasks may wait on the same flag */
EXPORT ID app_make_flg(void)
{
	T_CFLG	cflg = {0};

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	return tk_cre_flg(&cflg);
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== micro T-Kernel on Raspberry Pi Pico W"
		  " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(motor_control_init(), "motor_control_init");
	CHECK(line_barcode_init(), "line_barcode_init");
	CHECK(imu_terrain_init(), "imu_terrain_init");
	CHECK(dashboard_init(), "dashboard_init");
	CHECK(mission_obstacle_init(), "mission_obstacle_init");

	/* Phase 2: every subsystem starts its tasks */
#if APP_ONLY_OBSTACLE
	CHECK(gpio_isr_start(), "gpio_isr_start");	/* ECHO edges */
	CHECK(dashboard_start(), "dashboard_start");
	CHECK(mission_obstacle_start(), "mission_obstacle_start");
	tm_printf((UB*)"APP_ONLY_OBSTACLE: only dashboard and"
		  " mission_obstacle started\n\n");
#else
	CHECK(motor_control_start(), "motor_control_start");
	CHECK(line_barcode_start(), "line_barcode_start");
	CHECK(imu_terrain_start(), "imu_terrain_start");
	CHECK(dashboard_start(), "dashboard_start");
	CHECK(mission_obstacle_start(), "mission_obstacle_start");

	tm_printf((UB*)"all subsystems started\n\n");
#endif

#if APP_TEST
	tm_printf((UB*)"[TEST] === test run start ===\n");
	motor_control_test(); 
	line_barcode_test();
	imu_terrain_test(); 
	dashboard_test(); 
	mission_obstacle_test(); 
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
