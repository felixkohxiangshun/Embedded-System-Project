/*
 *	motor_control.h
 *	MOTOR CONTROL subsystem -- owner Tan Kun Yi Dave (2502730).
 *	Design review: section 6.3 (Motor statechart), FR-11..FR-13, NFR-14, NFR-15.
 *
 *	Does:         PID speed regulation of both wheels every 10 ms, encoder-based
 *	              distance (FR-12), straight-line drift trim (FR-13), 90/180
 *	              degree pivots and straight moves by encoder count (the bypass
 *	              legs), and the start button.  States: STOPPED, DRIVING,
 *	              TURNING, ENCODER_FAULT.
 *	Task:         motor_task (PRI_MOTOR, highest, woken by cyc_motor every 10 ms)
 *	Peripherals:  PWM slices 4 and 5 (GP8..GP11); GPIO interrupt bank
 *	              (encoders GP2/GP3/GP6/GP7 and BTN_START GP20)
 *	Receives:     MSG_NAV on mbf_nav (IF-08) and START/STOP/RESUME on flg_evt
 *	              (IF-10) from Mission
 *	Reports:      speed, distance, motor state, command-done counter in
 *	              app_state (IF-11); button press to Mission via EVT_BTN
 *
 *	This header is the subsystem's public interface; the rest stays local to
 *	its .c files.  Other subsystems only need app_if.h.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "../app.h"
#include "motor_ctrl.h"

/* IF-08 / IF-10, declared in app_if.h, created in motor_control_init() */
/* ID mbf_nav; ID flg_evt; */

/* ENC_CNT: signed quadrature counts since boot, updated by the GPIO ISR,
   read by the Motor task (32-bit aligned reads are atomic on Cortex-M0+). */
IMPORT volatile INT enc_cnt[2];		/* [0] = left, [1] = right */
IMPORT volatile UW  s1_btn_presses;	/* debounced presses seen by the ISR */
IMPORT volatile UW  s1_isr_count;	/* GPIO interrupts handled */

/* Private to the Motor control subsystem, shared between its own .c files */
IMPORT ID	tid_motor;
IMPORT ID	cyc_motor;
IMPORT volatile UW s1_ticks;		/* motor loop iterations (tests) */
IMPORT volatile UB s1_state;		/* MOT_xxx (tests) */

IMPORT void motor_task(INT stacd, void *exinf);
IMPORT void motor_cyclic_handler(void *exinf);

/* drv_motor.c: PWM driver, called only from motor_task */
IMPORT void motor_hw_init(void);
IMPORT void motor_drive(INT side, INT duty_permille);	/* 0 left, 1 right */
IMPORT void motor_stop_all(void);

/* gpio_isr.c: encoder and button interrupt */
IMPORT ER   gpio_isr_start(void);

#endif /* MOTOR_CONTROL_H */
