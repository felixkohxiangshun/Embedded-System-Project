/*
 *	gpio_isr.c
 *	Motor control -- GPIO interrupt: quadrature encoders and the start button.
 *
 *	The RP2040 has ONE interrupt line for all of GPIO bank 0 (IO_IRQ_BANK0,
 *	NVIC number 13), so a single handler serves
 *	    GP2 / GP3  left  encoder A / B   (both edges)
 *	    GP6 / GP7  right encoder A / B   (both edges)
 *	    GP20       BTN_START             (falling edge, active low)
 *	The handler never blocks: it updates ENC_CNT, or sets EVT_BTN in flg_evt
 *	(tk_set_flg is allowed in a handler).
 *
 *	IO_BANK0 register layout used here (RP2040 datasheet 2.19.6):
 *	  INTR0..3        raw status, write 1 to clear the edge bits
 *	  PROC0_INTE0..3  interrupt enable for core 0
 *	  PROC0_INTS0..3  status after enable
 *	Each register covers 8 pins, 4 bits per pin:
 *	  bit0 level low, bit1 level high, bit2 edge low (falling), bit3 edge high
 *
 *	The interrupt is enabled from motor_control_start(), which runs in the
 *	initial task on core 0, so PROC0 registers match the core that is
 *	enabled in the NVIC.
 */

#include "motor_control.h"

#define IRQ_IO_BANK0		13
#define IRQ_PRIORITY		2

#define IO_INTR(n)		(IO_BANK0_BASE + 0x0F0 + 4 * (n))
#define IO_PROC0_INTE(n)	(IO_BANK0_BASE + 0x100 + 4 * (n))
#define IO_PROC0_INTS(n)	(IO_BANK0_BASE + 0x120 + 4 * (n))

#define EDGE_FALL		0x4
#define EDGE_RISE		0x8
#define PIN_BITS(pin, bits)	((UW)(bits) << (4 * ((pin) & 7)))

#define BTN_DEBOUNCE_US		200000

EXPORT volatile INT enc_cnt[2];
EXPORT volatile UW  s1_btn_presses;
EXPORT volatile UW  s1_isr_count;

LOCAL UB enc_prev[2];
LOCAL UW btn_last_us;

/* Ultrasonic ECHO (GP16, Felix) is also on this bank: pins 16..23 are in register 2.
   Added for Tribunal 1 (interrupt-driven echo); the edges are handed to
   ultra_echo_isr() in mission_obstacle/drv_ultra.c, which enables them itself. */
#define ECHO_MASK2		PIN_BITS(PIN_ULTRA_ECHO, 0xF)

/* Pins 0..7 are in register 0, pins 16..23 in register 2 */
#define ENC_MASK0	(PIN_BITS(PIN_ENC_LEFT_A, 0xF) | PIN_BITS(PIN_ENC_LEFT_B, 0xF) | \
			 PIN_BITS(PIN_ENC_RIGHT_A, 0xF) | PIN_BITS(PIN_ENC_RIGHT_B, 0xF))
#define BTN_MASK2	PIN_BITS(PIN_BTN_START, 0xF)

LOCAL void gpio_bank0_handler(UINT intno)
{
	UW  s0, s2, in;
	UW  t_us;
	UB  cur;
	INT d;

	(void)intno;
	t_us = APP_NOW_US();		/* stamp first: echo edges must not wait for the encoders */
	s1_isr_count++;

	s0 = in_w(IO_PROC0_INTS(0));
	s2 = in_w(IO_PROC0_INTS(2));
	out_w(IO_INTR(0), s0);		/* acknowledge the edges we saw */
	out_w(IO_INTR(2), s2);

	if (s0 & ENC_MASK0) {
		in = in_w(GPIO_IN);	/* sample both wheels at once */

		cur = (UB)((((in >> PIN_ENC_LEFT_A) & 1) << 1) | ((in >> PIN_ENC_LEFT_B) & 1));
		d = qdec_step(enc_prev[0], cur) * ENC_SIGN_LEFT;
		enc_prev[0] = cur;
		enc_cnt[0] += d;

		cur = (UB)((((in >> PIN_ENC_RIGHT_A) & 1) << 1) | ((in >> PIN_ENC_RIGHT_B) & 1));
		d = qdec_step(enc_prev[1], cur) * ENC_SIGN_RIGHT;
		enc_prev[1] = cur;
		enc_cnt[1] += d;
	}

	if (s2 & ECHO_MASK2) {
		UW edges = 0;

		if (s2 & PIN_BITS(PIN_ULTRA_ECHO, EDGE_RISE)) {
			edges |= ECHO_EDGE_RISE;
		}
		if (s2 & PIN_BITS(PIN_ULTRA_ECHO, EDGE_FALL)) {
			edges |= ECHO_EDGE_FALL;
		}
		ultra_echo_isr(t_us, edges);
	}

	if (s2 & BTN_MASK2) {
		UW now = APP_NOW_US();

		if ((UW)(now - btn_last_us) > BTN_DEBOUNCE_US) {
			btn_last_us = now;
			s1_btn_presses++;
			tk_set_flg(flg_evt, EVT_BTN);
		}
	}
}

LOCAL void input_pullup(UINT pin)
{
	gpio_set_pin(pin, GPIO_MODE_IN);
	set_w(GPIO(pin), GPIO_PUE);	/* encoder/button lines idle high */
}

EXPORT ER gpio_isr_start(void)
{
	T_DINT	dint = {0};
	UW	in;
	ER	er;

	input_pullup(PIN_ENC_LEFT_A);
	input_pullup(PIN_ENC_LEFT_B);
	input_pullup(PIN_ENC_RIGHT_A);
	input_pullup(PIN_ENC_RIGHT_B);
	input_pullup(PIN_BTN_START);

	in = in_w(GPIO_IN);
	enc_prev[0] = (UB)((((in >> PIN_ENC_LEFT_A) & 1) << 1) | ((in >> PIN_ENC_LEFT_B) & 1));
	enc_prev[1] = (UB)((((in >> PIN_ENC_RIGHT_A) & 1) << 1) | ((in >> PIN_ENC_RIGHT_B) & 1));

	/* clear anything latched, then enable the edges we care about */
	out_w(IO_INTR(0), 0xFFFFFFFF);
	out_w(IO_INTR(2), 0xFFFFFFFF);
	out_w(IO_PROC0_INTE(0),
	      in_w(IO_PROC0_INTE(0)) |
	      PIN_BITS(PIN_ENC_LEFT_A,  EDGE_FALL | EDGE_RISE) |
	      PIN_BITS(PIN_ENC_LEFT_B,  EDGE_FALL | EDGE_RISE) |
	      PIN_BITS(PIN_ENC_RIGHT_A, EDGE_FALL | EDGE_RISE) |
	      PIN_BITS(PIN_ENC_RIGHT_B, EDGE_FALL | EDGE_RISE));
	out_w(IO_PROC0_INTE(2),
	      in_w(IO_PROC0_INTE(2)) | PIN_BITS(PIN_BTN_START, EDGE_FALL));

	/* tk_def_int: register the handler for the NVIC line */
	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_bank0_handler;
	er = tk_def_int(IRQ_IO_BANK0, &dint);
	if (er < E_OK) {
		return er;
	}
	ClearInt(IRQ_IO_BANK0);
	EnableInt(IRQ_IO_BANK0, IRQ_PRIORITY);
	return E_OK;
}
