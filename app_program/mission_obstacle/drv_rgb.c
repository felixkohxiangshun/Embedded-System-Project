/*
 *	drv_rgb.c
 *	Status LEDs: the Robo Pico's NeoPixel (WS2812) RGB LEDs on GP18.
 *
 *	A NeoPixel takes a stream of 24-bit colours (green, red, blue, most
 *	significant bit first), one bit every 1.25 us: a short high pulse is a 0,
 *	a long one a 1.  A plain GPIO high/low (what GP18 did until 2026-11-23)
 *	means nothing to it, so the LEDs stayed dark.
 *
 *	The bits are timed by a PIO state machine (the standard 4-instruction
 *	ws2812 program from the Pico SDK examples), so the CPU only writes one
 *	word per LED into the PIO FIFO: no busy-wait and no interrupts masked.
 *
 *	The WiFi driver (cyw43) also runs a PIO program, placed by the Pico SDK
 *	(on this car it lands in PIO1).  The state machine AND the instruction
 *	memory here are taken through the same SDK call, so the SDK keeps the
 *	two programs apart; writing the program into a fixed place would
 *	overwrite the WiFi one (the SDK fills instruction memory from the top).
 */

#include "mission_obstacle.h"
#include "drv_rgb.h"

/* Pico SDK hardware_pio (linked for the WiFi driver).  Declared here rather
   than including the SDK header, which needs the SDK's whole configuration.
   pio_program_t as in SDK 2.x with PICO_PIO_VERSION 0 (RP2040); a PIO is the
   base address of the block.  The SDK relocates the JMP targets. */
typedef struct {
	const UH	*instructions;
	UB		length;
	B		origin;		/* -1: load anywhere */
	UB		pio_version;
} SDK_PIO_PROGRAM;
IMPORT UB pio_claim_free_sm_and_add_program(	/* C bool */
					      const SDK_PIO_PROGRAM *program,
					      void **pio_out, unsigned int *sm_out,
					      unsigned int *offset_out);

#define PIO0_BASE		0x50200000u
#define PIO1_BASE		0x50300000u
#define PIO_CTRL		0x000u
#define PIO_TXF(sm)		(0x010u + 4u * (UW)(sm))
#define PIO_SM(sm, off)		(0x0c8u + 0x18u * (UW)(sm) + (off))
#define SM_CLKDIV		0x00u
#define SM_EXECCTRL		0x04u
#define SM_SHIFTCTRL		0x08u
#define SM_INSTR		0x10u
#define SM_PINCTRL		0x14u
#define REG_SET			0x2000u		/* atomic set alias */
#define REG_CLR			0x3000u		/* atomic clear alias */

#define RGB_RESETS_BASE		0x4000c000u	/* RESETS: RESET, then RESET_DONE + 8 */
#define RGB_RESET_PIO(n)	(1u << (10 + (n)))
#define IO_GPIO_CTRL(pin)	(0x40014004u + 8u * (UW)(pin))
#define FUNCSEL_PIO(n)		(6u + (n))

/* ws2812.pio, side-set 1 pin, T1 2 + T2 5 + T3 3 = 10 cycles per bit.
   Addresses relative to the load offset. */
LOCAL const UH ws2812_instr[4] = {
	0x6221,			/* 0 bitloop: out x, 1       side 0 [2] */
	0x1120 | 3,		/* 1          jmp !x do_zero side 1 [1] */
	0x1400 | 0,		/* 2 do_one:  jmp bitloop    side 1 [4] */
	0xa442,			/* 3 do_zero: nop            side 0 [4] */
};
LOCAL const SDK_PIO_PROGRAM ws2812_prog = { ws2812_instr, 4, -1, 0 };

/* 800 kHz x 10 cycles = 8 MHz from the 125 MHz system clock: 15.625 */
#define CLKDIV_INT		15u
#define CLKDIV_FRAC		160u		/* 0.625 x 256 */

LOCAL UW   rgb_pio = 0;			/* PIO block base, 0 = not running */
LOCAL UW   rgb_sm;
LOCAL UW   rgb_last = 0xFFFFFFFFu;	/* last colour sent */

EXPORT ER rgb_init(void)
{
	void        *pio;
	unsigned int sm, offset;
	UW           p, n, done;
	INT          i;

	if (pio_claim_free_sm_and_add_program(&ws2812_prog, &pio, &sm, &offset) == 0) {
		return E_LIMIT;		/* no room: LEDs stay dark, nothing else affected */
	}
	p = (UW)pio;
	n = (p == PIO1_BASE) ? 1u : 0u;

	/* make sure the block is out of reset (never put it back) */
	out_w(RGB_RESETS_BASE + REG_CLR, RGB_RESET_PIO(n));
	for (i = 0, done = 0; i < 1000 && !done; i++) {
		done = in_w(RGB_RESETS_BASE + 8u) & RGB_RESET_PIO(n);
	}

	out_w(p + PIO_CTRL + REG_CLR, 1u << sm);			/* SM off */
	out_w(p + PIO_SM(sm, SM_CLKDIV), (CLKDIV_INT << 16) | (CLKDIV_FRAC << 8));
	/* wrap over the 4 instructions */
	out_w(p + PIO_SM(sm, SM_EXECCTRL), ((offset + 3u) << 12) | (offset << 7));
	/* join FIFOs for TX, autopull every 24 bits, shift left (MSB first) */
	out_w(p + PIO_SM(sm, SM_SHIFTCTRL), (1u << 30) | (24u << 25) | (1u << 17));

	/* GP18 driven by the SM, low, as an output */
	out_w(p + PIO_SM(sm, SM_PINCTRL), (1u << 26) | ((UW)PIN_LED_STATUS << 5));
	out_w(p + PIO_SM(sm, SM_INSTR), 0xe000);			/* set pins, 0 */
	out_w(p + PIO_SM(sm, SM_INSTR), 0xe081);			/* set pindirs, 1 */
	out_w(p + PIO_SM(sm, SM_PINCTRL), (1u << 29) | ((UW)PIN_LED_STATUS << 10));
	out_w(IO_GPIO_CTRL(PIN_LED_STATUS), FUNCSEL_PIO(n));

	out_w(p + PIO_SM(sm, SM_INSTR), offset);			/* jmp offset */
	out_w(p + PIO_CTRL + REG_SET, (1u << (4 + sm)) | (1u << (8 + sm)));	/* restart */
	out_w(p + PIO_CTRL + REG_SET, 1u << sm);			/* go */

	rgb_sm = sm;
	rgb_pio = p;
	rgb_set(RGB_OFF);
	return E_OK;
}

/* colour = 0xRRGGBB, the same on every LED.  Sent only when it changes;
   the LEDs latch it after 50 us without data. */
EXPORT void rgb_set(UW colour)
{
	UW grb, k;

	if (rgb_pio == 0 || colour == rgb_last) {
		return;
	}
	rgb_last = colour;
	grb = ((colour & 0x00FF00u) << 8) | ((colour & 0xFF0000u) >> 8) | (colour & 0xFFu);
	for (k = 0; k < RGB_LED_COUNT; k++) {
		/* the 8-deep FIFO never fills with RGB_LED_COUNT words */
		out_w(rgb_pio + PIO_TXF(rgb_sm), grb << 8);
	}
}
