/*
 *	drv_ir.c
 *	Line/barcode -- ADC driver and the 1 kHz IR sampler.
 *
 *	ir_sampler_handler() is a CYCLIC HANDLER (the kernel calls it every 1 ms
 *	from the timer interrupt; this needs CNF_TIMER_PERIOD = 1, see
 *	config/config.h).  It is the single reader of the ADC:
 *	  - left and centre sensors: single-pole low-pass filtered, read by
 *	    line_task through ir_filtered();
 *	  - right sensor (barcode): thresholded with hysteresis; every change of
 *	    level is sent with its timestamp to mbf_edge for the Barcode task.
 *	It never blocks: tk_snd_mbf is called with TMO_POL.
 *
 *	One conversion takes about 2 us (ADC clock 48 MHz, 96 cycles), so
 *	three channels cost roughly 10 us per millisecond.
 */

#include "line_barcode.h"

#define ADC_BASE		0x4004C000
#define ADC_CS			(ADC_BASE + 0x00)
#define ADC_RESULT		(ADC_BASE + 0x04)
#define ADC_CS_EN		(1u << 0)
#define ADC_CS_START_ONCE	(1u << 2)
#define ADC_CS_READY		(1u << 8)
#define ADC_CS_AINSEL(n)	((UW)(n) << 12)

/* Barcode threshold with hysteresis (12-bit counts) [CAL] */
#define BC_THRESH		1800
#define BC_HYST			300

EXPORT volatile UW s2_samples;
EXPORT volatile UW s2_edge_drops;

LOCAL volatile INT filt[3];
LOCAL UB   bar_dark;

LOCAL const UINT ir_pin[3] = { PIN_IR_LEFT, PIN_IR_CENTER, PIN_IR_RIGHT };

/* The kernel already released the ADC block from reset (USE_SDEV_DRV). */
EXPORT void ir_hw_init(void)
{
	INT i;

	for (i = 0; i < 3; i++) {
		out_w(GPIO_CTRL(ir_pin[i]), GPIO_CTRL_FUNCSEL_NULL);
		clr_w(GPIO(ir_pin[i]), GPIO_IE | GPIO_PUE | GPIO_PDE);
	}
	out_w(ADC_CS, ADC_CS_EN);
	while (!(in_w(ADC_CS) & ADC_CS_READY)) {
		;
	}
}

LOCAL INT adc_read(INT channel)
{
	out_w(ADC_CS, ADC_CS_EN | ADC_CS_AINSEL(channel) | ADC_CS_START_ONCE);
	while (!(in_w(ADC_CS) & ADC_CS_READY)) {
		;
	}
	return (INT)(in_w(ADC_RESULT) & 0xFFF);
}

EXPORT INT ir_filtered(INT channel)
{
	return filt[channel];
}

EXPORT void ir_sampler_handler(void *exinf)
{
	MSG_EDGE e;
	INT raw;
	UB  dark;

	(void)exinf;
	s2_samples++;

	/* line sensors: y += (x - y) / 2 */
	raw = adc_read(0);
	filt[0] += (raw - filt[0]) / 2;
	raw = adc_read(1);
	filt[1] += (raw - filt[1]) / 2;

	/* barcode sensor: no filter, keep the edge times sharp */
	raw = adc_read(2);
	filt[2] = raw;
#if IR_DARK_IS_HIGH
	if (bar_dark) {
		dark = (raw > BC_THRESH - BC_HYST) ? 1 : 0;
	} else {
		dark = (raw > BC_THRESH + BC_HYST) ? 1 : 0;
	}
#else
	if (bar_dark) {
		dark = (raw < BC_THRESH + BC_HYST) ? 1 : 0;
	} else {
		dark = (raw < BC_THRESH - BC_HYST) ? 1 : 0;
	}
#endif
	if (dark != bar_dark) {
		bar_dark = dark;
		e.t_us = APP_NOW_US();
		e.dark = dark;
		if (MBF_SEND(mbf_edge, e) != E_OK) {
			s2_edge_drops++;
		}
	}
}
