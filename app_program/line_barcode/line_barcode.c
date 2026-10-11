/*
 *	line_barcode.c
 *	Line/barcode -- creates the objects and starts the tasks and handlers.
 *
 */

#include "line_barcode.h"

EXPORT ID	mbf_edge;	/* IF-02  1 kHz handler -> Barcode */
EXPORT ID	tid_line;
EXPORT ID	tid_barcode;
EXPORT ID	cyc_ir;
EXPORT ID	cyc_line;

LOCAL ID make_cyc(FP handler, RELTIM period_ms)
{
	T_CCYC ccyc = {0};

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = handler;
	ccyc.cyctim = period_ms;
	ccyc.cycphs = period_ms;
	return tk_cre_cyc(&ccyc);
}

EXPORT ER line_barcode_init(void)
{
	/* A full barcode is 30 edges; room for two codes while the Barcode
	   task is busy. */
	mbf_edge = app_make_mbf((INT)sizeof(MSG_EDGE), 64);
	if (mbf_edge < E_OK) {
		return mbf_edge;
	}
	cyc_ir = make_cyc((FP)ir_sampler_handler, 1);		/* 1 kHz */
	if (cyc_ir < E_OK) {
		return cyc_ir;
	}
	cyc_line = make_cyc((FP)line_cyclic_handler, LINE_TICK_MS);
	return (cyc_line < E_OK) ? cyc_line : E_OK;
}

EXPORT ER line_barcode_start(void)
{
	ER er;

	ir_hw_init();

	tid_barcode = app_start_task(barcode_task, PRI_BARCODE, 0, "barcode_task");
	if (tid_barcode < E_OK) {
		return tid_barcode;
	}
	tid_line = app_start_task(line_task, PRI_LINE, 0, "line_task");
	if (tid_line < E_OK) {
		return tid_line;
	}
	er = tk_sta_cyc(cyc_ir);
	if (er < E_OK) {
		return er;
	}
	return tk_sta_cyc(cyc_line);
}
