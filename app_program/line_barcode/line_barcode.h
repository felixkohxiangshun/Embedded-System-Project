/*
 *	line_barcode.h
 *	LINE FOLLOWING AND BARCODE subsystem -- owner Vania Teng (2503206).
 *	Design review: section 6.1, FR-01..FR-03, NFR-03..NFR-05.
 *
 *	Does:         follows the black line with the left and centre IR sensors
 *	              (PD control) and decodes Code 39 "*X*" barcodes (A..D) read
 *	              by the right IR sensor while the car keeps following the line.
 *	Tasks:        line_task    (PRI_LINE, periodic 10 ms)
 *	                states SAMPLING, STEERING, LINE_LOST
 *	              barcode_task (PRI_BARCODE, event driven by mbf_edge)
 *	                states WAITING, CAPTURING, DECODING, POST_COMMAND, DECODE_FAILED
 *	Handler:      ir_sampler_handler, cyclic 1 ms (NFR-04, >= 1 kHz)
 *	Peripheral:   the ADC (GP26 left, GP27 centre, GP28 right).  There is ONE
 *	              converter behind a multiplexer, so exactly one owner reads it:
 *	              the 1 ms sampler handler, for all three sensors.  The kernel's
 *	              own ADC driver is switched off in config_device.h.  No task
 *	              touches the ADC; line_task reads the filtered values.
 *	Objects:      mbf_edge (IF-02), cyc_ir, cyc_line
 *	Publishes:    MSG_STEER on mbf_steer (IF-01), MSG_BCCMD on mbf_bccmd (IF-03),
 *	              both to Mission.  Neither task commands the motors.
 */

#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include "../app.h"
#include "line_ctrl.h"
#include "barcode_decode.h"

IMPORT ID	tid_line;
IMPORT ID	tid_barcode;
IMPORT ID	cyc_ir;
IMPORT ID	cyc_line;

/* Observable counters, used by the tests */
IMPORT volatile UW s2_samples;		/* sampler handler runs */
IMPORT volatile UW s2_edge_drops;	/* edges lost because mbf_edge was full */
IMPORT volatile UW s2_line_ticks;
IMPORT volatile UB s2_line_state;	/* LN_xxx */
IMPORT volatile UB s2_bc_state;		/* BCS_xxx */
IMPORT volatile UB s2_bc_last;
IMPORT volatile UW s2_bc_decoded;
IMPORT volatile UW s2_bc_failed;

/* drv_ir.c: ADC driver and the 1 kHz sampler (the only ADC user) */
IMPORT void ir_hw_init(void);
IMPORT void ir_sampler_handler(void *exinf);
IMPORT INT  ir_filtered(INT channel);		/* 0 left, 1 centre, 2 right */

/* task_line.c / task_barcode.c */
IMPORT void line_task(INT stacd, void *exinf);
IMPORT void line_cyclic_handler(void *exinf);
IMPORT void barcode_task(INT stacd, void *exinf);

/* Line task states */
#define LN_SAMPLING	0
#define LN_STEERING	1
#define LN_LOST		2

/* Barcode task states */
#define BCS_WAITING	0
#define BCS_CAPTURING	1
#define BCS_DECODING	2
#define BCS_POST	3
#define BCS_FAILED	4

#endif /* LINE_BARCODE_H */
