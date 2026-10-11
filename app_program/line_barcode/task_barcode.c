/*
 *	task_barcode.c
 *	Line/barcode -- the Barcode decoding task (design review 6.1).
 *
 *	States  WAITING    block on mbf_edge until a dark bar begins
 *	        CAPTURING  collect edge times until 30 edges or a timeout
 *	        DECODING   measure the widths and decode (barcode_decode.c)
 *	        POST_COMMAND  send the result to Mission
 *	        DECODE_FAILED bad width, bad frame or timeout: report, go on
 *
 *	The task blocks on a message buffer, so it uses no CPU between
 *	barcodes.  Line following continues while a barcode is scanned; this
 *	task only reports to Mission and never drives the motors.
 */

#include "line_barcode.h"

EXPORT volatile UB s2_bc_state;
EXPORT volatile UB s2_bc_last;		/* last decoded character, 0 = none */
EXPORT volatile UW s2_bc_decoded;
EXPORT volatile UW s2_bc_failed;

#define EDGE_TIMEOUT_MS		120	/* silence that ends a capture */

LOCAL void post(UB kind, UB cmd)
{
	MSG_BCCMD m;

	m.kind = kind;
	m.cmd = cmd;
	MBF_SEND(mbf_bccmd, m);
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	MSG_EDGE e;
	UW       t[BC_NUM_EDGES];
	INT      n = 0, r;
	UB       ch = 0;

	(void)stacd; (void)exinf;

	while (1) {
		/* WAITING */
		s2_bc_state = BCS_WAITING;
		do {
			r = MBF_RECV(mbf_edge, e, TMO_FEVR);
		} while (r <= 0 || !e.dark);	/* only a falling-to-dark edge starts a code */

		/* CAPTURING */
		s2_bc_state = BCS_CAPTURING;
		n = 0;
		t[n++] = e.t_us;
		post(BC_SEEN, 0);
		while (n < BC_NUM_EDGES) {
			r = MBF_RECV(mbf_edge, e, EDGE_TIMEOUT_MS);
			if (r <= 0) {
				break;		/* silence (E_TMOUT) ends the capture */
			}
			t[n++] = e.t_us;
		}

		/* DECODING */
		s2_bc_state = BCS_DECODING;
		r = bc_decode(t, n, &ch);

		if (r == BCD_OK) {
			s2_bc_state = BCS_POST;
			s2_bc_last = ch;
			s2_bc_decoded++;
			post(BC_OK, ch);
		} else {
			s2_bc_state = BCS_FAILED;
			s2_bc_failed++;
			post(BC_FAIL, 0);
		}
	}
}
