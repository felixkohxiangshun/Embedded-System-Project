/*
 *	line_barcode_test.c
 *	On-target tests for the Line/barcode subsystem (IR sampler, Line, Barcode).
 *	Built only with: make TEST=1.  Results: testing/line_barcode/TEST_LOG.md
 */

#include "line_barcode.h"
#include "../test_report.h"

#if APP_TEST

#define NARROW_US	5000
#define WIDE_US		12500

/* Build the 30 edge times of "*X*" from the Code 39 element patterns */
LOCAL void make_edges(const char *start, const char *data, const char *stop,
		      UW *t)
{
	const char *parts[3] = { start, data, stop };
	INT  n = 0, p, i;
	UW   now = 1000;

	t[n++] = now;				/* first bar begins */
	for (p = 0; p < 3; p++) {
		for (i = 0; i < 9; i++) {
			now += (parts[p][i] == 'W') ? WIDE_US : NARROW_US;
			t[n++] = now;
		}
		if (p < 2) {
			now += NARROW_US;	/* inter-character gap */
			t[n++] = now;
		}
	}
}

LOCAL INT decode_char(const char *data, UB *out)
{
	UW t[BC_NUM_EDGES];

	make_edges("NWNNWNWNN", data, "NWNNWNWNN", t);
	return bc_decode(t, BC_NUM_EDGES, out);
}

EXPORT void line_barcode_test(void)
{
	UW  t[BC_NUM_EDGES];
	UB  ch = 0;
	UW  t0;
	INT n, err;

	TEST_INIT();

	/* S2-U01..04: decode each command character (FR-03) */
	TEST_CHECK("S2-U01", decode_char("WNNNNWNNW", &ch) == BCD_OK && ch == 'A', "decode *A*");
	TEST_CHECK("S2-U02", decode_char("NNWNNWNNW", &ch) == BCD_OK && ch == 'B', "decode *B*");
	TEST_CHECK("S2-U03", decode_char("WNWNNWNNN", &ch) == BCD_OK && ch == 'C', "decode *C*");
	TEST_CHECK("S2-U04", decode_char("NNNNWWNNW", &ch) == BCD_OK && ch == 'D', "decode *D*");

	/* S2-U05: speed independence -- the same code at half speed */
	make_edges("NWNNWNWNN", "WNNNNWNNW", "NWNNWNWNN", t);
	for (n = 1; n < BC_NUM_EDGES; n++) {
		t[n] = t[0] + (t[n] - t[0]) * 2;
	}
	TEST_CHECK("S2-U05", bc_decode(t, BC_NUM_EDGES, &ch) == BCD_OK && ch == 'A',
		   "decoding does not depend on car speed");

	/* S2-U06: bad inputs are rejected, never decoded wrongly */
	make_edges("NWNNWNWNN", "WNNNNWNNW", "NWNNWNWNN", t);
	TEST_CHECK("S2-U06a", bc_decode(t, BC_NUM_EDGES - 1, &ch) == BCD_ERR_COUNT, "29 edges rejected");
	t[5] = t[4] + 100;			/* squash one element */
	err = bc_decode(t, BC_NUM_EDGES, &ch);
	TEST_CHECK("S2-U06b", err != BCD_OK, "a corrupted width is rejected");
	TEST_CHECK("S2-U06c", decode_char("NNNNNNWWW", &ch) != BCD_OK, "unknown pattern rejected");

	/* S2-U07: line error sign and "off the line" */
	{
		INT e;

		TEST_CHECK("S2-U07a", line_error(1000, 0, &e) && e < 0, "line under left sensor -> error < 0");
		TEST_CHECK("S2-U07b", line_error(0, 1000, &e) && e > 0, "line under centre sensor -> error > 0");
		TEST_CHECK("S2-U07c", line_error(1000, 1000, &e) && e == 0, "line between sensors -> error 0");
		TEST_CHECK("S2-U07d", !line_error(10, 10, &e), "both sensors on white -> line lost");
	}

	/* S2-U08: PD output is limited to +/-STEER_MAX */
	{
		LINE_PD pd;

		line_pd_init(&pd);
		TEST_CHECK("S2-U08", line_pd_step(&pd, 100000) == STEER_MAX, "steer is clamped");
	}

	/* S2-U09: the IR sampler runs at 1 kHz (NFR-04): ~1000 in 1 s */
	t0 = s2_samples;
	tk_dly_tsk(1000);
	n = (INT)(s2_samples - t0);
	TEST_CHECK("S2-U09", n >= 950 && n <= 1050, "IR sampler handler runs at 1 kHz");

	/* S2-U10: the Line task runs every 10 ms (NFR-03) */
	t0 = s2_line_ticks;
	tk_dly_tsk(1000);
	n = (INT)(s2_line_ticks - t0);
	TEST_CHECK("S2-U10", n >= 95 && n <= 105, "line task runs every 10 ms");

	/* INT-01 (IF-02): 30 edges injected into mbf_edge are decoded by the
	   Barcode task.  The real right sensor must be on white. */
	{
		MSG_EDGE e;
		UW       base = s2_bc_decoded;
		UW       tt[BC_NUM_EDGES];

		make_edges("NWNNWNWNN", "NNWNNWNNW", "NWNNWNWNN", tt);	/* 'B' */
		for (n = 0; n < BC_NUM_EDGES; n++) {
			e.t_us = tt[n];
			e.dark = (UB)((n & 1) == 0);	/* bar starts dark */
			MBF_SEND(mbf_edge, e);
		}
		tk_dly_tsk(300);
		TEST_CHECK("INT-01", s2_bc_decoded == base + 1 && s2_bc_last == 'B',
			   "edges on mbf_edge -> Barcode task decodes 'B'");
	}

	TEST_SUMMARY("line_barcode");
}

#endif /* APP_TEST */
