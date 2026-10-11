/*
 *	barcode_decode.c
 *	Line/barcode -- pure Code 39 decoder.  See barcode_decode.h.
 */

#include "line_barcode.h"

typedef struct {
	UB		ch;
	const char	*pat;	/* 9 elements, N = narrow, W = wide */
} PATTERN;

LOCAL const PATTERN patterns[] = {
	{ '*', "NWNNWNWNN" },
	{ 'A', "WNNNNWNNW" },
	{ 'B', "NNWNNWNNW" },
	{ 'C', "WNWNNWNNN" },
	{ 'D', "NNNNWWNNW" },
};
#define NUM_PATTERNS	((INT)(sizeof(patterns) / sizeof(patterns[0])))

/* Classify 9 element widths into a pattern string.  Returns FALSE unless
   exactly three elements are wide. */
LOCAL BOOL classify(const UW *w, char *pat)
{
	UW   sorted[9];
	UW   sum = 0, narrow;
	INT  i, j, wide = 0;

	for (i = 0; i < 9; i++) {
		sorted[i] = w[i];
	}
	for (i = 1; i < 9; i++) {			/* insertion sort */
		UW v = sorted[i];

		for (j = i - 1; j >= 0 && sorted[j] > v; j--) {
			sorted[j + 1] = sorted[j];
		}
		sorted[j + 1] = v;
	}
	for (i = 0; i < 6; i++) {
		sum += sorted[i];
	}
	narrow = sum / 6;
	if (narrow == 0) {
		return FALSE;
	}
	for (i = 0; i < 9; i++) {
		if (w[i] * 2 > narrow * 3) {		/* w > 1.5 * narrow */
			pat[i] = 'W';
			wide++;
		} else {
			pat[i] = 'N';
		}
	}
	pat[9] = '\0';
	return (wide == 3) ? TRUE : FALSE;
}

LOCAL INT lookup(const char *pat)
{
	INT p, i;

	for (p = 0; p < NUM_PATTERNS; p++) {
		for (i = 0; i < 9 && patterns[p].pat[i] == pat[i]; i++) {
			;
		}
		if (i == 9) {
			return (INT)patterns[p].ch;
		}
	}
	return -1;
}

EXPORT INT bc_decode(const UW *edge_us, INT n_edges, UB *out)
{
	UW   w[BC_NUM_ELEMENTS];
	char pat[10];
	INT  i, c[3];

	if (n_edges != BC_NUM_EDGES) {
		return BCD_ERR_COUNT;
	}
	for (i = 0; i < BC_NUM_ELEMENTS; i++) {
		w[i] = edge_us[i + 1] - edge_us[i];	/* wrap-safe */
	}
	/* elements 0-8 start, 9 gap, 10-18 data, 19 gap, 20-28 stop */
	if (!classify(&w[0], pat)) {
		return BCD_ERR_WIDTH;
	}
	c[0] = lookup(pat);
	if (!classify(&w[10], pat)) {
		return BCD_ERR_WIDTH;
	}
	c[1] = lookup(pat);
	if (!classify(&w[20], pat)) {
		return BCD_ERR_WIDTH;
	}
	c[2] = lookup(pat);

	if (c[0] != '*' || c[2] != '*') {
		return BCD_ERR_FRAME;
	}
	if (c[1] < 'A' || c[1] > 'D') {
		return BCD_ERR_CHAR;
	}
	*out = (UB)c[1];
	return BCD_OK;
}
