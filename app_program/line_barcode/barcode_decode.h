/*
 *	barcode_decode.h
 *	Line/barcode -- pure Code 39 decoder for "*X*" where X is A, B, C or D.
 *	No hardware and no kernel calls, so it is unit-tested on a PC as well.
 *
 *	Code 39: each character is 9 elements (5 bars and 4 spaces, bars first),
 *	exactly 3 of them wide, followed by one narrow inter-character gap.
 *	A message is  * X *  = 9 + 1 + 9 + 1 + 9 = 29 elements, i.e. 30 edges
 *	(the first edge starts the first bar, the last edge ends the last bar).
 *
 *	The decoder takes the edge TIMES, never counts samples, so it does not
 *	depend on the car's speed.  Within each character, the narrow width is
 *	taken as the mean of its six shortest elements and an element is wide if
 *	it is more than 1.5 times that.
 */

#ifndef BARCODE_DECODE_H
#define BARCODE_DECODE_H

#define BC_NUM_EDGES		30
#define BC_NUM_ELEMENTS		29

#define BCD_OK			0
#define BCD_ERR_COUNT		(-1)	/* not exactly 30 edges */
#define BCD_ERR_WIDTH		(-2)	/* a character has no 3-wide pattern */
#define BCD_ERR_CHAR		(-3)	/* pattern is not * A B C or D */
#define BCD_ERR_FRAME		(-4)	/* no start / stop '*' */

/* edge_us[] are the times of the 30 edges, ascending, wrap-safe (unsigned
   differences).  On BCD_OK, *out is 'A'..'D'. */
IMPORT INT bc_decode(const UW *edge_us, INT n_edges, UB *out);

#endif /* BARCODE_DECODE_H */
