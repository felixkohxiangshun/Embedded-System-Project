/*
 * barcode.h - Barcode Detection & Decoding (owner: Vania)
 * Covers FR-02, FR-03, NFR-04.
 */
#ifndef BARCODE_H
#define BARCODE_H

void barcode_init(void);

// Event-driven: called by line_tracking.c when it detects a barcode
// edge, not on a fixed period.
// TODO(Vania): decode the Code 39 barcode from edge timing and hand the
// resulting command (Turn Left / Turn Right / Go Straight / U-Turn) to
// mission_state.c.
void barcode_on_edge_detected(void);

#endif // BARCODE_H
