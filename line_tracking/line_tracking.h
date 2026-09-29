/*
 * line_tracking.h - IR Line Following (owner: Vania)
 * Covers FR-01, FR-09, FR-10, NFR-03, NFR-10.
 */
#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

void line_tracking_init(void);

// Target: sample IR sensors at least every 10ms (NFR-03).
// TODO(Vania): read PIN_IR_LEFT/CENTER/RIGHT_AOUT via ADC, compute
// steering correction, and signal barcode.c when the right sensor
// crosses into a barcode zone (see the barcode-trigger edge in the
// redrawn Task Interaction Diagram).
void line_tracking_update(void);

#endif // LINE_TRACKING_H
