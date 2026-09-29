#include "obstacle_scan.h"
#include "config.h"
#include "hardware/pwm.h"

// TODO(Felix): placeholder. Remember PIN_ULTRA_ECHO must come through
// the 1k/2k divider in hardware before reaching this pin - see
// config.h and the wiring sketch.

void obstacle_scan_init(void) {
    // TODO(Felix): set up servo PWM, TRIG/ECHO GPIO
}

void obstacle_scan_update(void) {
    // TODO(Felix): sweep + measure + report
}
