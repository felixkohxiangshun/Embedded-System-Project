#include "line_tracking.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

// TODO(Vania): placeholder. Remember: these sensors are read via ADC
// (PIN_IR_LEFT_AOUT / PIN_IR_CENTER_AOUT / PIN_IR_RIGHT_AOUT), not as
// plain digital inputs - see config.h's comment on why.

void line_tracking_init(void) {
    // TODO(Vania): adc_init(), adc_gpio_init() on the three IR pins
}

void line_tracking_update(void) {
    // TODO(Vania): read + weight the three ADC channels, steer
}
