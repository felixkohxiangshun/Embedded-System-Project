#include "telemetry.h"
#include "config.h"
#include "pico/stdlib.h"

// TODO(Felix): this is a placeholder. Replace with real telemetry packaging:
//  - build a compact status payload (mission state, speed, IMU flags,
//    obstacle distance, battery/E-stop status)
//  - rate-limit publishing to the NFR-12 interval
//  - hand the payload to wifi_mqtt_publish() (see wifi_mqtt.h)
//
// See NFR-11 (telemetry must not block/stall the control loop) and
// NFR-12 (max publish latency) in the design review for the numeric
// targets this needs to hit.

void telemetry_init(void) {
    // TODO(Felix): init any buffers/timers needed for rate limiting
}

void telemetry_update(void) {
    // TODO(Felix): build payload, then call wifi_mqtt_publish(...)
}
