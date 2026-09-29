#include "wifi_mqtt.h"
#include "config.h"
#include "pico/stdlib.h"

// TODO(Jun Kai): this is a placeholder. Replace with real cyw43_arch +
// lwIP MQTT client setup:
//  - cyw43_arch_init(), cyw43_arch_enable_sta_mode(),
//    cyw43_arch_wifi_connect_timeout_ms(...)
//  - MQTT client connect (see pico-examples/pico_w/wifi/mqtt for a
//    working reference)
//  - wire incoming remote E-stop/resume commands (FR-17) back into
//    mission_state
//
// Remember: CMakeLists.txt already links pico_cyw43_arch_none - swap
// that for pico_cyw43_arch_lwip_poll (or _threadsafe_background) once
// this actually uses lwIP/MQTT.

void wifi_mqtt_init(void) {
    // TODO(Jun Kai): cyw43_arch_init(), WiFi connect, MQTT client connect
}

void wifi_mqtt_update(void) {
    // TODO(Jun Kai): poll cyw43_arch / MQTT client, handle reconnects
}

bool wifi_mqtt_publish(const char *payload, size_t len) {
    // TODO(Jun Kai): publish payload over MQTT
    (void)payload;
    (void)len;
    return false;
}
