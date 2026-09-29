/*
 * wifi_mqtt.h - WiFi / MQTT (owner: Jun Kai)
 * Covers FR-14, FR-15, FR-17, FR-19, NFR-11.
 * Owns the CYW43/WiFi connection and MQTT client. Other subsystems
 * (mainly telemetry) call wifi_mqtt_publish() rather than touching the
 * radio directly, so there's a single owner for the connection state.
 */
#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdbool.h>
#include <stddef.h>

// Call once at startup (after stdio_init_all()). Brings up cyw43_arch,
// connects to WiFi, and connects the MQTT client.
// TODO(Jun Kai): return/report failure so main.c can decide whether to
// keep retrying or run "offline" (telemetry disabled, everything else OK).
void wifi_mqtt_init(void);

// Call once per main-loop iteration to service the WiFi/MQTT stack
// (cyw43_arch_poll() etc.) and handle reconnects.
void wifi_mqtt_update(void);

// Publish a telemetry payload. Must not block the caller for long -
// see NFR-11. Returns true if the message was handed off successfully.
// TODO(Jun Kai): define the actual MQTT topic(s) and QoS used.
bool wifi_mqtt_publish(const char *payload, size_t len);

// TODO(Jun Kai): expose a way for incoming MQTT commands (e.g. remote
// E-stop / resume, FR-17) to reach mission_state, e.g. a callback
// registered from main.c or a direct call into mission_state.

#endif // WIFI_MQTT_H
