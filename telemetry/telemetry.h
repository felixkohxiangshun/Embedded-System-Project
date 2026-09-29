/*
 * telemetry.h - Telemetry / Logging (owner: Felix)
 * Covers FR-16, FR-18, NFR-11, NFR-12.
 * Packages status (mission state, sensor readings, faults) for
 * publishing over MQTT via wifi_mqtt, and/or local logging.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

// Call once at startup.
void telemetry_init(void);

// Call once per main-loop iteration. Internally rate-limits itself to the
// NFR-12 publish interval - it does not need to publish every call.
// TODO(Felix): pull current state from mission_state / motor / imu_terrain
// / obstacle_scan and hand a formatted payload to wifi_mqtt_publish().
void telemetry_update(void);

#endif // TELEMETRY_H
