/*
 * main.c - top-level super-loop
 *
 * This file is intentionally thin: it just brings every subsystem up
 * once (*_init) and then calls each subsystem's *_update() every loop
 * iteration. Subsystem logic belongs in that subsystem's own .c file,
 * not here - see NFR-02 (modular, one header + one source per
 * subsystem, shared config.h, no circular includes).
 *
 * This is a bare pico-sdk polling loop, matching the button/motor demo
 * (motor_test.c) that already runs on hardware. It is NOT the FreeRTOS
 * task table from the design review yet - each *_update() below stands
 * in for one RTOS task's body. Swapping this super-loop for real
 * FreeRTOS tasks later should not require changing any subsystem's
 * *_init()/*_update() signatures.
 */

#include "pico/stdlib.h"

#include "config.h"
#include "motor.h"
#include "line_tracking.h"
#include "barcode.h"
#include "imu_terrain.h"
#include "obstacle_scan.h"
#include "mission_state.h"
#include "telemetry.h"
#include "wifi_mqtt.h"

int main(void) {
    stdio_init_all(); // USB serial, for printf() debugging

    // Bring up every subsystem once. Order matters a little: mission_state
    // doesn't depend on the others at init time, but wifi_mqtt_init() can
    // block for a few seconds while it connects - if that's a problem,
    // move it later, or make it non-blocking and just poll for "connected"
    // in wifi_mqtt_update().
    mission_state_init();
    motor_init();
    line_tracking_init();
    barcode_init();
    imu_terrain_init();
    obstacle_scan_init();
    telemetry_init();
    wifi_mqtt_init();

    while (true) {
        // Sensing / input subsystems first, so mission_state sees fresh
        // events this iteration.
        line_tracking_update();
        barcode_on_edge_detected();   // event-driven: checks its own edge pin
        imu_terrain_update();
        obstacle_scan_update();

        // Mission state reacts to whatever the above just reported.
        mission_state_update();

        // Actuation follows the (possibly just-updated) mission state.
        motor_update();

        // Comms last, so telemetry reflects this iteration's state.
        wifi_mqtt_update();
        telemetry_update();

        sleep_ms(10); // matches the ~10ms loop target used elsewhere (NFR-13)
    }
}
