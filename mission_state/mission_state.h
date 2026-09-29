/*
 * mission_state.h - Mission State (owner: Felix)
 * Coordinates events from line_tracking, barcode, imu_terrain, and
 * obstacle_scan into navigation decisions. Implements the System-Level
 * Statechart from the design review (INIT / LINE_FOLLOW / OBSTACLE_AVOID
 * / BARCODE_TURN / E_STOP), including FR-17 (E_STOP / RESUME).
 */
#ifndef MISSION_STATE_H
#define MISSION_STATE_H

typedef enum {
    MISSION_INIT,
    MISSION_LINE_FOLLOW,
    MISSION_OBSTACLE_AVOID,
    MISSION_BARCODE_TURN,
    MISSION_E_STOP,
} mission_state_t;

void mission_state_init(void);
void mission_state_update(void);
mission_state_t mission_state_get(void);

// Called by the other subsystems to report an event into the state
// machine - fill these in together so everyone agrees on the contract.
// TODO(Felix + team): agree on the exact signatures, e.g.:
// void mission_report_obstacle(float distance_cm, float width_cm);
// void mission_report_barcode_command(char command);
// void mission_report_estop(bool stop);

#endif // MISSION_STATE_H
