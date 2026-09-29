/*
 * imu_terrain.h - IMU / Terrain Monitoring (owner: Sook Yee)
 * Covers FR-04, FR-05, FR-06, NFR-05, NFR-06.
 */
#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

void imu_terrain_init(void);

// Target: sample the accelerometer/magnetometer every 10ms (NFR-05).
// TODO(Sook Yee): detect humps (peak height within +/-5mm, NFR-06) and
// SHOCK events (>3g for <3 samples), report both to mission_state.c
// and telemetry.c.
void imu_terrain_update(void);

#endif // IMU_TERRAIN_H
