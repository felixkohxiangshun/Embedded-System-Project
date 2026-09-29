#include "imu_terrain.h"
#include "config.h"
#include "hardware/i2c.h"

// TODO(Sook Yee): placeholder. LSM303DLHC is on PIN_IMU_SDA/PIN_IMU_SCL
// (I2C). See Subsystem Statechart 6.3 (IMU / Terrain) in the design
// review for the IDLE -> MONITORING -> HUMP_DETECTED / SHOCK_DETECTED /
// SENSOR_FAULT states this is expected to implement.

void imu_terrain_init(void) {
    // TODO(Sook Yee): i2c_init() on i2c0, configure the LSM303DLHC
}

void imu_terrain_update(void) {
    // TODO(Sook Yee): sample + classify hump/shock events
}
