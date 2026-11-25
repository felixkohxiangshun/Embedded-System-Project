/**
 * @file drv_imu.c
 *      IMU/terrain -- LSM303DLHC accelerometer over the kernel I2C driver.
 *
 *      Settings: 100 Hz, all axes on (CTRL_REG1_A = 0x57); block data update,
 *      +/-8 g, high-resolution (CTRL_REG4_A = 0xA8): 4 mg per LSB on the 12-bit
 *      value left-justified in the 16-bit output.  (+/-8 g was chosen for the
 *      3 g shock threshold, since dropped; it is kept.)  Block data update
 *      stops the 6-byte read mixing two samples.
 *
 *      All three axes are read in ONE transfer (sub-address auto-increment), so
 *      one 10 ms tick costs one I2C transaction (~0.2 ms at 400 kHz).
 *
 *      Only imu_task calls these functions (one owning task per peripheral).
 *      The kernel I2C driver serialises transfers inside the device driver.
 */

#include "imu_terrain.h"
#include "../../device/include/dev_i2c.h"

#define ACC_ADDR                0x19    /* 7-bit */
#define REG_CTRL1_A             0x20
#define REG_CTRL4_A             0x23
#define REG_OUT_X_L_A           0x28
#define SUB_AUTO_INC            0x80    /* MSB of the sub-address: auto-increment */
#define CTRL1_100HZ_XYZ         0x57
#define CTRL4_BDU_8G_HR         0xA8
#define MG_PER_LSB              4       /* +/-8 g, high-resolution mode */

LOCAL ID dd = -1;

EXPORT ER imu_hw_open(void)
{
    ER er;
    UB v = 0;

    if (dd < E_OK) {
        dd = tk_opn_dev((UB*)"iica", TD_UPDATE);        /* I2C unit 0 */
        if (dd < E_OK) {
            return dd;
        }
    }
    er = i2c_write_reg(dd, ACC_ADDR, REG_CTRL1_A, CTRL1_100HZ_XYZ);
    if (er < E_OK) {
        return er;
    }
    er = i2c_write_reg(dd, ACC_ADDR, REG_CTRL4_A, CTRL4_BDU_8G_HR);
    if (er < E_OK) {
        return er;
    }
    /* The DLHC accelerometer has no WHO_AM_I: read a setting back instead */
    er = i2c_read_reg(dd, ACC_ADDR, REG_CTRL1_A, &v);
    if (er < E_OK) {
        return er;
    }
    return (v == CTRL1_100HZ_XYZ) ? E_OK : E_IO;
}

EXPORT ER imu_read_accel(INT mg[3])
{
    T_I2C_EXEC exec;
    UB  sub = (UB)(REG_OUT_X_L_A | SUB_AUTO_INC);
    UB  b[6];
    SZ  asz;
    ER  er;
    INT k;

    if (dd < E_OK) {
        return E_NOEXS;
    }
    exec.sadr = ACC_ADDR;
    exec.snd_size = 1;
    exec.snd_data = &sub;
    exec.rcv_size = 6;
    exec.rcv_data = b;
    er = tk_swri_dev(dd, TDN_I2C_EXEC, &exec, (SZ)sizeof(exec), &asz);
    if (er < E_OK) {
        return er;
    }
    for (k = 0; k < 3; k++) {
        /* X, Y, Z: low byte first, 12 bits left-justified, signed */
        H raw = (H)(((UH)b[(2 * k) + 1] << 8) | b[2 * k]);

        mg[k] = ((INT)raw / 16) * MG_PER_LSB;
    }
    return E_OK;
}

/*** end of file ***/
