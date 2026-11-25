/**
 * @file motor_control.c
 *      Motor control -- Motor: creates its kernel objects and starts its task.
 *
 */

#include "motor_control.h"

EXPORT ID       mbf_nav;        /* IF-08  Mission -> Motor commands      */
EXPORT ID       flg_evt;        /* IF-10  START / STOP / RESUME / BTN    */
EXPORT ID       tid_motor;
EXPORT ID       cyc_motor;

EXPORT ER motor_control_init(void)
{
    T_CCYC      ccyc = {0};

    /* Room for 8 pending commands; steering updates arrive every 10 ms. */
    mbf_nav = app_make_mbf((INT)sizeof(MSG_NAV), 8);
    if (mbf_nav < E_OK) {
        return mbf_nav;
    }
    flg_evt = app_make_flg();
    if (flg_evt < E_OK) {
        return flg_evt;
    }

    /* 10 ms cyclic handler wakes the Motor task (NFR-14).  Created
       stopped; started in motor_control_start() once the task exists. */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)motor_cyclic_handler;
    ccyc.cyctim = MOTOR_TICK_MS;
    ccyc.cycphs = MOTOR_TICK_MS;
    cyc_motor = tk_cre_cyc(&ccyc);
    return (cyc_motor < E_OK) ? cyc_motor : E_OK;
}

EXPORT ER motor_control_start(void)
{
    ER er;

    motor_hw_init();

    tid_motor = app_start_task(motor_task, PRI_MOTOR, 0, "motor_task");
    if (tid_motor < E_OK) {
        return tid_motor;
    }
    er = gpio_isr_start();
    if (er < E_OK) {
        return er;
    }
    return tk_sta_cyc(cyc_motor);
}

/*** end of file ***/
