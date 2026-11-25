/**
 * @file dashboard.c
 * @brief Creates the dashboard subsystem's message buffers.
 */

#include "dashboard.h"

EXPORT ID       mbf_telem;
EXPORT ID       mbf_mqtt_out;
EXPORT ID       mbf_mqtt_in;
EXPORT ID       tid_dashboard;
EXPORT ID       tid_telemetry;

EXPORT ER dashboard_init(void)
{

    mbf_telem = app_make_mbf((INT)sizeof(MSG_TELEM), 4);
    if (mbf_telem < E_OK) {
        return mbf_telem;
    }
    /* up to four frames in one dashboard tick: telemetry, debug, IMU and
       a barcode trace */
    mbf_mqtt_out = app_make_mbf((INT)sizeof(MQTT_MSG), 4);
    if (mbf_mqtt_out < E_OK) {
        return mbf_mqtt_out;
    }
    mbf_mqtt_in = app_make_mbf((INT)sizeof(MQTT_MSG), 2);
    return (mbf_mqtt_in < E_OK) ? mbf_mqtt_in : E_OK;
}

EXPORT ER dashboard_start(void)
{
    tid_dashboard = app_start_task(dashboard_task, PRI_DASHBOARD, 0, "dashboard_task");
    if (tid_dashboard < E_OK) {
        return tid_dashboard;
    }
    tid_telemetry = app_start_task(telemetry_task, PRI_TELEMETRY, 0, "telemetry_task");
    return (tid_telemetry < E_OK) ? tid_telemetry : E_OK;
}

/*** end of file ***/
