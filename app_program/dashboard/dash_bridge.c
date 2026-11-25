/**
 * @file dash_bridge.c
 *      Dashboard <-> MQTT bridge (IF-12), kernel side.  See dash_bridge.h.
 *      Called only from cyw43_utk_task, so the static buffers are not shared.
 */

#include "dashboard.h"
#include "dash_bridge.h"

EXPORT int dash_bridge_take(uint8_t *kind, uint8_t *data, uint16_t max, uint16_t *len)
{
    static MQTT_MSG m;
    INT i;

    while (MBF_RECV(mbf_mqtt_out, m, TMO_POL) > 0) {
        if (m.len > max) {
            continue;           /* cannot happen with our frames */
        }
        for (i = 0; i < m.len; i++) {
            data[i] = (uint8_t)m.data[i];
        }
        *kind = m.kind;
        *len = m.len;
        return 1;
    }
    return 0;
}

EXPORT void dash_bridge_put(uint8_t kind, const uint8_t *data, uint16_t len)
{
    static MQTT_MSG m;
    INT i;

    if (len > DASH_JSON_MAX) {
        return;
    }
    m.kind = kind;
    m.len = len;
    for (i = 0; i < len; i++) {
        m.data[i] = (char)data[i];
    }
    MBF_SEND(mbf_mqtt_in, m);
}

/*** end of file ***/
