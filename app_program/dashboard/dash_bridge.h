/*
 *	dash_bridge.h
 *	Dashboard <-> MQTT bridge (IF-12), kernel side.
 *
 *	The MQTT client runs inside cyw43_utk_task, the only task allowed to use
 *	lwIP (TEAM.md 3).  That code cannot include the kernel headers, so these
 *	two functions, written against plain C types, carry the messages across:
 *	mqtt_bridge.c calls them; dash_bridge.c moves the data through the
 *	message buffers mbf_mqtt_out and mbf_mqtt_in.  Neither ever blocks.
 */

#ifndef DASH_BRIDGE_H
#define DASH_BRIDGE_H

#include <stdint.h>

/* Message kinds (same values as MQTTM_xxx in dashboard.h) */
#define DASH_BRIDGE_TELEM	1	/* publish on the telemetry topic */
#define DASH_BRIDGE_COMMAND	2	/* received on the command topic */
#define DASH_BRIDGE_CONNECTED	3
#define DASH_BRIDGE_DISCONNECTED 4
#define DASH_BRIDGE_DEBUG	5	/* publish on the debug topic */
#define DASH_BRIDGE_HEARTBEAT	6	/* publish on the heartbeat topic */

/* Next message to publish.  Returns 1 and fills kind/data/len, or 0 if
   there is none.  data must hold max bytes; longer messages are dropped. */
int  dash_bridge_take(uint8_t *kind, uint8_t *data, uint16_t max, uint16_t *len);

/* Hand an event or a received command to the dashboard task.  Dropped if
   its buffer is full. */
void dash_bridge_put(uint8_t kind, const uint8_t *data, uint16_t len);

#endif
