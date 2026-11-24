#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "../app.h"

#define MQTTM_PUBLISH		1
#define MQTTM_COMMAND		2
#define MQTTM_CONNECTED		3
#define MQTTM_DISCONNECTED	4
#define MQTTM_DEBUG		5	/* binary debug frame (debug topic) */
#define MQTTM_HEARTBEAT		6	/* binary heartbeat (heartbeat topic) */
#define MQTTM_BARCODE		7	/* barcode trace (barcode topic) */
#define MQTTM_IMU		8	/* IMU frame (imu topic) */
#define MQTTM_MAP		9	/* junction mapping received (map topic) */

#include "dash_logic.h"

typedef struct {
	UB	kind;
	UH	len;
	char	data[DASH_JSON_MAX];
} MQTT_MSG;

IMPORT ID	mbf_mqtt_out;
IMPORT ID	mbf_mqtt_in;
IMPORT ID	tid_dashboard;
IMPORT ID	tid_telemetry;

#define TELEMETRY_MS		500

IMPORT volatile UB s4_state;
IMPORT volatile UW s4_telem_sent;
IMPORT volatile UW s4_frames;
IMPORT volatile UW s4_console_lines;

IMPORT void dashboard_task(INT stacd, void *exinf);
IMPORT void telemetry_task(INT stacd, void *exinf);

#endif
