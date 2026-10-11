/*
 *	dashboard.h
 *	ONLINE DASHBOARD subsystem -- owner Wong Jun Kai (2500258).
 *	Design review: section 6.5, FR-14..FR-18, NFR-12, NFR-13.
 *	The telemetry interface and frame are part of this subsystem.
 *
 *	Does:         shows the car's state online and takes remote commands.
 *	              Connection statechart: WIFI_CONNECTING, MQTT_CONNECTING,
 *	              ONLINE, HEARTBEAT, RECONNECTING.  Telemetry JSON is at most
 *	              512 bytes; a heartbeat is sent when no telemetry flows.
 *	Tasks:        dashboard_task (PRI_DASHBOARD, every 100 ms)
 *	              telemetry_task (PRI_TELEMETRY, every 500 ms): gathers app_state
 *	              (IF-11) into one MSG_TELEM frame (FR-16) and sends it on
 *	              mbf_telem.  Both tasks are low priority (16).
 *	Peripheral:   none directly.  The WiFi radio, lwIP and the MQTT client belong
 *	              to the kernel's cyw43_utk_task.  NO student task may call
 *	              cyw43_..., lwIP or MQTT functions (TEAM.md section 3).  This
 *	              task only
 *	                - reads the connection state with cyw43_utk_get_status()
 *	                - turns telemetry frames into JSON
 *	                - exchanges strings with the service task through the two
 *	                  message buffers below.
 *	Objects:      mbf_telem (IF-09), mbf_mqtt_out and mbf_mqtt_in (IF-12)
 *	Publishes:    MSG_MQTTCMD (E_STOP / RESUME) on mbf_mqttcmd (IF-07) to Mission
 *
 *	IF-12, THE MQTT BRIDGE.  Both buffers carry an MQTT_MSG.
 *	  mbf_mqtt_out  Dashboard -> service task: kind MQTTM_PUBLISH, the JSON
 *	                to publish (QoS 0, NFR-12).
 *	  mbf_mqtt_in   service task -> Dashboard: MQTTM_CONNECTED,
 *	                MQTTM_DISCONNECTED, or MQTTM_COMMAND with the payload of a
 *	                message received on the command topic (QoS 1, NFR-12).
 *	The service-task side (lib/liblwip/.../mqtt_utk.c) must read mbf_mqtt_out
 *	and write mbf_mqtt_in with TMO_POL only.  That is a change to shared
 *	library code: it needs the team's and the instructor's agreement and is
 *	NOT yet made (README section "Open items").  Until then the state chart
 *	stays in MQTT_CONNECTING and the task prints the telemetry JSON on the
 *	console so the frames can still be checked.
 */

#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "../app.h"

/* IF-12 message kinds */
#define MQTTM_PUBLISH		1
#define MQTTM_COMMAND		2
#define MQTTM_CONNECTED		3
#define MQTTM_DISCONNECTED	4

#include "dash_logic.h"

typedef struct {
	UB	kind;			/* MQTTM_xxx */
	UH	len;			/* bytes used in data[] */
	char	data[DASH_JSON_MAX];
} MQTT_MSG;

IMPORT ID	mbf_mqtt_out;		/* Dashboard -> MQTT service task */
IMPORT ID	mbf_mqtt_in;		/* MQTT service task -> Dashboard */
IMPORT ID	tid_dashboard;
IMPORT ID	tid_telemetry;

#define TELEMETRY_MS		500	/* FR-16 */

IMPORT volatile UB s4_state;		/* DASH_xxx, for tests */
IMPORT volatile UW s4_telem_sent;	/* frames the Telemetry task sent */
IMPORT volatile UW s4_frames;		/* telemetry frames formatted */
IMPORT volatile UW s4_console_lines;

IMPORT void dashboard_task(INT stacd, void *exinf);
IMPORT void telemetry_task(INT stacd, void *exinf);

#endif /* DASHBOARD_H */
