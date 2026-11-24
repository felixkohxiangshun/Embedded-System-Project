/*
 *	mqtt_bridge.c
 *	The car's MQTT client: binary telemetry, debug and heartbeat out, binary
 *	commands in.  Built instead of the demo lib/.../mqtt_utk.c when
 *	DASH_MQTT_BRIDGE=1 (testing/dashboard/broker/build_dashboard.sh).
 *
 *	cyw43_utk_task calls mqtt_utk_start() on every poll (2 ms) once the
 *	link is up, so this file also does the periodic work there: connect with
 *	back-off, publish everything the dashboard task queued, and pass
 *	connection changes and received commands back.  It runs only on that
 *	task (the lwIP owner) and includes no kernel header; dash_bridge.c does
 *	the kernel side.
 *
 *	Topics (all under the development ACL csc11/dev/#):
 *	  csc11/dev/car/telemetry   11 bytes every 500 ms, QoS 0
 *	  csc11/dev/car/debug       12 bytes every 100 ms, QoS 0
 *	  csc11/dev/car/heartbeat   1 byte (system state) when telemetry stops
 *	  csc11/dev/car/cmd         commands from the PC, QoS 1
 */

#if DASH_MQTT_BRIDGE

#include "lwip/apps/mqtt.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "dash_bridge.h"
#include <tm/tmonitor.h>
#include <stdint.h>
#include <string.h>

#define TOPIC_TELEM	"csc11/dev/car/telemetry"
#define TOPIC_DEBUG	"csc11/dev/car/debug"
#define TOPIC_BARCODE	"csc11/dev/car/barcode"
#define TOPIC_IMU	"csc11/dev/car/imu"
#define TOPIC_HB	"csc11/dev/car/heartbeat"
#define TOPIC_CMD	"csc11/dev/car/cmd"
#define TOPIC_MAP	"csc11/dev/car/map"	/* junction mapping, retained */

#define BACKOFF_MIN_MS	1000u
#define BACKOFF_MAX_MS	16000u
#define CMD_MAX		32u	/* longest command accepted */
#define FRAME_MAX	96u	/* longest frame published (barcode trace, 81) */

static mqtt_client_t *client;
static uint8_t  connecting, online, retry_pending, rx_active, rx_kind;
static uint32_t retry_at, retry_ms = BACKOFF_MIN_MS;
static uint32_t tx_ok, tx_fail;
static uint8_t  rx_data[CMD_MAX];
static uint16_t rx_len;

static void went_offline(void)
{
	if (online) {
		dash_bridge_put(DASH_BRIDGE_DISCONNECTED, NULL, 0);
	}
	online = 0;
	connecting = 0;
	rx_active = 0;
	retry_pending = 1;
	retry_at = sys_now() + retry_ms;
	tm_printf((const unsigned char *)"[mqtt] offline, retry in %d ms\n", (int)retry_ms);
	if (retry_ms < BACKOFF_MAX_MS) {
		retry_ms *= 2;
	}
}

static void incoming_publish(void *arg, const char *topic, uint32_t length)
{
	(void)arg;
	rx_len = 0;
	rx_kind = (strcmp(topic, TOPIC_CMD) == 0) ? DASH_BRIDGE_COMMAND :
		  ((strcmp(topic, TOPIC_MAP) == 0) ? DASH_BRIDGE_MAP : 0);
	rx_active = (rx_kind != 0 && length > 0 && length <= CMD_MAX);
}

static void incoming_data(void *arg, const uint8_t *data, uint16_t length, uint8_t flags)
{
	(void)arg;
	if (!rx_active) {
		return;
	}
	if ((length != 0 && data == NULL) || length > CMD_MAX - rx_len) {
		rx_active = 0;
		return;
	}
	if (length != 0) {
		memcpy(rx_data + rx_len, data, length);
	}
	rx_len += length;
	if ((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_active = 0;
		dash_bridge_put(rx_kind, rx_data, rx_len);
	}
}

static void map_subscribed(void *arg, err_t result)
{
	(void)arg;
	if (result != ERR_OK) {
		/* commands still work; the car keeps its default letters */
		tm_printf((const unsigned char *)"[mqtt] map subscribe failed err=%d\n", (int)result);
	}
}

static void subscribe_done(void *arg, err_t result)
{
	(void)arg;
	if (result != ERR_OK) {
		tm_printf((const unsigned char *)"[mqtt] subscribe failed err=%d\n", (int)result);
		mqtt_disconnect(client);
		went_offline();
		return;
	}
	/* the junction mapping: the broker sends the retained one straight away */
	if (mqtt_subscribe(client, TOPIC_MAP, 1, map_subscribed, NULL) != ERR_OK) {
		tm_printf((const unsigned char *)"[mqtt] map subscribe request failed\n");
	}
	online = 1;
	retry_pending = 0;
	retry_ms = BACKOFF_MIN_MS;
	dash_bridge_put(DASH_BRIDGE_CONNECTED, NULL, 0);
	tm_printf((const unsigned char *)"[mqtt] online: publishing car/telemetry, car/debug\n");
}

static void connection_changed(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	err_t result;

	(void)c;
	(void)arg;
	connecting = 0;
	if (status != MQTT_CONNECT_ACCEPTED) {
		tm_printf((const unsigned char *)"[mqtt] connection status=%d\n", (int)status);
		went_offline();
		return;
	}
	tm_printf((const unsigned char *)"[mqtt] connected to " MQTT_BROKER_IP ":%d\n",
		  (int)MQTT_BROKER_PORT);
	mqtt_set_inpub_callback(client, incoming_publish, incoming_data, NULL);
	result = mqtt_subscribe(client, TOPIC_CMD, 1, subscribe_done, NULL);
	if (result != ERR_OK) {
		tm_printf((const unsigned char *)"[mqtt] subscribe request error=%d\n", (int)result);
		mqtt_disconnect(client);
		went_offline();
	}
}

static const char *topic_for(uint8_t kind)
{
	switch (kind) {
	case DASH_BRIDGE_TELEM:		return TOPIC_TELEM;
	case DASH_BRIDGE_DEBUG:		return TOPIC_DEBUG;
	case DASH_BRIDGE_HEARTBEAT:	return TOPIC_HB;
	case DASH_BRIDGE_BARCODE:	return TOPIC_BARCODE;
	case DASH_BRIDGE_IMU:		return TOPIC_IMU;
	default:			return NULL;
	}
}

/* Publish everything the dashboard task queued.  A frame lwIP cannot take
   right now is dropped: the next one is newer anyway. */
static void publish_queued(void)
{
	uint8_t  frame[FRAME_MAX];
	uint8_t  kind;
	uint16_t len;
	const char *topic;

	while (dash_bridge_take(&kind, frame, sizeof(frame), &len)) {
		topic = topic_for(kind);
		if (topic == NULL) {
			continue;
		}
		if (mqtt_publish(client, topic, frame, len, 0, 0, NULL, NULL) == ERR_OK) {
			if (++tx_ok % 100 == 1) {
				tm_printf((const unsigned char *)"[mqtt] published %u frames (%u dropped)\n",
					  (unsigned)tx_ok, (unsigned)tx_fail);
			}
		} else {
			++tx_fail;
		}
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t info;
	ip_addr_t address;
	uint32_t now = sys_now();
	uint8_t  frame[FRAME_MAX];
	uint8_t  kind;
	uint16_t len;

	if (client != NULL && mqtt_client_is_connected(client)) {
		if (online) {
			publish_queued();
		}
		return;
	}
	if (online) {
		went_offline();		/* the broker or the link dropped us */
	}
	while (dash_bridge_take(&kind, frame, sizeof(frame), &len)) {
		;			/* offline: drop stale frames */
	}
	if (connecting || (retry_pending && (int32_t)(now - retry_at) < 0)) {
		return;
	}
	retry_pending = 0;
	if (client == NULL) {
		client = mqtt_client_new();
		if (client == NULL) {
			went_offline();
			return;
		}
	}
	if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&address))) {
		tm_printf((const unsigned char *)"[mqtt] invalid MQTT_BROKER_IP\n");
		went_offline();
		return;
	}
	IP_SET_TYPE(&address, IPADDR_TYPE_V4);
	memset(&info, 0, sizeof(info));
	info.client_id = MQTT_CLIENT_ID;
	info.keep_alive = 15;
	connecting = 1;
	tm_printf((const unsigned char *)"[mqtt] connecting to " MQTT_BROKER_IP ":%d\n",
		  (int)MQTT_BROKER_PORT);
	if (mqtt_client_connect(client, &address, MQTT_BROKER_PORT,
				connection_changed, NULL, &info) != ERR_OK) {
		connecting = 0;
		went_offline();
	}
}

#endif
