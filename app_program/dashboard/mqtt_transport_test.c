/* Opt-in transport test, NOT the final telemetry/command protocol.
 * The build substitutes this for mqtt_utk.c. mqtt_utk_start() and all callbacks
 * run on the existing radio/lwIP service task; no student task calls lwIP.
 * No Mission messages, motor GPIOs or application commands are generated.
 */
#if DASH_MQTT_TRANSPORT_TEST

#include "lwip/apps/mqtt.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include <tm/tmonitor.h>
#include <stdint.h>
#include <string.h>

#define TEST_TX_TOPIC "csc11/dev/pico/tx"
#define TEST_RX_TOPIC "csc11/dev/pico/rx"
#define TEST_PUBLISH_MS 500u
#define TEST_BACKOFF_MAX_MS 16000u

/* Artificial byte fixture includes NUL and non-ASCII: not sensor fields. */
static const uint8_t fixture[] = {0x8d, 0x23, 0x01, 0x00, 0xff};
static mqtt_client_t *client;
static uint8_t connecting, subscribed, retry_pending, rx_active;
static uint32_t retry_at, retry_ms = 1000, next_publish;
static uint32_t tx_accepted, tx_failed, rx_pass, rx_rejected;
static uint8_t rx_data[sizeof(fixture)];
static uint16_t rx_len;

static void schedule_retry(void)
{
    connecting = 0;
    subscribed = 0;
    rx_active = 0;
    retry_pending = 1;
    retry_at = sys_now() + retry_ms;
    tm_printf((const unsigned char *)"[mqtt-test] retry in %d ms\n", (int)retry_ms);
    if (retry_ms < TEST_BACKOFF_MAX_MS) retry_ms *= 2;
}

static void reject_rx(void)
{
    rx_active = 0;
    ++rx_rejected;
    tm_printf((const unsigned char *)"[mqtt-test] RX REJECT count=%u\n", (unsigned)rx_rejected);
}

static void incoming_publish(void *arg, const char *topic, uint32_t length)
{
    (void)arg;
    rx_len = 0;
    rx_active = 0;
    if (strcmp(topic, TEST_RX_TOPIC) != 0 || length != sizeof(fixture)) {
        reject_rx();
        return;
    }
    rx_active = 1;
}

static void incoming_data(void *arg, const uint8_t *data, uint16_t length, uint8_t flags)
{
    (void)arg;
    if (!rx_active) return;
    if ((length != 0 && data == NULL) || length > sizeof(rx_data) - rx_len) {
        reject_rx();
        return;
    }
    if (length != 0) memcpy(rx_data + rx_len, data, length);
    rx_len += length;
    if ((flags & MQTT_DATA_FLAG_LAST) != 0) {
        if (rx_len != sizeof(fixture) || memcmp(rx_data, fixture, sizeof(fixture)) != 0) {
            reject_rx();
            return;
        }
        rx_active = 0;
        ++rx_pass;
        tm_printf((const unsigned char *)"[mqtt-test] RX PASS count=%u bytes=8d230100ff (log only)\n",
                  (unsigned)rx_pass);
    }
}

static void subscribe_done(void *arg, err_t result)
{
    (void)arg;
    if (result != ERR_OK) {
        tm_printf((const unsigned char *)"[mqtt-test] subscribe failed err=%d\n", (int)result);
        mqtt_disconnect(client);
        schedule_retry();
        return;
    }
    subscribed = 1;
    retry_pending = 0;
    retry_ms = 1000;
    next_publish = sys_now();
    tm_printf((const unsigned char *)"[mqtt-test] ONLINE; RX subscribed QoS1; TX QoS0 every 500ms\n");
}

static void publish_done(void *arg, err_t result)
{
    (void)arg;
    if (result != ERR_OK) {
        ++tx_failed;
        tm_printf((const unsigned char *)"[mqtt-test] publish completion error=%d\n", (int)result);
    }
}

static void connection_changed(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
    err_t result;
    (void)c;
    (void)arg;
    connecting = 0;
    subscribed = 0;
    rx_active = 0;
    if (status != MQTT_CONNECT_ACCEPTED) {
        tm_printf((const unsigned char *)"[mqtt-test] connection status=%d\n", (int)status);
        schedule_retry();
        return;
    }
    tm_printf((const unsigned char *)"[mqtt-test] connected to " MQTT_BROKER_IP ":%d\n",
              (int)MQTT_BROKER_PORT);
    mqtt_set_inpub_callback(client, incoming_publish, incoming_data, NULL);
    result = mqtt_subscribe(client, TEST_RX_TOPIC, 1, subscribe_done, NULL);
    if (result != ERR_OK) {
        tm_printf((const unsigned char *)"[mqtt-test] subscribe request error=%d\n", (int)result);
        mqtt_disconnect(client);
        schedule_retry();
    }
}

/* Called repeatedly only while the WiFi service sees CYW43_LINK_UP.
 * No extra tasks, timer chains or catch-up publication loops are created.
 */
void mqtt_utk_start(void)
{
    struct mqtt_connect_client_info_t info = {0};
    ip_addr_t address;
    uint32_t now = sys_now();
    err_t result;

    if (client != NULL && mqtt_client_is_connected(client)) {
        if (!subscribed || (int32_t)(now - next_publish) < 0) return;
        next_publish = now + TEST_PUBLISH_MS;
        result = mqtt_publish(client, TEST_TX_TOPIC, fixture, sizeof(fixture),
                              0, 0, publish_done, NULL);
        if (result == ERR_OK) {
            ++tx_accepted;
            if (tx_accepted == 1 || tx_accepted % 10 == 0) {
                tm_printf((const unsigned char *)"[mqtt-test] TX accepted=%u failed=%u bytes=8d230100ff\n",
                          (unsigned)tx_accepted, (unsigned)tx_failed);
            }
        } else {
            ++tx_failed;
            tm_printf((const unsigned char *)"[mqtt-test] publish request error=%d\n", (int)result);
        }
        return;
    }
    if (connecting || (retry_pending && (int32_t)(now - retry_at) < 0)) return;
    retry_pending = 0;
    if (client == NULL) client = mqtt_client_new();
    if (client == NULL) {
        schedule_retry();
        return;
    }
    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&address))) {
        tm_printf((const unsigned char *)"[mqtt-test] invalid broker IP\n");
        schedule_retry();
        return;
    }
    IP_SET_TYPE(&address, IPADDR_TYPE_V4);
    info.client_id = MQTT_CLIENT_ID;
    info.keep_alive = 15;
    connecting = 1;
    tm_printf((const unsigned char *)"[mqtt-test] connecting to " MQTT_BROKER_IP ":%d\n",
              (int)MQTT_BROKER_PORT);
    result = mqtt_client_connect(client, &address, MQTT_BROKER_PORT,
                                 connection_changed, NULL, &info);
    if (result != ERR_OK) {
        tm_printf((const unsigned char *)"[mqtt-test] connect request error=%d\n", (int)result);
        schedule_retry();
    }
}

#endif /* DASH_MQTT_TRANSPORT_TEST */
