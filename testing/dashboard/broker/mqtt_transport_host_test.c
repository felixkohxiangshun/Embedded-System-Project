#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#define DASH_MQTT_TRANSPORT_TEST 1
#include "../../../app_program/dashboard/mqtt_transport_test.c"

static uint32_t clock_ms;
static int connected, connect_calls, subscribe_calls, publish_calls, fake_client;
static err_t connect_result = ERR_OK, subscribe_result = ERR_OK, publish_result = ERR_OK;
static mqtt_connection_cb_t captured_connection;
static mqtt_request_cb_t captured_subscribe;

uint32_t sys_now(void) { return clock_ms; }
int tm_printf(const unsigned char *format, ...) { (void)format; return 0; }
mqtt_client_t *mqtt_client_new(void) { return (mqtt_client_t *)&fake_client; }
uint8_t mqtt_client_is_connected(mqtt_client_t *c) { (void)c; return (uint8_t)connected; }
void mqtt_disconnect(mqtt_client_t *c) { (void)c; connected = 0; }
int ip4addr_aton(const char *text, ip4_addr_t *out)
{
    assert(strcmp(text, "127.0.0.1") == 0);
    out->addr = 1;
    return 1;
}
err_t mqtt_client_connect(mqtt_client_t *c, const ip_addr_t *address, uint16_t port,
                         mqtt_connection_cb_t cb, void *arg,
                         const struct mqtt_connect_client_info_t *info)
{
    (void)c; (void)address; (void)arg;
    assert(port == 1883 && strcmp(info->client_id, MQTT_CLIENT_ID) == 0);
    captured_connection = cb;
    ++connect_calls;
    return connect_result;
}
void mqtt_set_inpub_callback(mqtt_client_t *c, mqtt_incoming_publish_cb_t pub,
                            mqtt_incoming_data_cb_t data, void *arg)
{
    (void)c; (void)arg;
    assert(pub == incoming_publish && data == incoming_data);
}
err_t mqtt_sub_unsub(mqtt_client_t *c, const char *topic, uint8_t qos,
                     mqtt_request_cb_t cb, void *arg, uint8_t sub)
{
    (void)c; (void)arg;
    assert(strcmp(topic, TEST_RX_TOPIC) == 0 && qos == 1 && sub == 1);
    captured_subscribe = cb;
    ++subscribe_calls;
    return subscribe_result;
}
err_t mqtt_publish(mqtt_client_t *c, const char *topic, const void *data,
                   uint16_t length, uint8_t qos, uint8_t retain,
                   mqtt_request_cb_t cb, void *arg)
{
    (void)c; (void)cb; (void)arg;
    assert(strcmp(topic, TEST_TX_TOPIC) == 0 && qos == 0 && retain == 0);
    assert(length == sizeof(fixture) && memcmp(data, fixture, length) == 0);
    ++publish_calls;
    return publish_result;
}

static void accept_connection(void)
{
    connected = 1;
    captured_connection(client, NULL, MQTT_CONNECT_ACCEPTED);
}

int main(void)
{
    uint8_t wrong[5] = {0};
    int i;
    mqtt_utk_start();
    mqtt_utk_start();
    assert(connect_calls == 1 && connecting);
    accept_connection();
    mqtt_utk_start();
    assert(subscribe_calls == 1 && publish_calls == 0);
    captured_subscribe(NULL, ERR_OK);
    mqtt_utk_start();
    assert(subscribed && tx_accepted == 1 && publish_calls == 1);
    clock_ms = 499; mqtt_utk_start();
    assert(publish_calls == 1);
    clock_ms = 500; mqtt_utk_start();
    assert(publish_calls == 2);
    clock_ms = 10000; mqtt_utk_start(); mqtt_utk_start();
    assert(publish_calls == 3);
    publish_result = ERR_MEM;
    clock_ms += 500; mqtt_utk_start();
    assert(tx_failed == 1 && tx_accepted == 3);
    publish_result = ERR_OK;

    incoming_publish(NULL, TEST_RX_TOPIC, 5);
    incoming_data(NULL, fixture, 2, 0);
    assert(rx_pass == 0 && rx_len == 2);
    incoming_data(NULL, fixture + 2, 3, MQTT_DATA_FLAG_LAST);
    assert(rx_pass == 1 && !rx_active);
    incoming_data(NULL, fixture, 5, MQTT_DATA_FLAG_LAST);
    assert(rx_pass == 1);
    incoming_publish(NULL, TEST_RX_TOPIC, 0);
    incoming_publish(NULL, TEST_RX_TOPIC, UINT32_MAX);
    incoming_publish(NULL, "other", 5);
    incoming_publish(NULL, TEST_RX_TOPIC, 5);
    incoming_data(NULL, fixture, 2, MQTT_DATA_FLAG_LAST);
    incoming_publish(NULL, TEST_RX_TOPIC, 5);
    incoming_data(NULL, fixture, 6, MQTT_DATA_FLAG_LAST);
    incoming_publish(NULL, TEST_RX_TOPIC, 5);
    incoming_data(NULL, wrong, 5, MQTT_DATA_FLAG_LAST);
    assert(rx_rejected == 6 && rx_pass == 1);

    connected = 0;
    captured_connection(client, NULL, MQTT_CONNECT_DISCONNECTED);
    mqtt_utk_start();
    assert(connect_calls == 1);
    clock_ms += 999; mqtt_utk_start();
    assert(connect_calls == 1);
    clock_ms += 1; mqtt_utk_start();
    assert(connect_calls == 2);
    accept_connection();
    assert(subscribe_calls == 2 && !subscribed);
    captured_subscribe(NULL, ERR_ABRT);
    assert(!connected && retry_pending && !subscribed);

    connect_result = ERR_CONN;
    for (i = 0; i < 10; ++i) {
        int before = connect_calls;
        clock_ms = retry_at - 1; mqtt_utk_start();
        assert(connect_calls == before);
        clock_ms = retry_at; mqtt_utk_start();
        assert(connect_calls == before + 1 && retry_pending);
        assert(retry_ms <= TEST_BACKOFF_MAX_MS);
    }

    retry_ms = 1000;
    clock_ms = UINT32_MAX - 499;
    schedule_retry();
    clock_ms = 499; i = connect_calls; mqtt_utk_start();
    assert(connect_calls == i);
    clock_ms = 500; mqtt_utk_start();
    assert(connect_calls == i + 1);
    puts("MQTT transport host PASS: subscription gating, 500ms scheduling, binary fragments, rejection, backoff, resubscription, clock wrap");
    return 0;
}
