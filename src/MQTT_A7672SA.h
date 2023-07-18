/**
 * @file       MQTT_A7672SA.h
 * @author     Giovanni de Rosso Unruh
 * @date       07/2023
 */

#ifndef MQTT_A7672SA_H_
#define MQTT_A7672SA_H_

#include <stdio.h>
#include "string.h"
#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "Arduino.h"

#define GSM_PROGMEM
typedef const char *ConstStr;

#define GSM_NL "\r\n"
static const char GSM_OK[] GSM_PROGMEM = GSM_NL "OK" GSM_NL;
static const char GSM_ERROR[] GSM_PROGMEM = GSM_NL "ERROR" GSM_NL;

struct mqtt_message
{
    char *topic;
    char *payload;
    size_t length;
};

enum registration_status
{
    NOT_REGISTERED = 0,
    REGISTERED_HOME = 1,
    NOT_REGISTERED_SEARCHING = 2,
    REGISTRATION_DENIED = 3,
    UNKNOWN = 4,
    REGISTERED_ROAMING = 5,
    REGISTERED_SMS_ONLY_HOME = 6,
    REGISTERED_SMS_ONLY_ROAMING = 7
};

class A7672SA
{
private:
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t en_pin;

    bool mqtt_connected;

    bool at_ok;
    bool at_ready;
    bool at_input;
    bool at_publish;
    char *at_response;
    uint32_t rx_buffer_size;

    void (*on_message_callback_)(mqtt_message &message);
    void (*mqtt_status_)(bool mqtt_connected);

    void rx_task();
    static void rx_taskImpl(void *pvParameters);
    void tx_task();
    static void tx_taskImpl(void *pvParameters);

    void simcomm_response_parser(const char *data);
    int send_cmd_to_simcomm(const char *logName, const char *data);

public:
    A7672SA();
    A7672SA(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin, int32_t baud_rate = 115200, uint32_t rx_buffer_size = 1024);
    ~A7672SA();

    void on_message_callback(void (*callback)(mqtt_message &message))
    {
        on_message_callback_ = callback;
    }

    void on_mqtt_status(void (*callback)(bool mqtt_connected))
    {
        mqtt_status_ = callback;
    }

    bool wait_input(uint32_t timeout = 1000);
    bool wait_publish(uint32_t timeout = 10000);
    bool wait_network(uint32_t timeout = 10000);
    bool wait_response(uint32_t timeout = 1000);
    bool wait_to_connect(uint32_t timeout = 10000);

    bool begin();
    bool is_ready();
    bool restart(uint32_t timeout = 1000);
    bool test_at(uint32_t timeout = 1000);
    bool sim_ready(uint32_t timeout = 1000);
    int signal_quality(uint32_t timeout = 1000);

    bool set_apn(const char *apn, uint32_t timeout = 1000);
    bool set_ntp_server(const char *ntp_server, int time_zone, uint32_t timeout = 1000);

    String get_ntp_time(uint32_t timeout = 1000);
    String get_provider_name(uint32_t timeout = 1000);
    String get_imei(uint32_t timeout = 1000);
    String get_local_ip(uint32_t timeout = 1000);

    bool set_ca_cert(const char *ca_cert, const char *ca_name, size_t cert_size, uint32_t timeout = 10000);
    bool mqtt_connect(const char *host, uint16_t port, const char *clientId, const char *username = NULL, const char *password = NULL, bool ssl = false, const char *ca_name = "ca.pem", uint16_t keepalive = 60, uint32_t timeout = 10000);
    bool mqtt_disconnect(uint32_t timeout = 1000);
    bool mqtt_publish(const char *topic, const char *data, uint16_t qos, uint32_t timeout = 1000);
    bool mqtt_subscribe(const char *topic, uint16_t qos, uint32_t timeout = 1000);
    bool mqtt_is_connected();
};

// enum mqtt_status //todo:
// {
//     MQTT_CONNECTED = 0,
//     MQTT_TIMEOUT = 17,
//     MQTT_CLIENT_USED = 19,
//     MQTT_CLIENT_NOT_AQUIRED = 20,
//     MQTT_CLIENT_NOT_RELEASED = 21,
//     MQTT_INVALID_USER_PASSWORD = 30,
//     MQTT_NOT_AUTHORIZED = 31,
//     MQTT_HANDSHAKE_FAIL = 32,
//     MQTT_NOT_SET_CERT = 33,
//     MQTT_DISCONNECTED_FAIL = 35
// };

#endif // MQTT_A7672SA_H_