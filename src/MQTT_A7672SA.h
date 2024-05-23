/**
 * @file       MQTT_A7672SA.h
 * @author     Giovanni de Rosso Unruh
 * @date       07/2023
 */

#ifndef MQTT_A7672SA_H_
#define MQTT_A7672SA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "Arduino.h"

#include <IPAddress.h>

#define GSM_PROGMEM
typedef const char *ConstStr;

#define UART_QUEUE_SIZE 10

#define GSM_NL "\r\n"
#define GSM_NM "+"
static const char GSM_OK[] GSM_PROGMEM = GSM_NL "OK" GSM_NL;
static const char GSM_ERROR[] GSM_PROGMEM = GSM_NL "ERROR" GSM_NL;

struct mqtt_message
{
    char *topic;
    uint8_t *payload;
    size_t length;
};

struct commandMessage
{
    char logName[48];
    char data[256];
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

enum mqtt_status // todo:
{
    A7672SA_MQTT_CONNECTED = 0,
    A7672SA_MQTT_TIMEOUT = 1,
    A7672SA_MQTT_CLIENT_USED = 2,
    A7672SA_MQTT_DISCONNECTED = 3
};

class A7672SA
{
private:
    SemaphoreHandle_t publish_semaphore;
    QueueHandle_t uartQueue;

    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t en_pin;

    bool mqtt_connected;

    bool at_ok;
    bool at_error;
    bool at_ready;
    bool at_input;
    bool at_publish;
    char *at_response;
    uint32_t rx_buffer_size;

    void (*on_message_callback_)(mqtt_message &message);
    void (*on_mqtt_status_)(mqtt_status &status);

    void rx_task();
    static void rx_taskImpl(void *pvParameters);
    void tx_task();
    static void tx_taskImpl(void *pvParameters);

    void sendCommand(const char *log, const char *data);
    void sendCommand(commandMessage message);
    bool receiveCommand(commandMessage *message);

    void simcomm_response_parser(const char *data);
    char **simcom_split_messages(const char *data, int *n_messages);

public:
    A7672SA();
    A7672SA(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin, int32_t baud_rate = 115200, uint32_t rx_buffer_size = 1024);
    ~A7672SA();

    void on_message_callback(void (*callback)(mqtt_message &message))
    {
        on_message_callback_ = callback;
    }

    void on_mqtt_status(void (*callback)(mqtt_status &status))
    {
        on_mqtt_status_ = callback;
    }

    int send_cmd_to_simcomm(const char *logName, const char *data);
    int send_cmd_to_simcomm(const char *logName, byte *data, int len);

    bool wait_input(uint32_t timeout = 2000);
    bool wait_publish(uint32_t timeout = 2000);
    bool wait_network(uint32_t timeout = 10000);
    bool wait_response(uint32_t timeout = 2000);
    bool wait_to_connect(uint32_t timeout = 10000);

    bool begin();
    bool is_ready();
    bool restart(uint32_t timeout = 1000);
    bool test_at(uint32_t timeout = 1000);
    bool sim_ready(uint32_t timeout = 1000);
    int signal_quality(uint32_t timeout = 1000);

    bool set_apn(const char *apn, uint32_t timeout = 1000);
    bool set_ntp_server(const char *ntp_server, int time_zone, uint32_t timeout = 1000);

    time_t get_ntp_time(uint32_t timeout = 1000);
    String get_provider_name(uint32_t timeout = 1000);
    String get_imei(uint32_t timeout = 1000);
    String get_iccid(uint32_t timeout = 1000);
    IPAddress get_local_ip(uint32_t timeout = 1000);

    bool set_ca_cert(const char *ca_cert, const char *ca_name, size_t cert_size, uint32_t timeout = 10000);
    bool mqtt_connect(const char *host, uint16_t port, const char *clientId, bool clean_session = true, const char *username = "", const char *password = "", bool ssl = false, const char *ca_name = "ca.pem", uint16_t keepalive = 60, uint32_t timeout = 10000);
    bool mqtt_disconnect(uint32_t timeout = 1000);
    bool mqtt_release_client(uint32_t timeout = 1000);
    bool mqtt_publish(const char *topic, byte *data, size_t len, uint16_t qos = 0, uint32_t timeout = 3000);
    bool mqtt_subscribe_topics(const char *topic[10], int n_topics = 10, uint16_t qos = 0, uint32_t timeout = 1000);
    bool mqtt_subscribe(const char *topic, uint16_t qos, uint32_t timeout = 1000);
    bool mqtt_is_connected();

    bool http_term(uint32_t timeout = 1000);
    bool http_request(const char *url, uint8_t method, bool ssl = false, const char *data = "", size_t size = 0, uint32_t timeout = 1000);
    // bool http_read(uint32_t timeout = 1000);
    // bool http_read_file(const char *filename, uint32_t timeout = 1000);
    // bool http_post_file(const char *filename, uint32_t timeout = 1000);
};

#endif // MQTT_A7672SA_H_