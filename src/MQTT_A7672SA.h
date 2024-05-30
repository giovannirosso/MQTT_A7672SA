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
    char data[1024]; // todo 1024
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

enum HTTP_METHOD
{
    GET = 0,
    POST = 1,
    HEAD = 2,
    DELETE = 3,
    PUT = 4
};

class A7672SA
{
private:
    SemaphoreHandle_t publish_semaphore;
    QueueHandle_t uartQueue;
    TaskHandle_t rxTaskHandle;
    TaskHandle_t txTaskHandle;

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

    bool http_response;
    size_t http_response_size;

    void (*on_message_callback_)(mqtt_message &message);
    void (*on_mqtt_status_)(mqtt_status &status);

    void rx_task();
    static void rx_taskImpl(void *pvParameters);
    void tx_task();
    static void tx_taskImpl(void *pvParameters);

    int send_cmd_to_simcomm(const char *logName, const char *data);
    int send_cmd_to_simcomm(const char *logName, byte *data, int len);

    bool receiveCommand(commandMessage *message);

    void simcomm_response_parser(const char *data);
    char **simcom_split_messages(const char *data, int *n_messages);

public:
    A7672SA();
    A7672SA(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin, int32_t baud_rate = 115200, uint32_t rx_buffer_size = 1024 * 10); // todo 1024
    ~A7672SA();

    void on_message_callback(void (*callback)(mqtt_message &message))
    {
        on_message_callback_ = callback;
    }

    void on_mqtt_status(void (*callback)(mqtt_status &status))
    {
        on_mqtt_status_ = callback;
    }

    void sendCommand(const char *log, const char *data);

    bool wait_input(uint32_t timeout = 2000);
    bool wait_publish(uint32_t timeout = 2000);
    bool wait_network(uint32_t timeout = 10000);
    bool wait_response(uint32_t timeout = 2000);
    bool wait_to_connect(uint32_t timeout = 10000);
    bool wait_http_response(uint32_t timeout = 10000);

    bool begin();
    bool stop();
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

    /*
    <url> URL of network resource.String,start with "http://" or"https://" a)http://’server’ :’tcpPort’ /’path’. b)https://’server’ :’tcpPort’ /’path’. "server" DNS domain name or IP address "path" path to a file or directory of a server "tcpPort" http default value is 80,https default value is 443.(canbeomitted)
    <method> HTTP request method, enum HTTP_METHOD, range is 0-4. 0: GET, 1: POST, 2: HEAD 3: DELETE, 4: PUT.
    <ssl> Whether to use SSL, Boolean type. false: no SSL, true: use SSL. Default is false.
    <ca_name> The name of the CA certificate uploaded to simcom, String type, default is "ca.pem".
    <user_data> The customized HTTP header information. String type, max lengthis 256.
    <user_data_size> The size of user_data, Numeric type, range is 0-256, default is 0.
    <conn_timeout> Timeout for accessing server, Numeric type, range is 20-120s, default is 120s.
    <recv_timeout> Timeout for receiving data from server, Numeric type range is 2s-120s, default is 20s.
    <content_type> This is for HTTP "Content-Type" tag, String type, max length is 256, and default is "text/plain".
    <accept-type> This is for HTTP "Accept-type" tag, String type, max length is 256, and default is "".
    <sslcfg_id> This is setting SSL context id, Numeric type, range is 0-9. Default is0.Please refer to Chapter 19 of this document.
    <data_post> Data to be sent to server, String type. Default is "".
    <size> The size of data to be sent to server, Numeric type. Default is 0.
    <readmode> For HTTPREAD, Numeric type, it can be set to 0 or 1. If set to1, youcan read the response content data from the same position repeatly. The limit is that the size of HTTP server response content shouldbeshorter than 1M.Default is 0.
    */
    uint32_t http_request(const char *url, HTTP_METHOD method, bool ssl = false, const char *ca_name = "ca.pem",
                          const char *user_data = "", size_t user_data_size = 0, uint32_t con_timeout = 120, uint32_t recv_timeout = 120,
                          const char *content = "text/plain", const char *accept = "*/*", uint8_t read_mode = 0, const char *data_post = "", size_t size = 0, uint32_t timeout = 10000);

    bool http_request_file(const char *url, HTTP_METHOD method, const char *filename, bool ssl = false, const char *ca_name = "ca.pem",
                           const char *user_data = "", size_t user_data_size = 0, uint32_t con_timeout = 120, uint32_t recv_timeout = 120,
                           const char *content = "text/plain", const char *accept = "*/*", uint8_t read_mode = 0, const char *data_post = "", size_t size = 0, uint32_t timeout = 10000);
    bool http_read_response(size_t read_size, uint32_t timeout = 1000);
    bool http_read_file(const char *filename, uint32_t timeout = 1000);
    bool http_term(uint32_t timeout = 1000);
    bool http_read(uint32_t timeout = 1000);
    // bool http_read_file(const char *filename, uint32_t timeout = 1000);
    // bool http_post_file(const char *filename, uint32_t timeout = 1000);
    bool read_file(const char *filename, size_t len, uint32_t timeout = 1000);
};

#endif // MQTT_A7672SA_H_