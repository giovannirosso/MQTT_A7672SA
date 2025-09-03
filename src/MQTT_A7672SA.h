/**
 * @file       MQTT_A7672SA.h
 * @author     Giovanni de Rosso Unruh
 * @date       07/2023
 * @revision   1.0.2
 */

#ifndef MQTT_A7672SA_H_
#define MQTT_A7672SA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <functional>
#include <string>

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

// #define DEBUG_LTE

#define GSM_PROGMEM

#define UART_QUEUE_SIZE 10

#define DEFAULT_CID 1

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
    char data[1024];
};

struct http_response
{
    uint32_t http_status_code;
    size_t http_header_size;
    size_t http_content_size;
    char http_etag[33];
};

struct NetworkOperator
{
    int status;
    char long_name[32];
    char short_name[16];
    char numeric_code[8];
    int access_tech;
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

enum network_mode
{
    AUTOMATIC = 2,
    GSM_ONLY = 13,
    WCDMA_ONLY = 14,
    LTE_ONLY = 38
};

class A7672SA
{
private:
    QueueHandle_t uartQueue;
    TaskHandle_t rxTaskHandle;
    TaskHandle_t txTaskHandle;
    SemaphoreHandle_t rx_guard, publish_guard;

    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t en_pin;

    bool mqtt_connected;
    bool http_response;
    bool publishing;

    struct http_response http_response_data = {0, 0, 0, ""};

    registration_status cs_reg_stat_ = UNKNOWN;  // CREG (CS domain)
    registration_status ps_reg_stat_ = UNKNOWN;  // CGREG (PS 2G/3G)
    registration_status eps_reg_stat_ = UNKNOWN; // CEREG (PS LTE/EPS)

    bool pdn_active[11] = {false}; // suporta CIDs 1..10

    bool at_ok;
    bool at_error;
    bool at_ready;
    bool at_input;
    bool at_publish;
    bool silent_mode = false;
    char *at_response;
    uint32_t rx_buffer_size;
    std::vector<NetworkOperator> available_operators;
    bool operators_list_updated;

    void (*on_message_callback_)(mqtt_message &message);
    void (*on_mqtt_status_)(mqtt_status &status);
    void (*on_ps_reg_event_)(registration_status stat) = nullptr;

    void on_ps_lost_();
    void apply_creg_(registration_status st);
    void apply_cgreg_(registration_status st);
    void apply_cereg_(registration_status st);

    void rx_task();
    static void rx_taskImpl(void *pvParameters);
    void tx_task();
    static void tx_taskImpl(void *pvParameters);

    int send_cmd_to_simcomm(const char *logName, const char *data);
    int send_cmd_to_simcomm(const char *logName, uint8_t *data, int len);

    bool receiveCommand(commandMessage *message);

    void simcomm_response_parser(const char *data);
    char **simcom_split_messages(const char *data, int *n_messages);

public:
    A7672SA();
    A7672SA(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin, int32_t baud_rate = 115200, uint32_t rx_buffer_size = 1024);
    ~A7672SA();

    void RX_LOCK(uint32_t timeout = portMAX_DELAY);
    void RX_UNLOCK();
    bool PUBLISH_LOCK(uint32_t timeout = portMAX_DELAY);
    void PUBLISH_UNLOCK();
    void REINIT_UART(uint32_t resize = 1024, bool at_ready = true);
    void DEINIT_UART();

    void on_message_callback(void (*callback)(mqtt_message &message))
    {
        on_message_callback_ = callback;
    }

    void on_mqtt_status(void (*callback)(mqtt_status &status))
    {
        on_mqtt_status_ = callback;
    }

    void on_ps_reg_event(void (*callback)(registration_status stat))
    {
        on_ps_reg_event_ = callback;
    }

    bool ps_ready() const
    { // Dados prontos
        return (eps_reg_stat_ == REGISTERED_HOME || eps_reg_stat_ == REGISTERED_ROAMING) ||
               (ps_reg_stat_ == REGISTERED_HOME || ps_reg_stat_ == REGISTERED_ROAMING);
    }

    bool cs_ready() const
    { // Voz/SMS prontos
        return (cs_reg_stat_ == REGISTERED_HOME || cs_reg_stat_ == REGISTERED_ROAMING);
    }

    registration_status cs_registration() const { return cs_reg_stat_; }
    registration_status ps_registration() const { return ps_reg_stat_; }
    registration_status eps_registration() const { return eps_reg_stat_; }

    void handle_cgreg_stat_(registration_status st);

    bool is_pdn_active(int cid = 1) const { return (cid >= 0 && cid < 11) ? pdn_active[cid] : false; }

    /** Set silent mode - se marcado como true, ao religar a uart não enviao os comandos de URC*/
    void set_silent_mode(bool mode) { silent_mode = mode; }

    void sendCommand(const char *log, const char *data, bool publish = false);
    void sendCommand(const char *log, uint8_t *data, int len, bool publish = false);

    bool wait_for_condition(uint32_t timeout, std::function<bool()> condition_check, const char *operation_name);
    bool wait_input(uint32_t timeout = 2000);
    bool wait_publish(uint32_t timeout = 2000);
    bool wait_network(uint32_t timeout = 10000);
    bool wait_response(uint32_t timeout = 2000);
    bool wait_to_connect(uint32_t timeout = 10000);
    bool wait_http_response(uint32_t timeout = 10000);
    bool wait_read(size_t len, uint32_t timeout = 120000);

    bool begin();
    bool stop();
    bool is_ready();
    bool restart(uint32_t timeout = 1000);
    bool test_at(uint32_t timeout = 1000);
    bool sim_ready(uint32_t timeout = 1000);
    int signal_quality(uint32_t timeout = 1000);

    bool ping(const char *host = "www.google.com", uint32_t timeout = 2000);

    bool set_network_mode(network_mode mode, uint32_t timeout = 1000);
    std::vector<NetworkOperator> get_operator_list(uint32_t timeout = 60000);

    bool set_operator(bool automatic, NetworkOperator op, uint32_t timeout = 1000);
    bool set_apn(const char *apn, const char *user, const char *password, uint32_t timeout = 1000);
    bool set_ntp_server(const char *ntp_server, int time_zone, uint32_t timeout = 1000);

    time_t get_ntp_time(uint32_t timeout = 1000);
    String get_provider_name(uint32_t timeout = 1000);
    String get_imei(uint32_t timeout = 1000);
    String get_iccid(uint32_t timeout = 1000);
    IPAddress get_local_ip(uint32_t timeout = 1000);
    String get_local_ipv6(uint32_t timeout = 10000);

    bool set_ca_cert(const char *ca_cert, const char *ca_name, size_t cert_size, uint32_t timeout = 10000);
    bool mqtt_connect(const char *host, uint16_t port, const char *clientId, bool clean_session = true, const char *username = "", const char *password = "", bool ssl = false, const char *ca_name = "ca.pem", uint16_t keepalive = 60, uint32_t timeout = 10000);
    bool mqtt_disconnect(uint32_t timeout = 1000);
    bool mqtt_release_client(uint32_t timeout = 1000);
    bool mqtt_publish(const char *topic, uint8_t *data, size_t len, uint16_t qos = 0, uint32_t timeout = 3000);
    bool mqtt_subscribe_topics(const char *topic[10], int n_topics = 10, uint16_t qos = 0, uint32_t timeout = 1000);
    bool mqtt_subscribe(const char *topic, uint16_t qos, uint32_t timeout = 1000);
    bool mqtt_is_connected();

    /*
    <url> URL of network resource.String,start with "http://" or"https://" a)http://’server’ :’tcpPort’ /’path’. b)https://’server’ :’tcpPort’ /’path’. "server" DNS domain name or IP address "path" path to a file or directory of a server "tcpPort" http default value is 80,https default value is 443.(canbeomitted)
    <method> HTTP request method, enum HTTP_METHOD, range is 0-4. 0: GET, 1: POST, 2: HEAD 3: DELETE, 4: PUT.
    <save_to_fs> Whether to save the response content to file system, Boolean type. false: not save, true: save. Default is false.
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
    uint32_t http_request(const char *url, HTTP_METHOD method, bool save_to_fs = false, bool ssl = false, const char *ca_name = "ca.pem",
                          const char *user_data = "", size_t user_data_size = 0, uint32_t con_timeout = 120, uint32_t recv_timeout = 120,
                          const char *content = "text/plain", const char *accept = "*/*", uint8_t read_mode = 0, const char *data_post = "", size_t size = 0, uint32_t timeout = 30000);

    uint32_t http_request_file(const char *url, HTTP_METHOD method, const char *filename, bool ssl = false, const char *ca_name = "ca.pem",
                               const char *user_data = "", size_t user_data_size = 0, uint32_t con_timeout = 120, uint32_t recv_timeout = 120,
                               const char *content = "text/plain", const char *accept = "*/*", uint8_t read_mode = 0, const char *data_post = "", size_t size = 0, uint32_t timeout = 30000);
    void http_read_file(const char *filename, uint32_t timeout = 1000);
    bool http_term(uint32_t timeout = 1000);
    void http_save_response(bool https = false);
    size_t http_read_response(uint8_t *buffer, size_t read_size, size_t offset = 0, uint32_t timeout = 1000);
    uint32_t http_response_size();
    uint32_t http_response_header_size();
    char *http_response_etag();

    /*
    <filename> The name of the file to be created, String type, max length is 256.
    <mode> The mode to open the file, Numeric type, range is 0-2.
    0 - if the file does not exist,it will be created. If the file exists, it will be directly opened. And both of them can be read and written.
    1 - if the file does not exist,it will be created. If the file exists, it will be overwritten and cleared. And both of them can be read and written.
    2 - if the file exist, open it and it can be read only. When thefiledoesnot exist, it will respond an error
    default is 2.
    <timeout> Timeout for accessing file.
    */
    bool fs_open(const char *filename, uint16_t mode = 2, uint32_t timeout = 1000);
    bool fs_close(uint32_t timeout = 1000);
    uint32_t fs_size(const char *filename, uint32_t timeout = 1000);
    bool fs_delete(const char *filename, uint32_t timeout = 1000);
    void fs_list_files(uint32_t timeout = 1000);
    size_t fs_read(size_t read_size, uint8_t *buffer, uint32_t timeout = 1000);
};

#endif // MQTT_A7672SA_H_