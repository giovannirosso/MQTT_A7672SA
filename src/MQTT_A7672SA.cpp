/**
 * @file       MQTT_A7672SA.cpp
 * @author     Giovanni de Rosso Unruh
 * @date       07/2023
 */

#include "MQTT_A7672SA.h"

A7672SA::A7672SA(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin, int32_t baud_rate, uint32_t rx_buffer_size)
{
    this->at_ready = false;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->en_pin = en_pin;
    this->rx_buffer_size = rx_buffer_size;
    this->mqtt_connected = false;

    const uart_config_t uart_config =
        {
            .baud_rate = baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

    uart_driver_install(UART_NUM_1, rx_buffer_size * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

A7672SA::~A7672SA()
{
    if (this->mqtt_connected)
    {
        this->mqtt_disconnect();
    }

    if (this->at_response != NULL)
    {
        free(this->at_response);
    }

    uart_driver_delete(UART_NUM_1);

    gpio_set_level(this->en_pin, 1);

    ESP_LOGI("DESTRUCTOR", "SIMCOMM Stopped");
}

void A7672SA::rx_taskImpl(void *pvParameters)
{
    static_cast<A7672SA *>(pvParameters)->rx_task();
}

void A7672SA::rx_task() //++ UART Receive Task
{
    static const char *RX_TASK_TAG = "SIM_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    this->at_response = (char *)malloc(this->rx_buffer_size + 1);

    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, this->at_response, this->rx_buffer_size, 300 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            this->at_response[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, this->at_response);
            this->simcomm_response_parser(this->at_response); //++ Call the AT Response Parser Function
        }
    }
    free(this->at_response);
}

void A7672SA::tx_taskImpl(void *pvParameters)
{
    static_cast<A7672SA *>(pvParameters)->tx_task();
}

void A7672SA::tx_task()
{
    static const char *TX_TASK_TAG = "SIM_TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    while (this->at_ready == false)
    {
        send_cmd_to_simcomm("AT_READY", "AT+CFUN=1" GSM_NL);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        send_cmd_to_simcomm("AT_READY", "AT+CFUN?" GSM_NL);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    send_cmd_to_simcomm("AT_ATE0", "ATE0" GSM_NL);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void A7672SA::simcomm_response_parser(const char *data) //++ Parser to parse AT Responses from Simcomm
{
    if (strstr(data, "+CMQTTCONNECT: 0,0" GSM_NL))
    {
        ESP_LOGI("PARSER", "MQTT Connected");
        this->mqtt_connected = true;
    }
    // else if (strstr(data, "+CMQTTSTART: 19" GSM_NL))
    // {
    //     ESP_LOGI("PARSER", "fail to start"); // todo need to release client
    //     this->mqtt_connected = false;
    // }
    else if (strstr(data, "+CFUN: 1" GSM_NL)) // ++ AT Response for *ATREADY: 1
    {
        ESP_LOGI("PARSER", "+CFUN: 1");
        this->at_ready = true;
    }
    else if (strstr(data, GSM_OK)) //++ AT Response for OK
    {
        ESP_LOGI("PARSER", "AT Successful");
        this->at_ok = true;
    }
    else if (strstr(data, GSM_ERROR)) //++ AT Response for ERROR
    {
        ESP_LOGI("PARSER", "AT Failed");
        this->at_ok = false;
    }
    else if (strstr(data, ">"))
    {
        ESP_LOGI("PARSER", "AT Input");
        this->at_input = true;
    }
    else if (strstr(data, "+CMQTTRECV:"))
    {
        mqtt_message message;

        String payloadString(data);

        int firstCommaIndex = payloadString.indexOf(',');
        int secondCommaIndex = payloadString.indexOf(',', firstCommaIndex + 1);
        int thirdCommaIndex = payloadString.indexOf(',', secondCommaIndex + 1);

        message.topic = payloadString.substring(firstCommaIndex + 2, secondCommaIndex - 1);
        message.topic.trim();

        String messageLenStr = payloadString.substring(secondCommaIndex + 1, thirdCommaIndex);
        messageLenStr.trim();
        message.length = messageLenStr.toInt();

        message.data = payloadString.substring(thirdCommaIndex + 2, thirdCommaIndex + 2 + message.length);
        message.data.trim();

        if (this->mqtt_callback != NULL)
            this->mqtt_callback(message);
    }
    else if (strstr(data, GSM_NL "+CMQTTCONNLOST:"))
    {
        ESP_LOGI("PARSER", "MQTT Disconnected");
        this->mqtt_connected = false;
    }
    else
    {
        ESP_LOGI("PARSER", "Unhandeled AT Response %s", data);
    }
}

bool A7672SA::begin()
{
    gpio_set_direction(this->en_pin, GPIO_MODE_OUTPUT); //++ Set GPIO Pin Directions

    ESP_LOGI("BEGIN", "Enable Pin: %d", this->en_pin);

    gpio_set_level(this->en_pin, 1); //++ Restarting Simcomm via ENABLE pin
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(this->en_pin, 0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    xTaskCreate(this->rx_taskImpl, "uart_rx_task", 2048 * 2, this, configMAX_PRIORITIES, NULL); //++ Create FreeRtos Tasks
    xTaskCreate(this->tx_taskImpl, "uart_tx_task", 2048 * 2, this, configMAX_PRIORITIES - 1, NULL);

    ESP_LOGI("BEGIN", "SIMCOMM Started");
    return true;
}

bool A7672SA::restart(uint32_t timeout)
{
    this->send_cmd_to_simcomm("RESTART", "AT+CRESET" GSM_NL);
    return this->wait_response(timeout);
}

int A7672SA::send_cmd_to_simcomm(const char *logName, const char *data) //++ Sending AT Commands to Simcomm via UART
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

bool A7672SA::wait_response(uint32_t timeout)
{
    this->at_ok = false;
    uint32_t start = millis();
    while (millis() - start < timeout)
    {
        if (this->at_ok)
        {
            return true;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return false;
}

bool A7672SA::wait_input(uint32_t timeout)
{
    this->at_input = false;
    uint32_t start = millis();
    while (millis() - start < timeout)
    {
        if (this->at_input)
        {
            return true;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return false;
}

bool A7672SA::wait_to_connect(uint32_t timeout)
{
    uint32_t start = millis();
    while (millis() - start < timeout)
    {
        if (this->mqtt_connected)
        {
            return true;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return false;
}

bool A7672SA::test_at(uint32_t timeout)
{
    this->send_cmd_to_simcomm("AT_TEST", "AT" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::sim_ready(uint32_t timeout)
{
    this->send_cmd_to_simcomm("SIM_READY", "AT+CPIN?" GSM_NL);
    return this->wait_response(timeout);
}

int A7672SA::signal_quality(uint32_t timeout)
{
    this->send_cmd_to_simcomm("SIGNAL_QUALITY", "AT+CSQ" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String signal_quality = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        signal_quality = data_string.substring(data_string.indexOf(":") + 1, data_string.indexOf(","));

        return signal_quality.toInt();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return 99;
}

bool A7672SA::set_apn(const char *apn, uint32_t timeout)
{
    char data[100];
    sprintf(data, "AT+CGDCONT=1,\"IP\",\"%s\"" GSM_NL, apn);
    this->send_cmd_to_simcomm("SET_APN", data);
    if (this->wait_response(timeout))
    {
        this->send_cmd_to_simcomm("SET_APN", "AT+CGACT=1,1" GSM_NL);
        if (this->wait_response(timeout))
        {
            this->send_cmd_to_simcomm("SET_APN", "AT+CREG=1" GSM_NL);
            return this->wait_response(timeout);
        }
    }
    return false;
}

bool A7672SA::wait_network(uint32_t timeout)
{
    this->send_cmd_to_simcomm("WAIT_NETWORK", "AT+CREG?" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String network_status = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        network_status = data_string.substring(data_string.indexOf(",") + 1, data_string.indexOf(",", data_string.indexOf(",") + 1));

        if (network_status.toInt() == REGISTERED_HOME || network_status.toInt() == REGISTERED_ROAMING)
        {
            return true;
        }
    }
    return false;
}

bool A7672SA::set_ntp_server(const char *ntp_server, int time_zone, uint32_t timeout)
{
    char data[100];
    sprintf(data, "AT+CNTP=\"%s\",%d" GSM_NL, ntp_server, time_zone);
    this->send_cmd_to_simcomm("SET_NTP_SERVER", data);
    if (this->wait_response(timeout))
    {
        this->send_cmd_to_simcomm("SET_NTP_SERVER", "AT+CNTP" GSM_NL);
        return this->wait_response(timeout);
    }
}

String A7672SA::get_ntp_time(uint32_t timeout)
{
    this->send_cmd_to_simcomm("GET_NTP_TIME", "AT+CCLK?" GSM_NL);
    if (wait_response(timeout))
    {
        String data_string = "";
        for (int i = 0; i <= strlen(this->at_response); i++)
        {
            if (this->at_response[i] == 'C' && this->at_response[i + 1] == 'C' && this->at_response[i + 2] == 'L' && this->at_response[i + 3] == 'K' && this->at_response[i + 4] == ':')
            {
                int index_of_data = i + 7;
                int index_of_time_str = 0;
                char sntp_time_string[100];
                memset(sntp_time_string, 0, sizeof(sntp_time_string));
                do
                {
                    sntp_time_string[index_of_time_str] = this->at_response[index_of_data];
                    index_of_time_str++;
                    index_of_data++;
                } while (this->at_response[index_of_data] != '+');

                // todo aa/mm/dd,hh:mm:ss+tz convert to timestamp

                for (int j = 0; j < strlen(sntp_time_string); j++)
                {
                    data_string += sntp_time_string[j];
                }

                return data_string;
            }
        }
    }
    return "";
}

String A7672SA::get_provider_name(uint32_t timeout)
{
    this->send_cmd_to_simcomm("GET_PROVIDER_NAME", "AT+CSPN?" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String provider_name = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        provider_name = data_string.substring(data_string.indexOf("\"") + 1, data_string.indexOf("\"", data_string.indexOf("\"") + 1));

        return provider_name;
    }
    return "";
}

String A7672SA::get_imei(uint32_t timeout)
{
    this->send_cmd_to_simcomm("GET_IMEI", "AT+CGSN" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String imei = "";
        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        data_string.trim();

        imei = data_string.substring(0, data_string.indexOf("\n") - 1);

        return imei;
    }
    return "";
}

String A7672SA::get_local_ip(uint32_t timeout)
{
    this->send_cmd_to_simcomm("GET_LOCAL_IP", "AT+CGPADDR" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String local_ip = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        local_ip = data_string.substring(data_string.indexOf(",") + 1, data_string.indexOf(GSM_OK) - 2);

        return local_ip;
    }
    return "";
}

bool A7672SA::set_ca_cert(const char *ca_cert, const char *ca_name, size_t cert_size, uint32_t timeout)
{
    char data[100];
    sprintf(data, "AT+CCERTDOWN=\"%s\",%d" GSM_NL, ca_name, cert_size);
    printf("data: %s\n", data);
    this->send_cmd_to_simcomm("SET_CA_CERT", data);
    if (this->wait_input(timeout))
    {
        int tx_bytes = uart_write_bytes(UART_NUM_1, ca_cert, cert_size);
        ESP_LOGI("SET_CA_CERT", "Wrote %d bytes", tx_bytes);
        return this->wait_response(timeout);
    }
    return false;
}

bool A7672SA::mqtt_connect(const char *host, uint16_t port, const char *clientId, const char *username, const char *password, bool ssl, const char *ca_name, uint16_t keepalive, uint32_t timeout)
{
    if (ssl)
    {
        this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CSSLCFG=\"sslversion\",0,4" GSM_NL);
        if (this->wait_response(timeout))
            this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CSSLCFG=\"authmode\",0,1" GSM_NL);
        if (this->wait_response(timeout))
            this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CSSLCFG=\"enableSNI\",0,0" GSM_NL);
        if (this->wait_response(timeout))
        {
            char data[100];
            sprintf(data, "AT+CSSLCFG=\"cacert\",0,\"%s\"" GSM_NL, ca_name);
            this->send_cmd_to_simcomm("MQTT_CONNECT", data);
            if (this->wait_response(timeout))
                this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CMQTTSTART" GSM_NL);
            if (this->wait_response(timeout))
            {
                char data[100];
                sprintf(data, "AT+CMQTTACCQ=0,\"%s\",1" GSM_NL, clientId);
                this->send_cmd_to_simcomm("MQTT_CONNECT", data);
                if (this->wait_response(timeout))
                {
                    this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CMQTTSSLCFG=0,0" GSM_NL);
                    if (this->wait_response(timeout))
                        this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CMQTTCFG=\"argtopic\",0,1,1" GSM_NL);
                    if (this->wait_response(timeout))
                    {
                        char *data;
                        if (username == NULL || username == "")
                        {
                            data = (char *)malloc(strlen(host) + 40);
                            sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,1" GSM_NL, host, port, keepalive);
                        }
                        else
                        {
                            data = (char *)malloc(strlen(host) + strlen(clientId) + strlen(password) + 40);
                            sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,1,\"%s\",\"%s\"" GSM_NL, host, port, keepalive, clientId, password);
                        }
                        printf("data: %s\n", data);
                        this->send_cmd_to_simcomm("MQTT_CONNECT", data);
                        return this->wait_to_connect(timeout);
                    }
                }
            }
        }
    }
    else
    {
        this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CMQTTSTART" GSM_NL);
        if (this->wait_response(timeout))
        {
            char data[100];
            sprintf(data, "AT+CMQTTACCQ=0,\"%s\"" GSM_NL, clientId);
            this->send_cmd_to_simcomm("MQTT_CONNECT", data);
            if (this->wait_response(timeout))
            {
                this->send_cmd_to_simcomm("MQTT_CONNECT", "AT+CMQTTCFG=\"argtopic\",0,1,1" GSM_NL);
                if (this->wait_response(timeout))
                {
                    char *data;
                    if (username == NULL || username == "")
                    {
                        data = (char *)malloc(strlen(host) + 40);
                        sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,1" GSM_NL, host, port, keepalive);
                    }
                    else
                    {
                        data = (char *)malloc(strlen(host) + strlen(clientId) + strlen(password) + 40);
                        sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,1,\"%s\",\"%s\"" GSM_NL, host, port, keepalive, clientId, password);
                    }
                    printf("data: %s\n", data);
                    this->send_cmd_to_simcomm("MQTT_CONNECT", data);
                    return this->wait_to_connect(timeout);
                }
            }
        }
    }
}

bool A7672SA::mqtt_disconnect(uint32_t timeout)
{
    this->send_cmd_to_simcomm("MQTT_DISCONNECT", "AT+CMQTTDISC=0,120" GSM_NL);
    if (this->wait_response(timeout))
    {
        this->mqtt_connected = false;
        this->send_cmd_to_simcomm("MQTT_DISCONNECT", "AT+CMQTTREL=0" GSM_NL);
    }
    this->wait_response(timeout);
    this->send_cmd_to_simcomm("MQTT_DISCONNECT", "AT+CMQTTSTOP" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::mqtt_publish(const char *topic, const char *data, uint16_t qos, uint32_t timeout)
{
    char *data_string = (char *)malloc(strlen(data) + strlen(topic) + 30);
    sprintf(data_string, "AT+CMQTTPUB=0,\"%s\",%d,%d" GSM_NL, topic, qos, strlen(data));
    this->send_cmd_to_simcomm("MQTT_PUBLISH", data_string);
    if (this->wait_input(timeout))
    {
        this->send_cmd_to_simcomm("MQTT_PUBLISH", data);
        return this->wait_response(timeout);
    }
    return false;
}

bool A7672SA::mqtt_subscribe(const char *topic, uint16_t qos, uint32_t timeout)
{
    char *data_string = (char *)malloc(strlen(topic) + 30);
    sprintf(data_string, "AT+CMQTTSUB=0,\"%s\",%d" GSM_NL, topic, qos);
    this->send_cmd_to_simcomm("MQTT_SUBSCRIBE", data_string);
    return this->wait_response(timeout);
}

bool A7672SA::mqtt_is_connected()
{
    return this->mqtt_connected;
}

bool A7672SA::is_ready()
{
    return this->at_ready;
}