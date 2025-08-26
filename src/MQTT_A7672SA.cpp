/*@file MQTT_A7672SA.cpp
/**
 * @author     Giovanni de Rosso Unruh
 * @date       07/2023
 */

#include "MQTT_A7672SA.h"

A7672SA::A7672SA()
{
    this->at_ok = false;
    this->at_ready = false;
    this->at_input = false;
    this->at_publish = false;
    this->mqtt_connected = false;
    this->http_response = false;
    this->publishing = false;
    this->operators_list_updated = false;
}

A7672SA::A7672SA(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t en_pin, int32_t baud_rate, uint32_t rx_buffer_size)
{
    this->at_ok = false;
    this->at_ready = false;
    this->at_input = false;
    this->at_publish = false;
    this->mqtt_connected = false;
    this->http_response = false;
    this->publishing = false;
    this->operators_list_updated = false;

    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->en_pin = en_pin;
    this->rx_buffer_size = rx_buffer_size;

    const uart_config_t uart_config =
        {
            .baud_rate = baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, rx_buffer_size * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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

    ESP_LOGV("DESTRUCTOR", "SIMCOMM Stopped");
}

bool A7672SA::begin()
{
    gpio_set_direction(this->en_pin, GPIO_MODE_OUTPUT); //++ Set GPIO Pin Directions

    ESP_LOGV("BEGIN", "Enable Pin: %d", this->en_pin);

    gpio_set_level(this->en_pin, 1); //++ Restarting Simcomm via ENABLE pin
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(this->en_pin, 0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    this->rx_guard = xSemaphoreCreateMutex();               //++ Create FreeRtos Semaphore
    this->publish_guard = xSemaphoreCreateRecursiveMutex(); //++ Recursive Mutex for publish (reentrancy-safe)

    uartQueue = xQueueCreate(UART_QUEUE_SIZE, sizeof(commandMessage));

    xTaskCreate(this->rx_taskImpl, "uart_rx_task", configIDLE_TASK_STACK_SIZE * 5, this, configMAX_PRIORITIES - 5, &rxTaskHandle); //++ Create FreeRtos Tasks //todo tamanho da memoria
    xTaskCreate(this->tx_taskImpl, "uart_tx_task", configIDLE_TASK_STACK_SIZE * 5, this, configMAX_PRIORITIES - 6, &txTaskHandle);

    ESP_LOGV("BEGIN", "SIMCOMM Started");
    return true;
}

bool A7672SA::stop()
{
    if (this->mqtt_connected)
    {
        this->mqtt_disconnect();
    }

    if (this->at_response != NULL)
    {
        free(this->at_response);
    }

    DEINIT_UART();

    ESP_LOGV("STOP", "SIMCOMM Stopped");
    return true;
}

void A7672SA::sendCommand(const char *logName, const char *data, bool publish)
{
#ifdef DEBUG_LTE
    ESP_LOGV("SEND_COMMAND", "ok:%d", this->at_ok);
#endif
    this->at_ok = false;
    commandMessage message;
    strcpy(message.logName, logName);
    strcpy(message.data, data);
    xQueueSend(uartQueue, &message, portMAX_DELAY);
}

void A7672SA::sendCommand(const char *logName, uint8_t *data, int len, bool publish)
{
#ifdef DEBUG_LTE
    ESP_LOGV("SEND_COMMAND", "ok:%d", this->at_ok);
#endif
    this->at_ok = false;
    commandMessage message;
    strcpy(message.logName, logName);
    memcpy(message.data, data, len);
    xQueueSend(uartQueue, &message, portMAX_DELAY);
}

bool A7672SA::receiveCommand(commandMessage *message)
{
    if (uxQueueMessagesWaiting(uartQueue) > 0)
    {
        xQueueReceive(uartQueue, message, 0);
        return true;
    }
    return false;
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

    if (silent_mode == false)
    {
        send_cmd_to_simcomm("AT_ATE0", "ATE0" GSM_NL);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        send_cmd_to_simcomm("CMEE", "AT+CMEE=2" GSM_NL); // verbose errors
        vTaskDelay(500 / portTICK_PERIOD_MS);
        send_cmd_to_simcomm("CREG=1", "AT+CREG=1" GSM_NL); // URC de registro 2G
        vTaskDelay(500 / portTICK_PERIOD_MS);
        send_cmd_to_simcomm("CGREG=1", "AT+CGREG=1" GSM_NL); // URC de registro PS
        vTaskDelay(500 / portTICK_PERIOD_MS);
        send_cmd_to_simcomm("CEREG=1", "AT+CEREG=1" GSM_NL); // URC de registro LTE
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // send_cmd_to_simcomm("AT_ATV1", "ATV1" GSM_NL); // error roport
    // vTaskDelay(500 / portTICK_PERIOD_MS);

    // send_cmd_to_simcomm("AT+CMEE", "AT+CMEE=1" GSM_NL); // cme error report
    // vTaskDelay(500 / portTICK_PERIOD_MS);

    while (1)
    {
        commandMessage receivedCommand;
        if (receiveCommand(&receivedCommand))
        {
            send_cmd_to_simcomm(receivedCommand.logName, receivedCommand.data);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
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
        this->RX_LOCK();
        const int rxBytes = uart_read_bytes(UART_NUM_1, this->at_response, this->rx_buffer_size, 100 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            this->at_response[rxBytes] = 0;
#ifdef DEBUG_LTE
            ESP_LOGV(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, this->at_response);
#endif
            int n_messages = 0;
            char **messages = this->simcom_split_messages(this->at_response, &n_messages);
            for (int i = 0; i < n_messages; i++)
            {
#ifdef DEBUG_LTE
                ESP_LOGV(RX_TASK_TAG, "Message %d: '%s'", i, messages[i]);
#endif
                this->simcomm_response_parser(messages[i]);
                free(messages[i]);
            }
            free(messages);
        }
        this->RX_UNLOCK();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(this->at_response);
}

void A7672SA::RX_LOCK(uint32_t timeout)
{
    if (this->rx_guard != NULL)
        do
        {
        } while (xSemaphoreTake(this->rx_guard, timeout) != pdPASS);
}

void A7672SA::RX_UNLOCK()
{
    if (this->rx_guard != NULL)
        xSemaphoreGive(this->rx_guard);
}

bool A7672SA::PUBLISH_LOCK(uint32_t timeout) // todo testar
{
    if (this->publish_guard == NULL)
    {
        // No guard available, treat as locked
        this->publishing = true;
        return true;
    }

    // Convert milliseconds timeout to ticks, guard against overflow
    TickType_t ticks = (timeout == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);

    // If we're in the modem RX/TX task context, do not block to avoid deadlocks and asserts
    TaskHandle_t cur = xTaskGetCurrentTaskHandle();
    if (cur == rxTaskHandle || cur == txTaskHandle)
    {
        ticks = 0; // non-blocking try
    }

#ifdef DEBUG_LTE
    TaskHandle_t owner = (this->publish_guard != NULL) ? xSemaphoreGetMutexHolder(this->publish_guard) : NULL;
    ESP_LOGV("PUBLISH_LOCK", "cur=%p owner=%p ticks=%u", (void *)cur, (void *)owner, (unsigned)ticks);
#endif

    // Attempt to take the mutex once with the provided timeout
    if (xSemaphoreTakeRecursive(this->publish_guard, ticks) == pdPASS)
    {
        this->publishing = true;
        return true;
    }

    // Failed to acquire
    return false;
}

void A7672SA::PUBLISH_UNLOCK()
{
    if (this->publish_guard != NULL)
    {
#ifdef DEBUG_LTE
        TaskHandle_t owner = xSemaphoreGetMutexHolder(this->publish_guard);
        ESP_LOGV("PUBLISH_UNLOCK", "owner(before)=%p", (void *)owner);
#endif
        xSemaphoreGiveRecursive(this->publish_guard);
    }

    this->publishing = false;
}

void A7672SA::DEINIT_UART()
{
    this->RX_LOCK();

    vTaskDelete(rxTaskHandle);
    vTaskDelete(txTaskHandle);

    if (uart_is_driver_installed(UART_NUM_1))
    {
        int ret = uart_driver_delete(UART_NUM_1);
        if (ret != ESP_OK)
        {
            ESP_LOGE("DEINIT_UART", "Error deleting UART driver: %d", ret);
        }
    }
}

void A7672SA::REINIT_UART(uint32_t resize, bool at_ready)
{
    this->at_ready = at_ready;
    this->rx_buffer_size = resize;

    const uart_config_t uart_config =
        {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, this->rx_buffer_size, 0, 0, NULL, 0));

    xTaskCreate(this->rx_taskImpl, "uart_rx_task", configIDLE_TASK_STACK_SIZE * 5, this, configMAX_PRIORITIES - 5, &rxTaskHandle); //++ Create FreeRtos Tasks //todo tamanho da memoria
    xTaskCreate(this->tx_taskImpl, "uart_tx_task", configIDLE_TASK_STACK_SIZE * 5, this, configMAX_PRIORITIES - 6, &txTaskHandle);

    this->RX_UNLOCK();
}

char **A7672SA::simcom_split_messages(const char *data, int *n_messages)
{
    char **messages = NULL;
    *n_messages = 0;

    char *data_copy = strdup(data);
    if (!data_copy)
    {
        ESP_LOGE("SPLIT_MESSAGES", "Falha ao alocar memória para data_copy");
        return NULL;
    }

    char *token = strtok(data_copy, GSM_NM);
    while (token != NULL)
    {
        if (strlen(token) > 0)
        {
            char **temp = (char **)realloc(messages, (*n_messages + 1) * sizeof(char *));
            if (!temp)
            {
                ESP_LOGE("SPLIT_MESSAGES", "Falha ao realocar memória para messages");
                // Libera a memória já alocada
                for (int i = 0; i < *n_messages; i++)
                {
                    free(messages[i]);
                }
                free(messages);
                free(data_copy);
                return NULL;
            }
            messages = temp;

            messages[*n_messages] = strdup(token);
            if (!messages[*n_messages])
            {
                ESP_LOGE("SPLIT_MESSAGES", "Falha ao alocar memória para messages[%d]", *n_messages);
                // Libera a memória já alocada
                for (int i = 0; i < *n_messages; i++)
                {
                    free(messages[i]);
                }
                free(messages);
                free(data_copy);
                return NULL;
            }

            (*n_messages)++;
        }
        token = strtok(NULL, GSM_NM);
    }
    free(data_copy);
    return messages;
}

static int parse_cid_from_cgev(const char *p)
{
    // Procura primeiro número após o prefixo CGEV...
    while (*p && !(*p >= '0' && *p <= '9'))
        ++p;
    int cid = 0;
    if (*p)
        cid = atoi(p);
    return cid;
}

using ResponseHandler = std::function<void(const char *, const char *)>;

void A7672SA::simcomm_response_parser(const char *data)
{
    // Mapa de padrões para funções de callback
    static const std::vector<std::pair<const char *, ResponseHandler>> response_handlers = {
        {"CMQTTCONNECT: 0,0" GSM_NL, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "MQTT Connected");
             this->mqtt_connected = true;
             mqtt_status status = A7672SA_MQTT_CONNECTED;
             if (this->on_mqtt_status_ != NULL)
                 this->on_mqtt_status_(status);
         }},
        {"CMQTTSTART: 19" GSM_NL, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "fail to start");
             this->at_ok = false;
             this->mqtt_connected = false;
             mqtt_status status = A7672SA_MQTT_CLIENT_USED;
             if (this->on_mqtt_status_ != NULL)
                 this->on_mqtt_status_(status);
             this->mqtt_release_client();
         }},
        {"CPIN: SIM REMOVED" GSM_NL, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "SIM Removed");
             this->at_error = true;
             this->at_ok = false;
         }},
        {"CFUN: 1" GSM_NL, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "CFUN: 1");
             this->at_ready = true;
         }},
        {"CMQTTPUB: 0,0" GSM_NL, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "Publish OK");
             this->at_publish = true;
         }},
        {"CMQTTSUB: 0,0" GSM_NL, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "Subscribe OK");
         }},
        {GSM_NL ">", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "AT Input");
             this->at_input = true;
         }},
        {GSM_NL "DOWNLOAD", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "AT Input");
             this->at_input = true;
         }},
        {GSM_ERROR, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "AT ERROR");
             this->at_error = true;
             this->at_ok = false;
             this->at_input = false;
             this->at_publish = false;
             this->http_response = false;
         }},
        {"CME ERROR:", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "AT ERROR");
             this->at_error = true;
             this->at_ok = false;
             this->at_input = false;
             this->at_publish = false;
             this->http_response = false;
         }},
        {"CMQTTRECV:", [this](const char *data, const char *found)
         {
             String payloadString(data);
             int currentIndex = 0;
             int mqttRecvIndex = payloadString.indexOf("CMQTTRECV:", currentIndex);

             while (mqttRecvIndex != -1)
             {
                 currentIndex = mqttRecvIndex;
                 int firstCommaIndex = payloadString.indexOf(',', currentIndex);
                 int secondCommaIndex = payloadString.indexOf(',', firstCommaIndex + 1);
                 int thirdCommaIndex = payloadString.indexOf(',', secondCommaIndex + 1);

                 String topic = payloadString.substring(firstCommaIndex + 2, secondCommaIndex - 1).c_str();
                 topic.trim();
                 int messageLength = payloadString.substring(secondCommaIndex + 1, thirdCommaIndex).toInt();

                 String payload = payloadString.substring(thirdCommaIndex + 2, thirdCommaIndex + 2 + messageLength);

                 mqtt_message message;
                 message.topic = (char *)malloc(topic.length() + 1);
                 strcpy(message.topic, topic.c_str());
                 message.length = messageLength;
                 message.payload = new uint8_t[messageLength];
                 memcpy(message.payload, payload.c_str(), messageLength);

                 if (this->on_message_callback_ != nullptr)
                 {
                     this->on_message_callback_(message);
                 }

                 free(message.topic);
                 delete[] message.payload;

                 mqttRecvIndex = payloadString.indexOf("CMQTTRECV:", currentIndex + 1);
             }
         }},
        {"CMQTTCONNLOST:", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "MQTT Disconnected");
             mqtt_status status = A7672SA_MQTT_DISCONNECTED;
             this->mqtt_connected = false;
             if (this->on_mqtt_status_ != NULL)
                 this->on_mqtt_status_(status);
             this->mqtt_disconnect();
         }},
        {"CMQTTDISC:", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "MQTT Disconnected");
             mqtt_status status = A7672SA_MQTT_DISCONNECTED;
             this->mqtt_connected = false;
             if (this->on_mqtt_status_ != NULL)
                 this->on_mqtt_status_(status);
             this->mqtt_disconnect();
         }},
        {GSM_NL "PB DONE", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "PB DONE");
         }},
        {"HTTPACTION:", [this](const char *data, const char *found)
         {
             int method, errcode, datalen;
             sscanf(found, "HTTPACTION: %d,%d,%d" GSM_NL, &method, &this->http_response_data.http_status_code, &this->http_response_data.http_content_size);
             ESP_LOGV("PARSER", "METHOD: %d, ERRORCODE: %d, DATALEN: %d", method, this->http_response_data.http_status_code, this->http_response_data.http_content_size);
             this->http_response = true;
         }},
        {"HTTPPOSTFILE:", [this](const char *data, const char *found)
         {
             int method;
             sscanf(found, "HTTPPOSTFILE: %d,%d,%d" GSM_NL, &method, &this->http_response_data.http_status_code, &this->http_response_data.http_content_size);
             ESP_LOGV("PARSER", "METHOD: %d, ERRORCODE: %d, DATALEN: %d", method, this->http_response_data.http_status_code, this->http_response_data.http_content_size);
             this->http_response = true;
         }},
        {"HTTPHEAD:", [this](const char *data, const char *found)
         {
             // Parse header size
             int hdr = 0;
             if (sscanf(found, "HTTPHEAD: %d" GSM_NL, &hdr) == 1)
             {
                 this->http_response_data.http_header_size = hdr;
                 ESP_LOGV("PARSER", "HEADLEN: %d", hdr);
             }

             // Work on a safe copy for searching headers
             const char *p = found;
             // Content-Length (case-insensitive)
             const char *cl = strstr(p, "Content-Length:");
             if (!cl)
             {
                 cl = strstr(p, "content-length:");
             }
             if (cl)
             {
                 int clen = 0;
                 if (sscanf(cl, "Content-Length: %d", &clen) == 1 || sscanf(cl, "content-length: %d", &clen) == 1)
                 {
                     this->http_response_data.http_content_size = clen;
                     ESP_LOGV("PARSER", "CONTENT_LENGTH: %d", clen);
                 }
             }

             // ETag (case-insensitive)
             const char *et = strstr(p, "etag:");
             if (!et)
             {
                 et = strstr(p, "ETag:");
             }
             if (et)
             {
                 // Copy token after etag:
                 char tag[sizeof(this->http_response_data.http_etag)] = {0};
                 if (sscanf(et, "etag: %31s", tag) == 1 || sscanf(et, "ETag: %31s", tag) == 1)
                 {
                     strncpy(this->http_response_data.http_etag, tag, sizeof(this->http_response_data.http_etag) - 1);
                     this->http_response_data.http_etag[sizeof(this->http_response_data.http_etag) - 1] = '\0';
                     ESP_LOGV("PARSER", "ETAG: %s", this->http_response_data.http_etag);
                 }
             }
             this->http_response = true;
         }},
        {"CPING:", [this](const char *data, const char *found)
         {
             // todo Handle three possible formats
             // 1) +CPING: <result_type>,<resolved_ip_addr>,<data_packet_size>,<rtt>,<TTL>
             // 2) +CPING: <result_type>
             // 3) +CPING: <result_type>,<num_pkts_sent>,<num_pkts_recvd>,<num_pkts_lost>,<min_rtt>,<max_rtt>,<avg_rtt>
             // We'll try to parse in a robust order: type-only, type-1, type-3.
             ESP_LOGV("PARSER", "Recebida resposta do comando CPING");
             this->at_ok = true;
         }},
        {"COPS:", [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "Recebida resposta do comando COPS");

             this->available_operators.clear();

             String response(found);
             if (response.indexOf("(") > 0)
             {
                 int start = response.indexOf("(");
                 int end = response.lastIndexOf(")");

                 if (start > 0 && end > start)
                 {
                     String operatorListStr = response.substring(start, end + 1);
                     int currentPos = 0;
                     while (true)
                     {
                         int openParen = operatorListStr.indexOf('(', currentPos);
                         if (openParen == -1)
                             break;

                         int closeParen = operatorListStr.indexOf(')', openParen);
                         if (closeParen == -1)
                             break;

                         String operatorData = operatorListStr.substring(openParen + 1, closeParen);
                         NetworkOperator op = {0};
                         int commaPos = 0;
                         int startPos = 0;
                         int fieldIndex = 0;

                         while (true)
                         {
                             commaPos = operatorData.indexOf(',', startPos);
                             String fieldValue;

                             if (commaPos == -1)
                             {
                                 fieldValue = operatorData.substring(startPos);
                             }
                             else
                             {
                                 fieldValue = operatorData.substring(startPos, commaPos);
                                 startPos = commaPos + 1;
                             }

                             switch (fieldIndex)
                             {
                             case 0: // Status
                                 op.status = fieldValue.toInt();
                                 break;
                             case 1: // Nome longo
                                 if (fieldValue.startsWith("\"") && fieldValue.endsWith("\""))
                                 {
                                     fieldValue = fieldValue.substring(1, fieldValue.length() - 1);
                                 }
                                 strncpy(op.long_name, fieldValue.c_str(), sizeof(op.long_name) - 1);
                                 op.long_name[sizeof(op.long_name) - 1] = '\0';
                                 break;
                             case 2: // Nome curto
                                 if (fieldValue.startsWith("\"") && fieldValue.endsWith("\""))
                                 {
                                     fieldValue = fieldValue.substring(1, fieldValue.length() - 1);
                                 }
                                 strncpy(op.short_name, fieldValue.c_str(), sizeof(op.short_name) - 1);
                                 op.short_name[sizeof(op.short_name) - 1] = '\0';
                                 break;
                             case 3: // Código numérico
                                 if (fieldValue.startsWith("\"") && fieldValue.endsWith("\""))
                                 {
                                     fieldValue = fieldValue.substring(1, fieldValue.length() - 1);
                                 }
                                 strncpy(op.numeric_code, fieldValue.c_str(), sizeof(op.numeric_code) - 1);
                                 op.numeric_code[sizeof(op.numeric_code) - 1] = '\0';
                                 break;
                             case 4: // Tecnologia de acesso
                                 op.access_tech = fieldValue.toInt();
                                 break;
                             }

                             fieldIndex++;
                             if (commaPos == -1 || fieldIndex >= 5)
                             {
                                 break;
                             }
                         }

                         if (strlen(op.numeric_code) > 0)
                         {
                             this->available_operators.push_back(op);
                             ESP_LOGV("PARSER", "Operadora: %s (%s), Código: %s, Status: %d, Tecnologia: %d",
                                      op.long_name, op.short_name, op.numeric_code, op.status, op.access_tech);
                         }
                         currentPos = closeParen + 1;
                     }

                     this->operators_list_updated = true;
                     this->at_ok = true;
                     ESP_LOGV("PARSER", "Processadas %d operadoras", this->available_operators.size());
                 }
             }
         }}, // --- CGEV: EPS PDN ACT <cid>
        {"CGEV: EPS PDN ACT", [this](const char *data, const char *found)
         {
             int cid = parse_cid_from_cgev(found);
             if (cid >= 0 && cid < 11)
                 pdn_active[cid] = true;
             ESP_LOGV("PARSER", "CGEV: EPS PDN ACT, cid=%d", cid);
         }},
        // --- CGEV: EPS PDN DEACT <cid>
        {"CGEV: EPS PDN DEACT", [this](const char *data, const char *found)
         {
             int cid = parse_cid_from_cgev(found);
             if (cid >= 0 && cid < 11)
                 pdn_active[cid] = false;
             ESP_LOGW("PARSER", "CGEV: EPS PDN DEACT, cid=%d", cid);
             // Sessões de app caíram
             if (cid == DEFAULT_CID)
             {
                 this->mqtt_connected = false;
                 if (this->on_mqtt_status_)
                 {
                     mqtt_status st = A7672SA_MQTT_DISCONNECTED;
                     this->on_mqtt_status_(st);
                 }
             }
         }},
        // --- CGEV: NW PDN ACT <cid>
        {"CGEV: NW PDN ACT", [this](const char *data, const char *found)
         {
             int cid = parse_cid_from_cgev(found);
             if (cid >= 0 && cid < 11)
                 pdn_active[cid] = true;
             ESP_LOGV("PARSER", "CGEV: NW PDN ACT, cid=%d", cid);
         }},
        // --- CGEV: NW PDN DEACT <cid>
        {"CGEV: NW PDN DEACT", [this](const char *data, const char *found)
         {
             int cid = parse_cid_from_cgev(found);
             if (cid >= 0 && cid < 11)
                 pdn_active[cid] = false;
             ESP_LOGW("PARSER", "CGEV: NW PDN DEACT, cid=%d", cid);
             if (cid == DEFAULT_CID)
             {
                 this->mqtt_connected = false;
                 if (this->on_mqtt_status_)
                 {
                     mqtt_status st = A7672SA_MQTT_DISCONNECTED;
                     this->on_mqtt_status_(st);
                 }
             }
         }},
        // --- CGEV: ME DEACT
        {"CGEV: ME DEACT", [this](const char *data, const char *found)
         {
             // Conservador: marcar todos CIDs como inativos //todo testar
             for (int i = 0; i < 11; i++)
                 pdn_active[i] = false;
             ESP_LOGW("PARSER", "CGEV: ME DEACT (desativacao local do(s) PDN)");
             this->mqtt_connected = false;
             if (this->on_mqtt_status_)
             {
                 mqtt_status st = A7672SA_MQTT_DISCONNECTED;
                 this->on_mqtt_status_(st);
             }
         }},
        // --- CGEV: ME DETACH
        {"CGEV: ME DETACH", [this](const char *data, const char *found)
         {
             for (int i = 0; i < 11; i++)
                 pdn_active[i] = false;
             ESP_LOGE("PARSER", "CGEV: ME DETACH (module detach)");
             this->mqtt_connected = false;
             if (this->on_mqtt_status_)
             {
                 mqtt_status st = A7672SA_MQTT_DISCONNECTED;
                 this->on_mqtt_status_(st);
             }
         }},
        // -
        {"CGEV: ME PDN ACT", [this](const char *data, const char *found)
         {
             int cid = parse_cid_from_cgev(found);
             if (cid >= 0 && cid < 11)
                 pdn_active[cid] = true;
             ESP_LOGV("PARSER", "CGEV: ME PDN ACT, cid=%d", cid);
         }},
        {"CGEV: ME PDN DEACT", [this](const char *data, const char *found)
         {
             int cid = parse_cid_from_cgev(found);
             if (cid >= 0 && cid < 11)
                 pdn_active[cid] = false;
             ESP_LOGW("PARSER", "CGEV: ME PDN DEACT, cid=%d", cid);
             if (cid == DEFAULT_CID)
             {
                 this->mqtt_connected = false;
                 if (this->on_mqtt_status_)
                 {
                     mqtt_status st = A7672SA_MQTT_DISCONNECTED;
                     this->on_mqtt_status_(st);
                 }
             }
         }},
        {"CREG: ", [this](const char *data, const char *found)
         {
             // Aceita "CREG: <stat>" ou "CREG: <n>,<stat>"
             int stat = -1, n = -1;
             if (sscanf(found, "CREG: %d,%d", &n, &stat) != 2)
             {
                 sscanf(found, "CREG: %d", &stat);
             }
             if (stat >= 0 && stat <= 7)
             {
                 apply_creg_(static_cast<registration_status>(stat));
                 ESP_LOGV("PARSER", "CREG stat=%d", stat);
             }
             else
             {
                 ESP_LOGW("PARSER", "CREG stat invalido: '%s'", found);
             }
         }},
        {"CEREG: ", [this](const char *data, const char *found)
         {
             int stat = -1, n = -1;
             if (sscanf(found, "CEREG: %d,%d", &n, &stat) != 2)
             {
                 sscanf(found, "CEREG: %d", &stat);
             }
             // idem: sanitize valores estendidos
             if (stat >= 0 && stat <= 7)
             {
                 apply_cereg_(static_cast<registration_status>(stat));
             }
             else
             {
                 apply_cereg_(UNKNOWN);
             }
             ESP_LOGV("PARSER", "CEREG stat=%d", stat);
         }},
        {"CGREG: ", [this](const char *data, const char *found)
         {
             int stat = -1, n = -1;
             if (sscanf(found, "CGREG: %d,%d", &n, &stat) != 2)
             {
                 sscanf(found, "CGREG: %d", &stat);
             }
             // Alguns firmwares emitem 10/11 etc.; trate >7 como UNKNOWN
             if (stat >= 0 && stat <= 7)
             {
                 apply_cgreg_(static_cast<registration_status>(stat));
             }
             else
             {
                 apply_cgreg_(UNKNOWN);
             }
             ESP_LOGV("PARSER", "CGREG stat=%d", stat);
         }},
        {GSM_OK, [this](const char *data, const char *found)
         {
             ESP_LOGV("PARSER", "AT Successful");
             this->at_ok = true;
         }},
    };

    if (strcmp(data, GSM_NL) == 0)
    {
        return;
    }

    for (const auto &handler : response_handlers)
    {
        const char *found = strstr(data, handler.first);
        if (found != nullptr)
        {
            handler.second(data, found);
            return;
        }
    }

    ESP_LOGV("PARSER", "Unhandled AT Response %s", data); //++ Unhandled AT Response
}

void A7672SA::on_ps_lost_()
{
    // Qualquer queda de PS implica derrubar app-layer (MQTT/HTTP)
    if (mqtt_connected)
    {
        mqtt_connected = false;
        if (on_mqtt_status_)
        {
            mqtt_status s = A7672SA_MQTT_DISCONNECTED;
            on_mqtt_status_(s);
        }
    }
    // aqui você pode acordar sua task de reconexão para refazer CGATT/CGACT/TCP/MQTT com backoff
}

void A7672SA::apply_creg_(registration_status st)
{
    bool before = cs_ready();
    cs_reg_stat_ = st;
    (void)before; // hoje CS não derruba MQTT, mas deixe aqui se um dia precisar reagir
}

void A7672SA::apply_cgreg_(registration_status st)
{
    bool before = ps_ready();
    ps_reg_stat_ = st;
    bool after = ps_ready();
    if (before && !after)
        on_ps_lost_();
}

void A7672SA::apply_cereg_(registration_status st)
{
    bool before = ps_ready();
    eps_reg_stat_ = st;
    bool after = ps_ready();
    if (before && !after)
        on_ps_lost_();
}

bool A7672SA::restart(uint32_t timeout)
{
    this->sendCommand("RESTART", "AT+CRESET" GSM_NL);
    return this->wait_response(timeout);
}

int A7672SA::send_cmd_to_simcomm(const char *logName, uint8_t *data, int len) //++ Sending AT Commands to Simcomm via UART
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
#ifdef DEBUG_LTE
    ESP_LOGV(logName, "Wrote %d bytes of %d requested", txBytes, len);
    printf("DATA WROTE BYTES [%s] ", logName);
    for (int i = 0; i < len; i++)
    {
        printf("%02X", data[i]);
    }
    printf("\n");
#endif
    return txBytes;
}

int A7672SA::send_cmd_to_simcomm(const char *logName, const char *data) //++ Sending AT Commands to Simcomm via UART
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
#ifdef DEBUG_LTE
    ESP_LOGV(logName, "Wrote %d bytes of %d requested", txBytes, len);
    printf("DATA WROTE CHARS [%s] ", logName);
    for (int i = 0; i < len; i++)
    {
        printf("%c", data[i]);
    }
    printf("\n");
#endif
    return txBytes;
}

/**
 * @brief Aguarda por uma condição específica com timeout
 * @param timeout Tempo máximo de espera em ms
 * @param condition_check Função que verifica a condição desejada
 * @param operation_name Nome da operação para log
 * @return true se a condição foi satisfeita, false caso contrário
 */
bool A7672SA::wait_for_condition(uint32_t timeout, std::function<bool()> condition_check, const char *operation_name)
{
    uint32_t start = millis();
    ESP_LOGV("WAIT", "Iniciando espera por %s, timeout: %d ms", operation_name, timeout);

    while (!condition_check() && millis() - start < timeout)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, this->at_response, this->rx_buffer_size, 250 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            this->at_response[rxBytes] = 0;
            this->simcomm_response_parser(this->at_response);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    bool result = condition_check();
    ESP_LOGV("WAIT", "Espera por %s finalizada: %s", operation_name, result ? "OK" : "FAIL");
    return result;
}

bool A7672SA::wait_response(uint32_t timeout)
{
    this->at_input = false;
    // this->at_ok = false;
    this->at_publish = false;
    this->at_error = false;
    this->http_response = false;

    if (this->at_ok)
        return true;

    // Wait for either OK or ERROR response
    bool result = wait_for_condition(timeout, [this]()
                                     { return this->at_ok || this->at_error; }, "AT RESPONSE");

    // Return false if an error occurred, even if the condition was met
    if (this->at_error)
    {
        return false;
    }

    this->at_ok = false;

    return result;
}

bool A7672SA::wait_input(uint32_t timeout)
{
    // this->at_input = false;
    this->at_ok = false;
    this->at_publish = false;
    this->at_error = false;
    this->http_response = false;

    if (this->at_input)
        return true;

    bool result = wait_for_condition(timeout, [this]()
                                     { return this->at_input || this->at_error; }, "AT INPUT");

    if (this->at_error)
    {
        return false;
    }

    this->at_input = false;

    return result;
}

bool A7672SA::wait_publish(uint32_t timeout)
{
    this->at_input = false;
    this->at_ok = false;
    // this->at_publish = false;
    this->at_error = false;
    this->http_response = false;

    if (this->at_publish)
        return true;

    bool result = wait_for_condition(timeout, [this]()
                                     { return this->at_publish || this->at_error; }, "MQTT PUBLISH");
    if (this->at_error)
    {
        return false;
    }

    this->at_publish = false;

    return result;
}

bool A7672SA::wait_to_connect(uint32_t timeout)
{
    this->at_input = false;
    this->at_ok = false;
    this->at_publish = false;
    this->at_error = false;
    this->http_response = false;
    // this->mqtt_connected = false;

    if (this->mqtt_connected)
        return true;

    return wait_for_condition(timeout, [this]()
                              { return this->mqtt_connected; }, "MQTT CONNECT");
}

bool A7672SA::wait_http_response(uint32_t timeout)
{
    this->at_input = false;
    this->at_ok = false;
    this->at_publish = false;
    this->at_error = false;
    this->http_response = false;

    if (this->http_response)
        return true;

    bool result = wait_for_condition(timeout, [this]()
                                     { return this->http_response || this->at_error; }, "HTTP RESPONSE");

    if (this->at_error)
    {
        return false;
    }

    this->http_response = false;

    return result;
}

bool A7672SA::test_at(uint32_t timeout)
{
    this->sendCommand("AT_TEST", "AT" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::sim_ready(uint32_t timeout)
{
    this->sendCommand("SIM_READY", "AT+CPIN?" GSM_NL);
    return this->wait_response(timeout);
}

int A7672SA::signal_quality(uint32_t timeout)
{
    if (this->publishing)
        return 0;

    this->sendCommand("SIGNAL_QUALITY", "AT+CSQ" GSM_NL);
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
    return 0;
}

bool A7672SA::set_operator(NetworkOperator op, uint32_t timeout)
{
    if (this->publishing)
        return false;

    char data[100];
    sprintf(data, "AT+COPS=0,2,\"%s\",%d" GSM_NL, op.numeric_code, op.access_tech);
    ESP_LOGV("SET_OPERATOR", "AT+COPS=0,2,\"%s\",%d", op.numeric_code, op.access_tech);
    this->sendCommand("SET_OPERATOR", data);

    return this->wait_response(timeout);
}

bool A7672SA::set_network_mode(network_mode mode, uint32_t timeout)
{
    if (this->publishing)
        return false;

    char data[100];
    sprintf(data, "AT+CNMP=%d" GSM_NL, mode);
    this->sendCommand("SET_NETWORK_MODE", data);

    return this->wait_response(timeout);
}

bool A7672SA::set_apn(const char *apn, const char *user, const char *password, uint32_t timeout)
{
    if (this->publishing)
        return false;

    this->sendCommand("SET_APN", "AT+CREG=1" GSM_NL);
    this->wait_response(timeout);
    this->sendCommand("SET_APN", "AT+CEREG=1" GSM_NL);
    this->wait_response(timeout);
    this->sendCommand("SET_APN", "AT+CGREG=1" GSM_NL);
    this->wait_response(timeout);

    char data[100];
    sprintf(data, "AT+CGDCONT=1,\"IPV4V6\",\"%s\"" GSM_NL, apn);
    this->sendCommand("SET_APN", data);
    if (this->wait_response(timeout))
    {
        sprintf(data, "AT+CGAUTH=1,1,\"%s\",\"%s\"" GSM_NL, user, password);
        this->sendCommand("SET_APN", data);
        if (this->wait_response(timeout))
        {
            sprintf(data, "AT+CGATT=1" GSM_NL);
            this->sendCommand("SET_APN", data);
            if (this->wait_response(timeout))
            {
                this->sendCommand("SET_APN", "AT+CGACT=1,1" GSM_NL);
                return this->wait_response(timeout);
            }
        }
    }
    return false;
}

bool A7672SA::wait_network(uint32_t timeout_ms)
{
    if (this->publishing)
        return false;

    auto ready = [this]()
    { return ps_ready(); };

    // Fallback opcional: se tudo UNKNOWN logo após boot, faça UMA consulta
    if (!ready())
    {
        if (ps_registration() == UNKNOWN && eps_registration() == UNKNOWN)
        {
            sendCommand("WAIT_NETWORK", "AT+CGREG?" GSM_NL);
            (void)wait_response(2000); // seu parser já aplicará o estado
            if (!ready())
            {
                sendCommand("WAIT_NETWORK", "AT+CEREG?" GSM_NL);
                (void)wait_response(2000);
            }
        }
    }
    return wait_for_condition(timeout_ms, ready, "wait_network(PS)");
}

bool A7672SA::set_ntp_server(const char *ntp_server, int time_zone, uint32_t timeout)
{
    if (this->publishing)
        return false;

    char data[100];
    sprintf(data, "AT+CNTP=\"%s\",%d" GSM_NL, ntp_server, time_zone);
    this->sendCommand("SET_NTP_SERVER", data);
    if (this->wait_response(timeout))
    {
        this->sendCommand("SET_NTP_SERVER", "AT+CNTP" GSM_NL);
        return this->wait_response(timeout);
    }
    return false;
}

time_t convertToTimestamp(const char *arry)
{
    struct tm timeStruct = {0};

    sscanf(arry, "%d/%d/%d,%d:%d:%d", &timeStruct.tm_year, &timeStruct.tm_mon,
           &timeStruct.tm_mday, &timeStruct.tm_hour, &timeStruct.tm_min, &timeStruct.tm_sec);

    timeStruct.tm_year += 100; // Years since 1900
    timeStruct.tm_mon -= 1;    // Months are 0-based

    time_t timestamp = mktime(&timeStruct);

    ESP_LOGV("CONVERT_TO_TIMESTAMP", "Timestamp: %ld", timestamp);
    if (timestamp < 0)
        timestamp = 0;

    return timestamp;
}

/*
72400	Nextel
72401	SISTEER DO BRASIL TELECOMUNICACOES
72402	TIM
72403	TIM
72404	TIM
72405	Claro
72406	Vivo
72410	Vivo
72411	Vivo
72415	Sercomtel
72416	Brasil Telecom GSM
72417	Correios
72418	datora
72423	Vivo
72424	Amazonia Celular
72430	Oi
72431	Oi
72432	Algar Telecom
72433	Algar Telecom
72434	Algar Telecom
72435	Telcom Telecomunicacoes
72436	Options Telecomunicacoes
72437	aeiou
72438	Claro
72439	Nextel
72454	Conecta
72499	Local
*/
/**
 * @brief Obtém a lista de operadoras de rede disponíveis
 * @param timeout Tempo máximo de espera pela resposta em ms
 * @return Vetor com as operadoras de rede encontradas
 */
std::vector<NetworkOperator> A7672SA::get_operator_list(uint32_t timeout)
{
    if (this->publishing)
    {
        ESP_LOGV("GET_OPERATOR_LIST", "Não foi possível executar: publicação em andamento");
        return this->available_operators;
    }

    this->operators_list_updated = false;

    if (this->available_operators.size() > 0)
    {
        ESP_LOGV("GET_OPERATOR_LIST", "Lista de operadoras já carregada, não é necessário executar novamente");
        return this->available_operators;
    }

    this->sendCommand("GET_OPERATOR_LIST", "AT+COPS=?" GSM_NL);

    bool result = wait_for_condition(timeout, [this]()
                                     { return this->operators_list_updated || this->at_error; }, "lista de operadoras");

    if (result)
    {
        ESP_LOGV("GET_OPERATOR_LIST", "Lista de operadoras atualizada com sucesso");
    }
    else
    {
        ESP_LOGV("GET_OPERATOR_LIST", "Timeout ao aguardar lista de operadoras");
    }

    return this->available_operators;
}

time_t A7672SA::get_ntp_time(uint32_t timeout)
{
    if (this->publishing)
        return 0;

    this->sendCommand("GET_NTP_TIME", "AT+CCLK?" GSM_NL);
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

                return convertToTimestamp(sntp_time_string);
            }
        }
    }
    return 0;
}

/*
AT+CSPN anwser:
>
> +CSPN: "TIM",1
>
> OK
*/
String A7672SA::get_provider_name(uint32_t timeout)
{
    if (this->publishing)
        return "NO SIM";

    this->sendCommand("GET_PROVIDER_NAME", "AT+CSPN?" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String provider_name = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        data_string.trim();

        provider_name = data_string.substring(data_string.indexOf("\"") + 1, data_string.indexOf("\"", data_string.indexOf("\"") + 1));

        ESP_LOGV("GET_PROVIDER_NAME", "PROVIDER_NAME: %s", provider_name.c_str());

        return provider_name;
    }
    return "NO SIM";
}

/*
AT+CGSN anwser:
>
> 860710050359929
>
> OK
*/
String A7672SA::get_imei(uint32_t timeout)
{
    if (this->publishing)
        return "0";

    this->sendCommand("GET_IMEI", "AT+CGSN" GSM_NL);
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

        ESP_LOGV("GET_IMEI", "IMEI: %s", imei.c_str());

        // validate IMEI
        if (imei.length() == 15)
            return imei;
    }
    return "0";
}

/*
AT+CICCID
+ICCID: 89860318760238610932
*/
String A7672SA::get_iccid(uint32_t timeout)
{
    if (this->publishing)
        return "0";

    this->sendCommand("GET_ICCID", "AT+CICCID" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String iccid = "";
        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        data_string.trim();

        iccid = data_string.substring(data_string.indexOf(":") + 1, data_string.indexOf("\n") - 1);

        ESP_LOGV("GET_ICCID", "ICCID: %s", iccid.c_str());

        return iccid;
    }
    return "0";
}

/*
AT+CGPADDR anwser:
>
> +CGPADDR: <cid>,<PDP_addr_IPV4>,<PDP_addr_IPV6>
> EXAMPLE:
> +CGPADDR: 8,41.3.5.144,254.128.0.0.0.0.0.0.24.69.231.10.121.241.106.179
>
> OK
*/
IPAddress A7672SA::get_local_ip(uint32_t timeout)
{
    if (this->publishing)
        return IPAddress(0, 0, 0, 0);

    this->sendCommand("GET_LOCAL_IP", "AT+CGPADDR" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String ipv4_str = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        data_string.trim();

        // Formato esperado: +CGPADDR: 8,41.3.5.144,254.128.0.0.0.0.0.0.24.69.231.10.121.241.106.179
        // Encontra a primeira vírgula
        int firstComma = data_string.indexOf(",");
        if (firstComma < 0)
            return IPAddress(0, 0, 0, 0);

        // Encontra a segunda vírgula (que separa IPv4 do IPv6)
        int secondComma = data_string.indexOf(",", firstComma + 1);

        // Se não houver segunda vírgula, pegamos até o final da linha
        if (secondComma < 0)
        {
            ipv4_str = data_string.substring(firstComma + 1, data_string.indexOf("\n") - 1);
        }
        else
        {
            // Caso contrário, pegamos da primeira à segunda vírgula
            ipv4_str = data_string.substring(firstComma + 1, secondComma);
        }

        ESP_LOGV("GET_LOCAL_IP", "IPv4: %s", ipv4_str.c_str());

        // get the four octets of the IPv4 address
        int octet1 = ipv4_str.substring(0, ipv4_str.indexOf(".")).toInt();
        ipv4_str.remove(0, ipv4_str.indexOf(".") + 1);
        int octet2 = ipv4_str.substring(0, ipv4_str.indexOf(".")).toInt();
        ipv4_str.remove(0, ipv4_str.indexOf(".") + 1);
        int octet3 = ipv4_str.substring(0, ipv4_str.indexOf(".")).toInt();
        ipv4_str.remove(0, ipv4_str.indexOf(".") + 1);
        int octet4 = ipv4_str.toInt();

        IPAddress local_ip(octet1, octet2, octet3, octet4);
        return local_ip;
    }
    return IPAddress(0, 0, 0, 0);
}

String A7672SA::get_local_ipv6(uint32_t timeout)
{
    if (this->publishing)
        return "";

    this->sendCommand("GET_LOCAL_IPV6", "AT+CGPADDR" GSM_NL);
    if (this->wait_response(timeout))
    {
        String data_string = "";
        String ipv6_str = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        data_string.trim();

        // Formato esperado: +CGPADDR: 8,41.3.5.144,254.128.0.0.0.0.0.0.24.69.231.10.121.241.106.179
        // Precisamos encontrar a segunda vírgula para pegar o IPv6
        int firstComma = data_string.indexOf(",");
        if (firstComma < 0)
            return "";

        int secondComma = data_string.indexOf(",", firstComma + 1);
        if (secondComma < 0)
            return ""; // Não há IPv6

        // Pega do segundo separador até o final da linha
        ipv6_str = data_string.substring(secondComma + 1, data_string.indexOf("\n") - 1);

        ESP_LOGV("GET_LOCAL_IPV6", "IPv6: %s", ipv6_str.c_str());
        return ipv6_str;
    }
    return "";
}

bool A7672SA::set_ca_cert(const char *ca_cert, const char *ca_name, size_t cert_size, uint32_t timeout)
{
    char data[100];
    this->at_input = false;
    sprintf(data, "AT+CCERTDOWN=\"%s\",%d" GSM_NL, ca_name, cert_size);
    this->sendCommand("SET_CA_CERT", data);
    if (this->wait_input(timeout))
    {
        int tx_bytes = uart_write_bytes(UART_NUM_1, ca_cert, cert_size);
        ESP_LOGV("SET_CA_CERT", "Wrote %d bytes", tx_bytes);
        return this->wait_response(timeout);
    }
    return false;
}

bool A7672SA::mqtt_connect(const char *host, uint16_t port, const char *clientId, bool clean_session, const char *username, const char *password, bool ssl, const char *ca_name, uint16_t keepalive, uint32_t timeout)
{
    if (ssl)
    {
        this->sendCommand("MQTT_CONNECT", "AT+CSSLCFG=\"sslversion\",0,4" GSM_NL);
        if (this->wait_response(timeout))
            this->sendCommand("MQTT_CONNECT", "AT+CSSLCFG=\"authmode\",0,1" GSM_NL);
        if (this->wait_response(timeout))
            this->sendCommand("MQTT_CONNECT", "AT+CSSLCFG=\"enableSNI\",0,0" GSM_NL);
        if (this->wait_response(timeout))
        {
            char cmd[100];
            sprintf(cmd, "AT+CSSLCFG=\"cacert\",0,\"%s\"" GSM_NL, ca_name);
            this->sendCommand("MQTT_CONNECT", cmd);
            if (this->wait_response(timeout))
                this->sendCommand("MQTT_CONNECT", "AT+CMQTTSTART" GSM_NL);
            if (this->wait_response(timeout))
            {
                sprintf(cmd, "AT+CMQTTACCQ=0,\"%s\",1" GSM_NL, clientId);
                this->sendCommand("MQTT_CONNECT", cmd);
                if (this->wait_response(timeout))
                {
                    this->sendCommand("MQTT_CONNECT", "AT+CMQTTSSLCFG=0,0" GSM_NL);
                    if (this->wait_response(timeout))
                        this->sendCommand("MQTT_CONNECT", "AT+CMQTTCFG=\"checkUTF8\",0,0" GSM_NL);
                    if (this->wait_response(timeout))
                        this->sendCommand("MQTT_CONNECT", "AT+CMQTTCFG=\"argtopic\",0,1,1" GSM_NL);
                    if (this->wait_response(timeout))
                    {
                        const size_t data_size = strlen(host) + strlen(username) + strlen(password) + 50;
                        char data[data_size];
                        if (username == "")
                        {
                            sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,%d" GSM_NL, host, port, keepalive, clean_session);
                        }
                        else
                        {
                            sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,%d,\"%s\",\"%s\"" GSM_NL, host, port, keepalive, clean_session, username, password);
                        }
                        this->mqtt_connected = false;
                        this->sendCommand("MQTT_CONNECT", data);
                        bool result = this->wait_to_connect(timeout);
                        return result;
                    }
                }
            }
        }
    }
    else
    {
        this->sendCommand("MQTT_CONNECT", "AT+CMQTTSTART" GSM_NL);
        if (this->wait_response(timeout))
        {
            char cmd[100];
            sprintf(cmd, "AT+CMQTTACCQ=0,\"%s\"" GSM_NL, clientId);
            this->sendCommand("MQTT_CONNECT", cmd);
            if (this->wait_response(timeout))
                this->sendCommand("MQTT_CONNECT", "AT+CMQTTCFG=\"checkUTF8\",0,0" GSM_NL);
            if (this->wait_response(timeout))
                this->sendCommand("MQTT_CONNECT", "AT+CMQTTCFG=\"argtopic\",0,1,1" GSM_NL);
            if (this->wait_response(timeout))
            {
                const size_t data_size = strlen(host) + strlen(username) + strlen(password) + 50;
                char data[data_size];
                if (username == "")
                {
                    sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,%d" GSM_NL, host, port, keepalive, clean_session);
                }
                else
                {
                    sprintf(data, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,%d,\"%s\",\"%s\"" GSM_NL, host, port, keepalive, clean_session, username, password);
                }
                this->mqtt_connected = false;
                this->sendCommand("MQTT_CONNECT", data);
                bool result = this->wait_to_connect(timeout);
                return result;
            }
        }
    }
    return false;
}

bool A7672SA::mqtt_disconnect(uint32_t timeout)
{
    this->sendCommand("MQTT_DISCONNECT", "AT+CMQTTDISC=0,120" GSM_NL);
    if (this->wait_response(timeout))
    {
        this->sendCommand("MQTT_DISCONNECT", "AT+CMQTTREL=0" GSM_NL);
        this->wait_response(timeout);
    }
    this->sendCommand("MQTT_DISCONNECT", "AT+CMQTTSTOP" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::mqtt_publish(const char *topic, uint8_t *data, size_t len, uint16_t qos, uint32_t timeout)
{
    if (data == nullptr || len == 0)
    {
        ESP_LOGE("MQTT_PUBLISH", "Data is null or length is zero");
        return false;
    }

    // Check for forbidden sequences "AT" //todo testar
    for (size_t i = 0; i + 2 < len; ++i)
    {
        if (data[i] == 'A' && data[i + 1] == 'T')
        {
            ESP_LOGE("MQTT_PUBLISH", "Payload contains forbidden sequence 'AT': reject");
            return false;
        }
    }

    const size_t data_size = strlen(topic) + len + 50;
    char data_string[data_size];
    ESP_LOGV("MQTT_PUBLISH", "LEN =  %d bytes", len);

    this->at_publish = false;
    this->at_input = false;
    sprintf(data_string, "AT+CMQTTPUB=0,\"%s\",%d,%d" GSM_NL, topic, qos, len);
    this->sendCommand("MQTT_PUBLISH_CMD", data_string);
    if (this->wait_input(timeout))
    {
        this->send_cmd_to_simcomm("MQTT_PUBLISH_DATA", data, len);
        if (this->wait_publish(timeout))
        {
            return true;
        }
    }
    return false;
}

bool A7672SA::mqtt_subscribe_topics(const char *topic[10], int n_topics, uint16_t qos, uint32_t timeout)
{
    if (this->publishing)
        return false;

    for (int i = 0; i < n_topics; i++)
    {
        const size_t data_size = strlen(topic[i]) + 50;
        char data_string[data_size];
        this->at_input = false;
        sprintf(data_string, "AT+CMQTTSUBTOPIC=0,%d,%d" GSM_NL, strlen(topic[i]), qos);
        this->sendCommand("MQTT_SUBSCRIBE", data_string);
        if (this->wait_input(timeout))
        {
            int tx_bytes = uart_write_bytes(UART_NUM_1, topic[i], strlen(topic[i]));
            ESP_LOGV("MQTT_SUBSCRIBE", "Wrote %d bytes", tx_bytes);
            // this->sendCommand("MQTT_SUBSCRIBE", topic[i]);
            this->wait_response(timeout);
        }
    }
    this->sendCommand("MQTT_SUBSCRIBE", "AT+CMQTTSUB=0" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::mqtt_subscribe(const char *topic, uint16_t qos, uint32_t timeout)
{
    if (this->publishing)
        return false;

    const size_t data_size = strlen(topic) + 50;
    char data_string[data_size];
    sprintf(data_string, "AT+CMQTTSUB=0,\"%s\",%d" GSM_NL, topic, qos);
    this->sendCommand("MQTT_SUBSCRIBE", data_string);
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

bool A7672SA::mqtt_release_client(uint32_t timeout)
{
    this->sendCommand("MQTT_RELEASE_CLIENT", "AT+CMQTTREL=0" GSM_NL);
    return this->wait_response(timeout);
}

/*
AT+CPING=<dest_addr>,<dest_addr_type>[,<num_pings>[,<data_packet_size>[,<interval_time>[,<wait_time>[,<TTL>]]]]]
EXAMPLE:
AT+CPING="www.baidu.com",1,4,64,1000,10000,255
*/
bool A7672SA::ping(const char *host, uint32_t timeout)
{
    if (this->publishing)
        return false;

    if (!this->ps_ready())
    {
        if (!this->wait_network(15000))
        {
            ESP_LOGE("PING", "PS não pronto (sem registro em dados)");
            return false;
        }
    }

    char data[128];
    sprintf(data, "AT+CPING=\"%s\",1,2,64,1000,2000,255" GSM_NL, host);

    this->sendCommand("PING", data);
    return this->wait_response(timeout);
}

uint32_t A7672SA::http_request(const char *url, HTTP_METHOD method, bool save_to_fs, bool ssl, const char *ca_name,
                               const char *user_data, size_t user_data_size, uint32_t con_timeout, uint32_t recv_timeout, const char *content,
                               const char *accept, uint8_t read_mode, const char *data_post, size_t size, uint32_t timeout)
{
    this->sendCommand("HTTP_INIT", "AT+HTTPINIT" GSM_NL);
    if (this->wait_response(timeout))
    {
        char cmd[100];
        sprintf(cmd, "AT+HTTPPARA=\"URL\",\"%s\"" GSM_NL, url);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);

        if (ssl)
        {
            this->sendCommand("HTTP_SSL", "AT+CSSLCFG=\"sslversion\",0,4" GSM_NL);
            if (this->wait_response(timeout))
                this->sendCommand("HTTP_SSL", "AT+CSSLCFG=\"authmode\",0,1" GSM_NL);
            if (this->wait_response(timeout))
                this->sendCommand("HTTP_SSL", "AT+CSSLCFG=\"enableSNI\",0,0" GSM_NL);
            if (this->wait_response(timeout))
            {
                sprintf(cmd, "AT+CSSLCFG=\"cacert\",0,\"%s\"" GSM_NL, ca_name);
                this->sendCommand("HTTP_SSL", cmd);
                this->wait_response(timeout);
                this->sendCommand("HTTP_SSL", "AT+HTTPPARA=\"SSLCFG\",0" GSM_NL);
                this->wait_response(timeout);
            }
        }

        sprintf(cmd, "AT+HTTPPARA=\"CONNECTTO\",%d" GSM_NL, con_timeout);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);
        sprintf(cmd, "AT+HTTPPARA=\"RECVTO\",%d" GSM_NL, recv_timeout);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);
        sprintf(cmd, "AT+HTTPPARA=\"CONTENT\",\"%s\"" GSM_NL, content);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);
        sprintf(cmd, "AT+HTTPPARA=\"ACCEPT\",\"%s\"" GSM_NL, accept);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);

        if (user_data_size > 0)
        {
            char data_[user_data_size + 40];
            sprintf(data_, "AT+HTTPPARA=\"USERDATA\",\"%s\"" GSM_NL, user_data);
            this->sendCommand("HTTP_REQUEST", data_);
            this->wait_response(timeout);
        }

        sprintf(cmd, "AT+HTTPPARA=\"READMODE\",%d" GSM_NL, read_mode);
        this->sendCommand("HTTP_REQUEST", cmd);
        if (this->wait_response(timeout))
        {
            if (method == HTTP_METHOD::POST)
            {
                ESP_LOGV("HTTP_REQUEST", "Sending HTTP POST request");
                this->at_input = false;
                sprintf(cmd, "AT+HTTPDATA=%d,%d" GSM_NL, size, timeout);
                this->sendCommand("HTTP_REQUEST", cmd);
                if (this->wait_input(timeout))
                {
                    this->at_ok = false;
                    this->send_cmd_to_simcomm("HTTP_REQUEST", (byte *)data_post, size);
                    if (this->wait_response(timeout))
                    {
                        return this->http_response_data.http_status_code;
                    }
                }
            }

            ESP_LOGV("HTTP_REQUEST", "Sending HTTP request, http_response:%d", this->http_response);
            // this->http_response = false;
            sprintf(cmd, "AT+HTTPACTION=%d" GSM_NL, method);
            this->sendCommand("HTTP_REQUEST", cmd);
            if (this->wait_http_response(recv_timeout * 1000))
            {
                // this->http_response = false;
                this->sendCommand("HTTP_REQUEST", "AT+HTTPHEAD" GSM_NL);
                this->wait_http_response(timeout);

                if (save_to_fs)
                {
                    this->http_save_response(ssl);
                    this->wait_response(timeout);
                }

                return this->http_response_data.http_status_code;
            }
        }
    }
    return 0;
}

uint32_t A7672SA::http_request_file(const char *url, HTTP_METHOD method, const char *filename, bool ssl, const char *ca_name,
                                    const char *user_data, size_t user_data_size, uint32_t con_timeout, uint32_t recv_timeout,
                                    const char *content, const char *accept, uint8_t read_mode, const char *data_post, size_t size, uint32_t timeout)
{
    char cmd[100];
    sprintf(cmd, "AT+FSOPEN=C:/%s,0" GSM_NL, filename);
    this->sendCommand("FS", cmd);
    this->wait_response(timeout);
    this->sendCommand("FS", "AT+FSCLOSE=1" GSM_NL);
    this->wait_response(timeout);
    this->sendCommand("HTTP_INIT", "AT+HTTPINIT" GSM_NL);
    if (this->wait_response(timeout))
    {
        sprintf(cmd, "AT+HTTPPARA=\"URL\",\"%s\"" GSM_NL, url);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);

        if (ssl)
        {
            this->sendCommand("HTTP_SSL", "AT+CSSLCFG=\"sslversion\",0,4" GSM_NL);
            if (this->wait_response(timeout))
                this->sendCommand("HTTP_SSL", "AT+CSSLCFG=\"authmode\",0,1" GSM_NL);
            if (this->wait_response(timeout))
                this->sendCommand("HTTP_SSL", "AT+CSSLCFG=\"enableSNI\",0,0" GSM_NL);
            if (this->wait_response(timeout))
            {
                sprintf(cmd, "AT+CSSLCFG=\"cacert\",0,\"%s\"" GSM_NL, ca_name);
                this->sendCommand("HTTP_SSL", cmd);
                this->wait_response(timeout);
                this->sendCommand("HTTP_SSL", "AT+HTTPPARA=\"SSLCFG\",0" GSM_NL);
                this->wait_response(timeout);
            }
        }

        sprintf(cmd, "AT+HTTPPARA=\"CONNECTTO\",%d" GSM_NL, con_timeout);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);
        sprintf(cmd, "AT+HTTPPARA=\"RECVTO\",%d" GSM_NL, recv_timeout);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);
        sprintf(cmd, "AT+HTTPPARA=\"CONTENT\",\"%s\"" GSM_NL, content);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);
        sprintf(cmd, "AT+HTTPPARA=\"ACCEPT\",\"%s\"" GSM_NL, accept);
        this->sendCommand("HTTP_REQUEST", cmd);
        this->wait_response(timeout);

        if (user_data_size > 0)
        {
            char data_[user_data_size + 40];
            sprintf(data_, "AT+HTTPPARA=\"USERDATA\",\"%s\"" GSM_NL, user_data);
            this->sendCommand("HTTP_REQUEST", data_);
            this->wait_response(timeout);
        }

        sprintf(cmd, "AT+HTTPPARA=\"READMODE\",%d" GSM_NL, read_mode);
        this->sendCommand("HTTP_REQUEST", cmd);
        if (this->wait_response(timeout))
        {
            ESP_LOGV("HTTP_REQUEST", "Method: %d", method);
            if (method == HTTP_METHOD::POST)
            {
                ESP_LOGV("HTTP_REQUEST", "Sending HTTP POST request");
                this->at_input = false;
                sprintf(cmd, "AT+HTTPDATA=%d,%d" GSM_NL, size, timeout);
                this->sendCommand("HTTP_REQUEST", cmd);
                if (this->wait_input(timeout))
                {
                    this->at_ok = false;
                    this->send_cmd_to_simcomm("HTTP_REQUEST", (byte *)data_post, size);
                    if (this->wait_response(timeout))
                    {
                        return this->http_response_data.http_status_code;
                    }
                }
            }
            ESP_LOGV("HTTP_REQUEST", "Sending HTTP GET/POSTFILE request");
            // this->http_response = false;
            sprintf(cmd, "AT+HTTPPOSTFILE=\"%s\",1,%d,1" GSM_NL, filename, method);
            this->sendCommand("HTTP_REQUEST", cmd);
            if (this->wait_http_response(recv_timeout * 1000))
            {
                // this->http_response = false;
                this->sendCommand("HTTP_REQUEST", "AT+HTTPHEAD" GSM_NL);
                this->wait_http_response(timeout);

                return this->http_response_data.http_status_code;
            }
        }
    }
    return 0;
}

uint32_t A7672SA::http_response_size()
{
    return this->http_response_data.http_content_size;
}

uint32_t A7672SA::http_response_header_size()
{
    return this->http_response_data.http_header_size;
}

char *A7672SA::http_response_etag()
{
    return this->http_response_data.http_etag;
}

// para que o buffer seja preenchido é necessário que que a tarefa que chama essa função pegue o mutex uart_guard
size_t A7672SA::fs_read(size_t read_size, uint8_t *buffer, uint32_t timeout)
{
    int read_len = 0;
    int res_len = 0;

    char cmd[100];
    sprintf(cmd, "AT+FSREAD=1,%d" GSM_NL, read_size);

    this->sendCommand("FS", cmd);
    const int rxBytes = uart_read_bytes(UART_NUM_1, this->at_response, this->rx_buffer_size, 250 / portTICK_RATE_MS);

    if (rxBytes > 0)
    {
        this->at_response[rxBytes] = 0;

        char *result = strstr(this->at_response, "CONNECT");
        if (result == NULL)
        {
            return -1;
        }

        sscanf(result, "CONNECT %d%*s", &read_len);
        if (read_len <= 0)
        {
            return -2;
        }
        // calcula o tamanho do read_len em dígitos, para obter o offset do buffer de resposta da UART
        // não queremos o CONNECT %d\r\n
        res_len = floor(log10(read_len)) + 1 + 12;
        memcpy(buffer, this->at_response + res_len, read_len);
    }

    return read_len;
}

size_t A7672SA::http_read_response(uint8_t *buffer, size_t read_size, size_t offset, uint32_t timeout)
{
    uint32_t start = millis();

    int read_len = 0;
    int res_len = 0;

    char cmd[100];
    sprintf(cmd, "AT+HTTPREAD=%d,%d" GSM_NL, offset, read_size);

    this->sendCommand("HTTP", cmd);
    const int rxBytes = uart_read_bytes(UART_NUM_1, this->at_response, this->rx_buffer_size, 250 / portTICK_RATE_MS);

    if (rxBytes > 0)
    {
        this->at_response[rxBytes] = 0;
        char *result = strstr(this->at_response, "+HTTPREAD");
        if (result == NULL)
        {
            return -1;
        }

        sscanf(result, "+HTTPREAD: %d%*s", &read_len);
        if (read_len <= 0)
            return -2;

        // calcula o tamanho do read_len em dígitos, para obter o offset do buffer de resposta da UART
        // não queremos o +HTTPREAD: %d\r\n
        res_len = floor(log10(read_len)) + 1 + 20;
        memcpy(buffer, this->at_response + res_len, read_len);
    }
    return read_len;
}

void A7672SA::http_read_file(const char *filename, uint32_t timeout)
{
    char cmd[100];
    sprintf(cmd, "AT+HTTPREADFILE=\"%s\"" GSM_NL, filename);
    this->sendCommand("HTTP_REQUEST", cmd);
    this->wait_response(timeout);
}

void A7672SA::http_save_response(bool https)
{
    if (https)
        this->sendCommand("FS", "AT+FSCOPY=C:/https_body.dat,http_res.dat" GSM_NL); // C:/
    else
        this->sendCommand("FS", "AT+FSCOPY=C:/http_body.dat,http_res.dat" GSM_NL); // C:/
    this->wait_response();
}

bool A7672SA::http_term(uint32_t timeout)
{
    this->sendCommand("HTTP_TERM", "AT+HTTPTERM" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::fs_open(const char *filename, uint16_t mode, uint32_t timeout)
{
    char cmd[100];
    sprintf(cmd, "AT+FSOPEN=C:/%s,%d" GSM_NL, filename, mode);
    this->sendCommand("FS_OPEN", cmd);
    return this->wait_response(timeout);
}

bool A7672SA::fs_close(uint32_t timeout)
{
    this->sendCommand("FS_CLOSE", "AT+FSCLOSE=1" GSM_NL);
    return this->wait_response(timeout);
}

bool A7672SA::fs_delete(const char *filename, uint32_t timeout)
{
    char cmd[100];
    sprintf(cmd, "AT+FSDEL=%s" GSM_NL, filename);
    this->sendCommand("FS_DELETE", cmd);
    return this->wait_response(timeout);
}

uint32_t A7672SA::fs_size(const char *filename, uint32_t timeout)
{
    char cmd[100];
    sprintf(cmd, "AT+FSATTRI=C:/%s" GSM_NL, filename);
    this->sendCommand("FS_LENGTH", cmd);
    if (this->wait_response(timeout))
    {
        // example: +FSATTRI: 8604
        String data_string = "";
        String length = "";

        for (int j = 0; j < strlen(this->at_response); j++)
        {
            data_string += this->at_response[j];
        }

        data_string.trim();

        length = data_string.substring(data_string.indexOf(":") + 1, data_string.indexOf("\n") - 1);

        return length.toInt();
    }
    return 0;
}

void A7672SA::fs_list_files(uint32_t timeout)
{
    this->sendCommand("FS_LIST", "AT+FSLS" GSM_NL);
    this->wait_response(timeout);
}