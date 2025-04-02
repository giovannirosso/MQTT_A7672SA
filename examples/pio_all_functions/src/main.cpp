#include "MQTT_A7672SA.h"
#include "cert.h"
#include "Update.h"

#define LTE_PIO GPIO_NUM_27
#define LTE_DTR GPIO_NUM_22
#define LTE_TX GPIO_NUM_17
#define LTE_RX GPIO_NUM_16

// #define APN "virtueyes.com.br"
// #define APN "inlog.claro.com.br"
// #define APN "nbiot.gsim"
#define APN "fulltime.com.br"

A7672SA modem(LTE_TX, LTE_RX, LTE_PIO);

bool http_ota = false;

struct ConnectionInformation
{
    String ip_address = "0.0.0.0";
    String lte_imei = "0";
    String lte_iccid = "0";
    String lte_carrier = "NO SIM";
    int32_t lte_signal_strength = 0;
    uint64_t timestamp = 0;
};

ConnectionInformation connectionInformation;

bool connected = false;

void mqttStatusCallback(mqtt_status &status)
{
    ESP_LOGI("MQTT_STATUS", "%d", status);
    connected = status == A7672SA_MQTT_CONNECTED;
    ESP_LOGI("MQTT_CONNECTED -> ", "%d", connected);
}

void mqttCallback(mqtt_message &message)
{
    printf("CALLBACK TOPIC: %s\n", message.topic);
    // print message.payload uint8_t array
    printf("CALLBACK PAYLOAD: ");
    for (int i = 0; i < message.length; i++)
    {
        printf("%c", message.payload[i]);
    }
    printf("\n");

    byte payload[message.length + 1];
    memcpy(payload, message.payload, message.length);
    if (strcmp(message.topic, "teste/sub") == 0)
    {
        modem.mqtt_publish("teste/pub", payload, message.length, 0, 5000);
    }
}

bool initialize_modem(bool cert_write)
{
    modem.set_network_mode(network_mode::AUTOMATIC);
    std::vector<NetworkOperator> operators = modem.get_operator_list(180000);

    for (const auto &op : operators)
    {
        Serial.println("[LTE] Operator: " + String(op.short_name));
        Serial.println("[LTE] Long Name: " + String(op.long_name));
        Serial.println("[LTE] Numeric Code: " + String(op.numeric_code));
        Serial.println("[LTE] Access Tech: " + String(op.access_tech));
        Serial.println("[LTE] Status: " + String(op.status));
        Serial.println("[LTE] ------------------------");
    }

    // Verificar se a operadora "VIVO" está disponível
    bool vivoAvailable = false;
    for (const auto &op : operators)
    {
        if (strcmp(op.short_name, modem.get_provider_name().c_str()) == 0)
        {
            vivoAvailable = true;
            Serial.println("[LTE] " + modem.get_provider_name() + " operator found");
            modem.set_operator(op);
            break;
        }
    }

    modem.set_apn(APN);

    uint32_t start = millis();
    while (!modem.wait_network() && millis() - start < 100000U)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        Serial.println("[LTE] Waiting for network...");
    }

    if (millis() - start >= 100000U)
    {
        Serial.println("[LTE] Network not connected, restarting ...");
        modem.restart();
        return false;
    }

    if (modem.wait_network())
    {
        Serial.println("[LTE] Network connected");
        modem.set_ntp_server("pool.ntp.org", 0, 5000);

        if (cert_write)
        {
            if (modem.set_ca_cert(https_ca_cert, "ca.pem", sizeof(https_ca_cert)))
            {
                Serial.println("[LTE] CA Cert set");
            }
            else
            {
                Serial.println("[LTE] CA cert fail to set");
                return false;
            }
        }
        return true;
    }
}

void update_sim_info()
{
    if (connectionInformation.lte_imei == "0")
        connectionInformation.lte_imei = modem.get_imei();
    if (connectionInformation.lte_carrier == "NO SIM")
        connectionInformation.lte_carrier = modem.get_provider_name();
    if (connectionInformation.lte_iccid == "0")
        connectionInformation.lte_iccid = modem.get_iccid();
    if (connectionInformation.lte_signal_strength == 0 || connectionInformation.lte_signal_strength == 99)
        connectionInformation.lte_signal_strength = modem.signal_quality();
}

void updateFromFS()
{
    int chunk_size = 10200;
    uint8_t buffer[chunk_size];

    modem.wait_response();
    uint32_t file_size = modem.fs_size("http_res.dat");
    String etag = modem.http_response_etag();
    if (!modem.fs_open("http_res.dat"))
        return;

    // é necessário interromper a rx_task aqui, para que ela não consuma bytes da uart
    modem.RX_LOCK();

    int total_chunks = ceil(file_size / chunk_size);
    ESP_LOGD("total chunks", "%d", total_chunks);

    if (Update.begin(file_size))
    {
        for (int i = 0; i <= total_chunks; i++)
        {
            int ret = modem.fs_read(chunk_size, buffer);
            ESP_LOGD("UPDATE", "Reading chunk %d, size: %d", i, ret);

            if (ret < 0)
            {
                ESP_LOGE("UPDATE", "Error reading from modem %d", ret);
                modem.fs_close();
                modem.fs_delete("http_res.dat");
                return;
            }

            if (int ret2 = Update.write(buffer, ret) != ret)
            {
                ESP_LOGE("UPDATE", "Error writing to flash %d", ret2);
                modem.fs_close();
                modem.fs_delete("http_res.dat");
                return;
            }
        }

        modem.fs_close();

        if (Update.end())
        {
            Serial.println("OTA finished!");
            if (Update.isFinished())
            {
                Serial.println("Restart ESP device!");
                modem.fs_delete("http_res.dat");
                ESP.restart();
            }
            else
            {
                Serial.println("OTA not finished");
                modem.fs_close();
                modem.fs_delete("http_res.dat");
            }
        }
        else
        {
            Serial.println("Error occured #: " + String(Update.getError()));
            modem.fs_close();
            modem.fs_delete("http_res.dat");
        }
    }

    modem.RX_UNLOCK();
}

void updateFromHTTP(void *vParameteres)
{
    while (!http_ota)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    uint32_t len = modem.http_request("https://api.hookalarme.com.br:444/device/image/", HTTP_METHOD::GET, true, true, "ca.pem", "Authorization: Basic SEtGQURBMzc6dW9IMEZhZXlJWDBIMHdKaHFQNFdCVW1WYUk4VDdzdg==", 78, 120);
    // bool len = modem.http_request_file("https://api.hookalarme.com.br/device/image/", HTTP_METHOD::GET, "hook.txt", true, "ca.pem", "Authorization: Basic SEtGQURBMzc6dW9IMEZhZXlJWDBIMHdKaHFQNFdCVW1WYUk4VDdzdg==", 78);

    // fazer fine tuning do chunk size
    // int file_size = modem.http
    int chunk_size = 10000;
    uint8_t buffer[chunk_size];

    // é necessário interromper a rx_task aqui, para que ela não consuma bytes da uart
    modem.RX_LOCK();

    int total_chunks = ceil(len / chunk_size);
    ESP_LOGD("total chunks", "%d", total_chunks);

    if (Update.begin(len))
    {
        for (int i = 0; i <= total_chunks; i++)
        {
            int ret = modem.http_read_response(buffer, chunk_size);
            ESP_LOGD("UPDATE", "Reading chunk %d, size: %d", i, ret);

            if (ret < 0)
            {
                ESP_LOGE("UPDATE", "Error reading from modem %d", ret);
                modem.sendCommand("FS", "AT+HTTPTERM" GSM_NL);
                vTaskDelete(NULL);
                return;
            }

            if (int ret2 = Update.write(buffer, ret) != ret)
            {
                ESP_LOGE("UPDATE", "Error writing to flash %d", ret2);
                modem.sendCommand("FS", "AT+HTTPTERM" GSM_NL);
                vTaskDelete(NULL);
                return;
            }
        }

        modem.http_term();

        if (Update.end())
        {
            Serial.println("OTA finished!");
            if (Update.isFinished())
            {
                Serial.println("Restart ESP device!");
                ESP.restart();
            }
            else
            {

                Serial.println("OTA not finished");
            }
        }
        else
        {
            Serial.println("Error occured #: " + String(Update.getError()));
        }
    }

    modem.RX_UNLOCK();
}

void setup()
{
    Serial.begin(115200);

    pinMode(LTE_PIO, OUTPUT);
    digitalWrite(LTE_PIO, LOW);

    pinMode(LTE_DTR, OUTPUT);
    digitalWrite(LTE_DTR, LOW);

    modem.on_message_callback(mqttCallback);
    modem.on_mqtt_status(mqttStatusCallback);

    modem.begin();

    // xTaskCreatePinnedToCore(updateFromFS, "updateFromFS", 4096 * 4, NULL, configMAX_PRIORITIES, NULL, 0);
    // xTaskCreatePinnedToCore(updateFromHTTP, "updateFromHTTP", 4096 * 4, NULL, configMAX_PRIORITIES, NULL, 0);
    ESP_LOGI("SETUP", "END");
}

bool modem_init = false;
void loop()
{
    if (modem_init)
    {
        if (modem.is_ready())
        {
            Serial.println("[MODEM] Init");
            if (modem.sim_ready())
            {
                Serial.println("[LTE] SIM READY");
                if (initialize_modem(true))
                {
                    update_sim_info();
                    modem_init = false;
                    modem.mqtt_connect("test.mosquitto.org", 1883, "A7672SA");
                }
            }
            else
            {
                Serial.println("[LTE] SIM NOT READY");
                modem_init = true;
            }
        }
    }

    if (Serial.available() > 0)
    {
        String income = Serial.readString();

        if (income == "init")
        {
            modem_init = true;
            Serial.println("Modem Init");
        }
        else if (income == "otafs")
        {
            Serial.println("ota");
            uint32_t status_code = modem.http_request("https://api.hookalarme.com.br:444/device/image/", HTTP_METHOD::GET, true, true, "ca.pem", "Authorization: Basic SEtGQURBMzc6dW9IMEZhZXlJWDBIMHdKaHFQNFdCVW1WYUk4VDdzdg==", 78, 120);
            Serial.println("Status code: " + String(status_code));
            String etag = modem.http_response_etag();
            Serial.println("Etag: " + etag);
            modem.http_term();
            if (status_code == 200)
                updateFromFS();
        }
        else if (income == "otahttp")
        {
            http_ota = true;
        }
        else
        {
            Serial.println("snd->" + income);
            income = income + "\r\n";
            modem.sendCommand("AT-LOOP", income.c_str());
        }
    }
}
