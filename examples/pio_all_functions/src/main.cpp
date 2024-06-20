#include "MQTT_A7672SA.h"
#include "cert.h"
#include "Update.h"
#include "FS.h"
#include "SPIFFS.h"

#define TX_PIN 17
#define RX_PIN 16
#define EN_PIN 27

A7672SA modem(GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_27);

// #define SSL
// #define APN "virtueyes.com.br"
// #define APN "inlog.claro.com.br"
#define APN "nbiot.gsim"
// #define APN "fulltime.com.br"

void updateFromFS(void *vParameters)
{
    // fazer parsing do header http para pegar o tamanho do arquivo
    int file_size = 1364512;
    // fazer fine tuning do chunk size
    int chunk_size = 9000;
    uint8_t buffer[chunk_size];

    modem.sendCommand("FS", "AT+FSLS" GSM_NL);
    modem.sendCommand("FS", "AT+FSOPEN=hook.dat" GSM_NL);
    // modem.sendCommand("FS", "AT+FSATTRI=hook.dat" GSM_NL);
    // modem.sendCommand("FS", "AT+FSREAD=1,100" GSM_NL);
    modem.wait_response(2000);

    // vTaskDelete(NULL);
    // return;

    // é necessário interromper a rx_task aqui, para que ela não consuma bytes da uart
    xSemaphoreTake(modem.uart_guard, portMAX_DELAY);

    int total_chunks = ceil(file_size / chunk_size);
    ESP_LOGD("total chunks", "%d", total_chunks);

    if (Update.begin(file_size))
    {
        for (int i = 0; i <= total_chunks; i++)
        {
            int ret = modem.fs_read_response(chunk_size, buffer);
            ESP_LOGD("UPDATE", "Reading chunk %d, size: %d", i, ret);

            if (ret < 0)
            {
                ESP_LOGE("UPDATE", "Error reading from modem %d", ret);
                modem.sendCommand("FS", "AT+FSCLOSE=1" GSM_NL);
                vTaskDelete(NULL);
                return;
            }

            if (int ret2 = Update.write(buffer, ret) != ret)
            {
                ESP_LOGE("UPDATE", "Error writing to flash %d", ret2);
                modem.sendCommand("FS", "AT+FSCLOSE=1" GSM_NL);
                vTaskDelete(NULL);
                return;
            }
        }

        modem.sendCommand("FS", "AT+FSCLOSE=1" GSM_NL);

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
}

void updateFromHTTP(void *vParameteres)
{
    uint32_t len = modem.http_request("https://api.hookalarme.com.br:444/device/image/", HTTP_METHOD::GET, true, "ca.pem", "Authorization: Basic SEtGQURBMzc6dW9IMEZhZXlJWDBIMHdKaHFQNFdCVW1WYUk4VDdzdg==", 78);
    // bool len = modem.http_request_file("https://api.hookalarme.com.br/device/image/", HTTP_METHOD::GET, "hook.txt", true, "ca.pem", "Authorization: Basic SEtGQURBMzc6dW9IMEZhZXlJWDBIMHdKaHFQNFdCVW1WYUk4VDdzdg==", 78);

    // fazer fine tuning do chunk size
    // int file_size = modem.http
    int chunk_size = 9000;
    uint8_t buffer[chunk_size];

    // modem.sendCommand("FS", "AT+FSOPEN=C:/https_body.dat" GSM_NL);
    // modem.wait_response(2000);

    // // é necessário interromper a rx_task aqui, para que ela não consuma bytes da uart
    // xSemaphoreTake(modem.uart_guard, portMAX_DELAY);

    // int total_chunks = ceil(file_size / chunk_size);
    // ESP_LOGD("total chunks", "%d", total_chunks);

    // if (Update.begin(file_size))
    // {
    //     for (int i = 0; i <= total_chunks; i++)
    //     {
    int ret = modem.http_read_response(buffer, chunk_size);
    ESP_LOGD("UPDATE", "Reading chunk %d, size: %d", 0, ret);

    //         if (ret < 0)
    //         {
    //             ESP_LOGE("UPDATE", "Error reading from modem %d", ret);
    //             return;
    //         }

    //         if (int ret2 = Update.write(buffer, ret) != ret)
    //         {
    //             ESP_LOGE("UPDATE", "Error writing to flash %d", ret2);
    //             return;
    //         }
    //     }

    //     if (Update.end())
    //     {
    //         Serial.println("OTA finished!");
    //         if (Update.isFinished())
    //         {
    //             Serial.println("Restart ESP device!");
    //             ESP.restart();
    //         }
    //         else
    //         {

    //             Serial.println("OTA not finished");
    //         }
    //     }
    //     else
    //     {
    //         Serial.println("Error occured #: " + String(Update.getError()));
    //     }
    // }
    modem.http_term();
}

bool initialize_modem(bool cert_write)
{
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

void setup()
{
    modem.begin();
    Serial.begin(115200);

    while (!modem.is_ready())
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    // initialize_modem(true);

    vTaskDelay(30000 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(updateFromFS, "updateFromFS", 4096 * 4, NULL, configMAX_PRIORITIES, NULL, 0);
    // xTaskCreatePinnedToCore(updateFromHTTP, "updateFromHTTP", 4096 * 4, NULL, configMAX_PRIORITIES, NULL, 0);
    ESP_LOGI("SETUP", "END");
}

void loop()
{
    vTaskDelete(NULL);
}
