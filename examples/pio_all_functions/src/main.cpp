#include "MQTT_A7672SA.h"
#include "cert.h"
#include "Update.h"
#include "FS.h"
#include "SPIFFS.h"

#define TX_PIN 17
#define RX_PIN 16
#define EN_PIN 27
bool fodase = false;

A7672SA modem(GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_27);

// #define SSL

void doUpdateTask(void *vParameters)
{
    // fazer parsing do header http para pegar o tamanho do arquivo
    int file_size = 1364512;
    // fazer fine tuning do chunk size
    int chunk_size = 9000;
    uint8_t buffer[chunk_size];

    modem.sendCommand("FS", "AT+FSOPEN=C:/https_body.dat" GSM_NL);
    modem.wait_response(2000);

    // é necessário interromper a rx_task aqui, para que ela não consuma bytes da uart
    xSemaphoreTake(modem.uart_guard, portMAX_DELAY);

    int total_chunks = ceil(file_size / chunk_size);
    ESP_LOGD("total chunks", "%d", total_chunks);

    if (Update.begin(file_size))
    {
        for (int i = 0; i <= total_chunks; i++)
        {
            int ret = modem.http_read_response(chunk_size, buffer);
            ESP_LOGD("UPDATE", "Reading chunk %d, size: %d", i, ret);

            if (ret < 0)
            {
                ESP_LOGE("UPDATE", "Error reading from modem %d", ret);
                return;
            }

            if (int ret2 = Update.write(buffer, ret) != ret)
            {
                ESP_LOGE("UPDATE", "Error writing to flash %d", ret2);
                return;
            }
        }

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

void setup()
{
    modem.begin();
    Serial.begin(115200);

    while (!modem.is_ready())
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(20000 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(doUpdateTask, "doUpdateTask", 4096 * 4, NULL, configMAX_PRIORITIES, NULL, 0);
    ESP_LOGI("SETUP", "END");
}

void loop()
{
    vTaskDelete(NULL);
}
