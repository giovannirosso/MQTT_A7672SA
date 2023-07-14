#include "MQTT_A7672SA.h"
#include "cert.h"

A7672SA modem(GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_27);

#define APN "inlog.claro.com.br"

void mqttCallback(mqtt_message &message)
{
    printf("CALLBACK TOPIC: %s\n", message.topic);
    printf("CALLBACK DATA: %s\n", message.data);
}

void setup()
{
    modem.begin();

    modem.register_callback(mqttCallback);

start:

    while (!modem.is_ready())
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    bool sim = modem.sim_ready(5000);
    ESP_LOGI("SIM", "%d", sim);

    int signal;
    uint32_t start = millis();
    do
    {
        signal = modem.signal_quality(5000);
        ESP_LOGI("SIGNAL", "%d", signal);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    } while (signal == 99 && millis() - start < 10000);
    if (signal == 99)
    {
        modem.restart();
    }

    bool apn = modem.set_apn(APN, 5000);
    ESP_LOGI("APN", "%d", apn);
    start = millis();
    while (!modem.wait_network() && millis() - start < 10000)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    if (millis() - start >= 10000)
    {
        modem.restart();
    }

    bool ntp = modem.set_ntp_server("pool.ntp.org", 0, 5000);
    ESP_LOGI("NTP", "%d", ntp);

    String provider;
    do
    {
        provider = modem.get_provider_name(5000);
        printf("PROVIDER: %s\n", provider);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    } while (provider == "");

    String imei;
    do
    {
        imei = modem.get_imei(5000);
        printf("IMEI: %s\n", imei.c_str());
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    } while (imei == "");

    String ip;
    do
    {
        ip = modem.get_local_ip(5000);
        printf("IP: %s\n", ip);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    } while (ip == "");

    String time;
    do
    {
        time = modem.get_ntp_time(5000);
        printf("TIME: %s\n", time.c_str());
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    } while (time == "");

    bool mqtt = modem.mqtt_connect("test.mosquitto.org", 1883, "A7672SA");
    ESP_LOGI("MQTT_CONNECTED_NO_SSL -> ", "%d", mqtt);

    bool sub = modem.mqtt_subscribe("teste/sub", 0, 5000);
    ESP_LOGI("MQTT_SUBSCRIBE -> ", "%d", sub);

    bool pub = modem.mqtt_publish("teste/pub", "TESTE", 0, 5000);
    ESP_LOGI("MQTT_PUBLISH -> ", "%d", pub);

    vTaskDelay(30000 / portTICK_PERIOD_MS);

    bool dis = modem.mqtt_disconnect(5000);
    ESP_LOGI("MQTT_DISCONNECT -> ", "%d", dis);

    bool cert = modem.set_ca_cert(ca_cert, "ca.pem", sizeof(ca_cert), 5000);
    ESP_LOGI("CERT", "%d", cert);

    mqtt = modem.mqtt_connect("test.mosquitto.org", 8883, "A7672SA", NULL, NULL, true, "ca.pem");
    ESP_LOGI("MQTT_CONNECTED_SSL -> ", "%d", mqtt);

    sub = modem.mqtt_subscribe("teste/sub", 0, 5000);
    ESP_LOGI("MQTT_SUBSCRIBE -> ", "%d", sub);

    pub = modem.mqtt_publish("teste/pub", "TESTE", 0, 5000);
    ESP_LOGI("MQTT_PUBLISH -> ", "%d", pub);

    vTaskDelay(60000 / portTICK_PERIOD_MS);

    dis = modem.mqtt_disconnect(5000);
    ESP_LOGI("MQTT_DISCONNECT -> ", "%d", dis);

    printf("RESTARTING...\n");
    modem.restart();
    goto start;
}

void loop()
{
    yield();
}
