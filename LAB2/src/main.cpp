#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
//#define DHT_PIN 18

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
// #include <DHT.h>

constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";

constexpr char TOKEN[] = "D6Z43EoQK2hdK1CkcdMm";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
//DHT dht(DHT_PIN, DHT11);

void InitWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
}

void TaskWiFiReconnect(void *pvParameters) {
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();
            InitWiFi();
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskCoreIOTConnect(void *pvParameters) {
    while (1) {
        if (!tb.connected()) {
            Serial.println("Connecting to CoreIOT...");
            if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
                Serial.println("Connected to CoreIOT!");
            } else {
                Serial.println("Failed to connect to CoreIOT");
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskSendTelemetry(void *pvParameters) {
    while (1) {
        dht20.read();
        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();

        // float temperature = dht.readTemperature();
        // float humidity = dht.readHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
            tb.sendTelemetryData("temperature", temperature);
            tb.sendTelemetryData("humidity", humidity);
        } 
        else {
            Serial.println("Failed to read from DHT sensor!");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskThingsBoardLoop(void *pvParameters) {
    while (1) {
        tb.loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);

    InitWiFi();
    Wire.begin(SDA_PIN, SCL_PIN);
    dht20.begin();

    // dht.begin();

    xTaskCreate(TaskWiFiReconnect, "WiFiReconnect", 4096, NULL, 1, NULL);
    xTaskCreate(TaskCoreIOTConnect, "CoreIOTConnect", 4096, NULL, 1, NULL);
    xTaskCreate(TaskSendTelemetry, "SendTelemetry", 4096, NULL, 1, NULL);
    xTaskCreate(TaskThingsBoardLoop, "ThingsBoardLoop", 4096, NULL, 1, NULL);
}

void loop() {

}



