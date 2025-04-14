#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>

constexpr char WIFI_SSID[] = "Pnt";
constexpr char WIFI_PASSWORD[] = "123456789";
constexpr char TOKEN[] = "nERqZH8zcAwlBBaXfkrH";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

volatile bool sensorStatus = false;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;


constexpr std::array<const char *, 4U> SHARED_ATTRIBUTES_LIST = { "fw_url", "fw_version", "fw_title"};

String firmwareUrl = ""; // lưu link url nhận từ CoreIoT
String lastFirmwareUrl = ""; // Biến lưu URL firmware đã xử lý trong phiên chạy hiện tại

// Callback khi nhận shared attribute
void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
        const char* key = it->key().c_str();

        if (strcmp(key, "fw_url") == 0) {
            firmwareUrl = it->value().as<const char*>();
            Serial.println("Received OTA fw_url: " + firmwareUrl);
        }
        
        if (strcmp(key, "fw_version") == 0) {
            Serial.print("Received OTA fw_version: ");
            Serial.println(it->value().as<const char*>());    
        }
        
        if (strcmp(key, "fw_title") == 0) {
            Serial.print("Received OTA fw_title: ");
            Serial.println(it->value().as<const char*>());
        }
    }
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

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
                tb.Shared_Attributes_Subscribe(attributes_callback);
                tb.Shared_Attributes_Request(attribute_shared_request_callback);
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

        if (!isnan(temperature) && !isnan(humidity)) {
            Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
            tb.sendTelemetryData("temperature", temperature);
            tb.sendTelemetryData("humidity", humidity);
        } else {
            Serial.println("Failed to read from DHT sensor!");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void TaskThingsBoardLoop(void *pvParameters) {
    while (1) {
        tb.loop();
        mqttClient.loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void TaskOTAUpdate(void *pvParameters) {
    while (1) {
        if (firmwareUrl != "" && firmwareUrl != lastFirmwareUrl) { 
            Serial.println("Starting OTA Update from URL: " + firmwareUrl);

            HTTPClient http;
            http.begin(firmwareUrl);
            int httpCode = http.GET();

            if (httpCode == HTTP_CODE_OK) {
                int contentLength = http.getSize();
                WiFiClient *stream = http.getStreamPtr();

                if (contentLength > 0) {
                    bool canBegin = Update.begin(contentLength);
                    if (canBegin) {
                        size_t written = Update.writeStream(*stream);
                        if (written == contentLength) {
                            Serial.println("Firmware written successfully.");
                        } else {
                            Serial.printf("Written only %d/%d bytes.\n", (int)written, contentLength);
                        }

                        if (Update.end()) {
                            if (Update.isFinished()) {
                                Serial.println("OTA Update successful. Rebooting...");
                                lastFirmwareUrl = firmwareUrl;
                                http.end();
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                ESP.restart();
                            } else {
                                Serial.println("OTA Update not finished properly.");
                            }
                        } else {
                            Serial.printf("Update failed: %s\n", Update.errorString());
                        }
                    } else {
                        Serial.println("Not enough space to begin OTA");
                    }
                } else {
                    Serial.println("Content length is not correct");
                }
            } else {
                Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
            }
            http.end();
            lastFirmwareUrl = firmwareUrl;
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}


void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);

    InitWiFi();
    Wire.begin(SDA_PIN, SCL_PIN);
    dht20.begin();

    xTaskCreate(TaskWiFiReconnect, "WiFiReconnect", 4096, NULL, 1, NULL);
    xTaskCreate(TaskCoreIOTConnect, "CoreIOTConnect", 4096, NULL, 1, NULL);
    xTaskCreate(TaskSendTelemetry, "SendTelemetry", 4096, NULL, 1, NULL);
    xTaskCreate(TaskThingsBoardLoop, "ThingsBoardLoop", 4096, NULL, 1, NULL);
    xTaskCreate(TaskOTAUpdate, "OTAUpdate", 8192, NULL, 1, NULL);  
}

void loop() {
    
}
