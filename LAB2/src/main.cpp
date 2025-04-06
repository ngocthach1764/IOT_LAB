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

constexpr char WIFI_SSID[] = "Pnt";
constexpr char WIFI_PASSWORD[] = "123456789";

constexpr char TOKEN[] = "voZfZn8nPI67MxFzxLzS";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

volatile bool sensorStatus = false;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
//DHT dht(DHT_PIN, DHT11);

// Key của shared attribute
constexpr char SENSOR_STATUS_ATTR[] = "status";

// List of shared attributes
constexpr std::array<const char *, 1U> SHARED_ATTRIBUTES_LIST = {
    SENSOR_STATUS_ATTR
};

// Callback khi nhận shared attribute
void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), SENSOR_STATUS_ATTR) == 0) {
        // Cập nhật trạng thái cảm biến
        sensorStatus = it->value().as<bool>();
        Serial.printf("Sensor status changed: %d\n", sensorStatus);
    }
  }
}

// callback để nhận và lấy thuộc tính shared attributes
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
        if (sensorStatus) {
            dht20.read();
            float temperature = dht20.getTemperature();
            float humidity = dht20.getHumidity();
    
            // float temperature = dht.readTemperature();
            // float humidity = dht.readHumidity();

            if (!isnan(temperature) && !isnan(humidity)) {
                Serial.printf("Temp: %.2f°C, Humi: %.2f%%\n", temperature, humidity);
                tb.sendTelemetryData("temperature", temperature);
                tb.sendTelemetryData("humidity", humidity);
            } else {
                Serial.println("Failed to read from DHT20 sensor!");
            }
        } else {
            Serial.println("Sensor is OFF => Not sending data.");
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



