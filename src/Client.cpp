#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Adafruit_SHTC3.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
};
#include <env.h>

#define MQTT_HOST IPAddress(192, 168, 0, 202)
#define MQTT_PORT 1883

#define SDA_1 21
#define SCL_1 22

#define SDA_2 33
#define SCL_2 32


TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
Adafruit_AHTX0 aht;


AsyncMqttClient mqttClient;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println("");
  
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  I2Ctwo.begin(SDA_2, SCL_2, 100000);
  I2Cone.begin(SDA_1, SCL_1, 100000);
  shtc3.begin(&I2Ctwo);
  aht.begin(&I2Cone);
}

void loop() {
    if (mqttClient.connected()) {
      Serial.println("MQTT connected");
      sensors_event_t shtc3_humidity, shtc3_temp;  
      shtc3.getEvent(&shtc3_humidity, &shtc3_temp);
      sensors_event_t aht_humidity, aht_temp;
      aht.getEvent(&aht_humidity, &aht_temp);
      
      float avg_temperature = (shtc3_temp.temperature + aht_temp.temperature) / 2.0;
      float avg_humidity = (shtc3_humidity.relative_humidity + aht_humidity.relative_humidity) / 2.0;
      mqttClient.publish("outside/both/temperature", 1, true, String(avg_temperature).c_str());
      mqttClient.publish("outside/both/humidity", 1, true, String(avg_humidity).c_str());
      mqttClient.publish("outside/shtc3/temperature", 1, true, String(shtc3_temp.temperature).c_str());
      mqttClient.publish("outside/shtc3/humidity", 1, true, String(shtc3_humidity.relative_humidity).c_str());
      mqttClient.publish("outside/aht10/temperature", 1, true, String(aht_temp.temperature).c_str());
      mqttClient.publish("outside/aht10/humidity", 1, true, String(aht_humidity.relative_humidity).c_str());
      mqttClient.disconnect();
      
    } else {
      delay(5000);
      connectToMqtt();
      delay(5000);
    }
}
