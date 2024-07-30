#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h> 
#include <env.h>

extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"  
}
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ClosedCube_HDC1080.h>

AsyncMqttClient mqttClient;
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

ClosedCube_HDC1080 hdc1080;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t mqttReconnectTimer;
unsigned long lastMsg = 0;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.print(F("MQTT disconnect reason: "));

    if (WiFi.isConnected()) {
      xTimerStart(mqttReconnectTimer, 0);
    }

    switch (reason) {
    case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
        Serial.println(F("TCP_DISCONNECTED"));
        break;
    case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
        Serial.println(F("MQTT_SERVER_UNAVAILABLE"));
        break;
    case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
        Serial.println(F("MQTT_UNACCEPTABLE_PROTOCOL_VERSION"));
        break;
    case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
        Serial.println(F("TLS_BAD_FINGERPRINT"));
        break;
    case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
        Serial.println(F("MQTT_IDENTIFIER_REJECTED"));
        break;
    case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
        Serial.println(F("MQTT_MALFORMED_CREDENTIALS"));
        break;
    case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
        Serial.println(F("MQTT_NOT_AUTHORIZED"));
        break;
    case AsyncMqttClientDisconnectReason::TCP_FORCE_DISCONNECTED:
        Serial.println(F("TCP_FORCE_DISCONNECTED"));
        break;
    default:
        Serial.printf_P(PSTR("unknown %d\n"), reason);
    }
}

void setup() {
  Serial.begin(115200);

  while (!bme.begin(0x76)) {
    Serial.println("bme not find");
    delay(1000); 
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  hdc1080.begin(0x40);

  String clientId = "sensors";

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setClientId(clientId.c_str());
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  mqttClient.setKeepAlive(10000);

  connectToWifi();
}

void loop() {
  if (mqttClient.connected()) {

    unsigned long now = millis();
    
    if (now - lastMsg > 10000) {

      sensors_event_t temp_event, pressure_event, humidity_event;
      bme_temp->getEvent(&temp_event);
      bme_pressure->getEvent(&pressure_event);
      bme_humidity->getEvent(&humidity_event);
      delay(500);

      lastMsg = now;

      float temperature = temp_event.temperature;
      if (temperature >= -35 && temperature <= 50) {
          mqttClient.publish("outside/bme280/temperature", 1, false, String(temperature).c_str());
      }

      delay(500);

      float humidity = humidity_event.relative_humidity;
      if (humidity >= 1 && humidity <= 100) {
          mqttClient.publish("outside/bme280/humidity", 1, false, String(humidity).c_str());
      }

      delay(500);

      float pressure = pressure_event.pressure * 0.750064;
      if (pressure >= 600 && pressure <= 800) {
          mqttClient.publish("outside/bme280/pressure", 1, false, String(pressure).c_str());
      }

      delay(500);

      float temperatureHDC = hdc1080.readTemperature();
      if (temperatureHDC >= -35 && temperatureHDC <= 50) {
          mqttClient.publish("outside/hdc1080/temperature", 1, false, String(temperatureHDC).c_str());
      }

      delay(500);

      float humidityHDC = hdc1080.readHumidity();
      if (humidityHDC >= 1 && humidityHDC <= 100) {
          mqttClient.publish("outside/hdc1080/humidity", 1, false, String(humidityHDC).c_str());
      }

      delay(1000);
      mqttClient.disconnect(true);
    }
  } 
}
