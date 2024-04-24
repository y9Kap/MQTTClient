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
    default:
        Serial.printf_P(PSTR("unknown %d\n"), reason);
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  while (!bme.begin(0x76)) {
    Serial.println("bme not find");
    delay(1000); 
  }

  hdc1080.begin(0x40);

  String clientId = "sensors";
  for (int i = 0; i < 5; i++) {
    clientId += char(random(25) + 'a'); 
  }

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setClientId(clientId.c_str());
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

  connectToWifi();
}

void loop() {
  if (mqttClient.connected()) {

    unsigned long now = millis();
    
    if (now - lastMsg > 10000) {
      lastMsg = now;

      mqttClient.publish("outside/bme/temperature", 1, false, String(bme.readTemperature()).c_str());
      delay(200);
      mqttClient.publish("outside/bme/humidity", 1, false, String(bme.readHumidity()).c_str());
      delay(200);
      mqttClient.publish("outside/bme/pressure", 1, false, String(bme.readPressure()).c_str());
      delay(200);
      mqttClient.publish("outside/hdc1080/temperature", 1, false, String(hdc1080.readTemperature()).c_str());
      delay(200);
      mqttClient.publish("outside/hdc1080/humidity", 1, false, String(hdc1080.readHumidity()).c_str());
      delay(200);
      
      delay(1000);
      mqttClient.disconnect(true);

    }
  } 
}
