#include <Arduino.h>
#include <WiFi.h> 
#include <env.h>

extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"  
}
#include <AsyncMqttClient.h>
#include <Adafruit_SHTC3.h>

#define SDA_1 21
#define SCL_1 22

TwoWire I2C_one = TwoWire(0);

AsyncMqttClient mqttClient;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t mqttReconnectTimer;

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

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

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  I2C_one.begin(SDA_1, SCL_1, 100000);

  while (!shtc3.begin(&I2C_one)) { 
    Serial.println("shtc3 not find");
    delay(1000); 
  }

  String clientId = "sensors";
  for (int i = 0; i < 5; i++) {
    clientId += char(random(25) + 'a'); 
  }

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setClientId(clientId.c_str());
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

  connectToWifi();
}

void loop() {
  if (mqttClient.connected()) {

    unsigned long now = millis();
    
    if (now - lastMsg > 5000) {
      lastMsg = now;

      sensors_event_t shtc3_humidity, shtc3_temp;  
      shtc3.getEvent(&shtc3_humidity, &shtc3_temp);

      mqttClient.publish("outside/shtc3/temperature", 1, true, String(shtc3_temp.temperature).c_str());
      mqttClient.publish("outside/shtc3/humidity", 1, true, String(shtc3_humidity.relative_humidity).c_str());

      delay(1000);
      mqttClient.disconnect(true);
    }
  } 
}
