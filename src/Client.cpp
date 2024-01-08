#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Adafruit_SHTC3.h>
#include <Adafruit_BMP085.h>
#include <RCSwitch.h>
#include <ArduinoJson.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
};
#include <env.h>

#define MQTT_PORT 1883

#define SDA_1 21
#define SCL_1 22

#define SDA_2 33
#define SCL_2 32


TwoWire I2C_one = TwoWire(0);
TwoWire I2C_two = TwoWire(1);

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
Adafruit_AHTX0 aht10;
Adafruit_BMP085 bmp;

RCSwitch mySwitch = RCSwitch();

AsyncMqttClient mqttClient;
TimerHandle_t wifiReconnectTimer;

bool lightSwitchState = false;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  mqttClient.subscribe("home/light_switch", 1);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  const char* username = doc["username"];
  const char* password = doc["password"];
  if (strcmp(username, CONTROL_USERNAME) != 0 || strcmp(password, CONTROL_PASSWORD) != 0) {
      Serial.print("Invalid username or password: ");
      Serial.print("Username: ");
      Serial.print(username);
      Serial.print(", Password: ");
      Serial.println(password);
  } else {
      mySwitch.send("000000000001010100010001"); //switch light
      lightSwitchState = !lightSwitchState;
      mqttClient.publish("home/light_switch_state", 1, true, String(lightSwitchState).c_str());
  }
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

  I2C_one.begin(SDA_1, SCL_1, 100000);
  I2C_two.begin(SDA_2, SCL_2, 100000);

  mySwitch.enableTransmit(19);

  while (!shtc3.begin(&I2C_two)) { 
    Serial.println("shtc3 not find");
    delay(1000); 
  }

  while (!bmp.begin(BMP085_ULTRAHIGHRES, &I2C_one)) { 
    Serial.println("bmp not find");
    delay(1000); 
  }
  while (!aht10.begin(&I2C_one)) { 
    Serial.println("aht10 not find");
    delay(1000); 
  }
  
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
    if (mqttClient.connected()) {
      Serial.println("MQTT connected");

      sensors_event_t shtc3_humidity, shtc3_temp;  
      shtc3.getEvent(&shtc3_humidity, &shtc3_temp);
      sensors_event_t aht_humidity, aht_temp;
      aht10.getEvent(&aht_humidity, &aht_temp);

      float avg_temperature = (shtc3_temp.temperature + aht_temp.temperature) / 2.0;
      float avg_humidity = (shtc3_humidity.relative_humidity + aht_humidity.relative_humidity) / 2.0;
      float bmp_temperature = bmp.readTemperature();
      float bmp_pressure = bmp.readPressure();

      mqttClient.publish("outside/both/temperature", 1, true, String(avg_temperature).c_str());
      mqttClient.publish("outside/both/humidity", 1, true, String(avg_humidity).c_str());
      mqttClient.publish("outside/shtc3/temperature", 1, true, String(shtc3_temp.temperature).c_str());
      mqttClient.publish("outside/shtc3/humidity", 1, true, String(shtc3_humidity.relative_humidity).c_str());
      mqttClient.publish("outside/aht10/temperature", 1, true, String(aht_temp.temperature).c_str());
      mqttClient.publish("outside/aht10/humidity", 1, true, String(aht_humidity.relative_humidity).c_str());
      mqttClient.publish("outside/bmp180/temperature", 1, true, String(bmp_temperature).c_str());
      mqttClient.publish("outside/bmp180/pressure", 1, true, String(bmp_pressure / 133.322).c_str());

      delay(10000);
      
    } else {
      delay(5000);
      connectToMqtt();
      delay(10000);
    }
}
