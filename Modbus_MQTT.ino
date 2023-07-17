#include "DHT.h"
#include <WiFi.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqtt_Generic.h>

#include <ModbusIP_ESP8266.h>
#include <Modbus.h>

#define DHTPIN 4 // D4 of NodeMCU is GPIO2
#define DHTTYPE DHT11

#define WIFI_SSID "Fox_2.4G"
#define WIFI_PASSWORD "159753@dc"

#define MQTT_HOST "broker.hivemq.com"
#define MQTT_PORT 1883
//MQTT Topics
#define MQTT_PUB_TEMP "esp32/temperature"
#define MQTT_PUB_HUM  "esp32/humidity"

float temp;
float hum;

DHT dht(DHTPIN, DHTTYPE);
ModbusIP mb;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Modbus register addresses for temperature and humidity
const int TEMPERATURE_REGISTER_ADDRESS = 0;
const int HUMIDITY_REGISTER_ADDRESS = 1;

unsigned long previousMillis = 0;   
const long interval = 5000;  

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  /*while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Conectado à rede WiFi: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  connectToMqtt();*/
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %dn", event);
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

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  connectToWifi();

  mb.server();
  mb.addIreg(TEMPERATURE_REGISTER_ADDRESS);
  mb.addIreg(HUMIDITY_REGISTER_ADDRESS);

  dht.begin();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  //connectToWifi();
}

void loop() {

  unsigned long currentMillis = millis();
  mb.task();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    hum = dht.readHumidity();
    temp = dht.readTemperature();

    uint16_t temperature = dht.readTemperature();
    uint16_t humidity = dht.readHumidity();
 
    if (isnan(temp) || isnan(hum)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    
    // Publish an MQTT message on topic esp32/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f n", temp);

    // Publish an MQTT message on topic esp32/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f ", hum);

    mb.Ireg(TEMPERATURE_REGISTER_ADDRESS, temperature);
    mb.Ireg(HUMIDITY_REGISTER_ADDRESS, humidity);

    Serial.println("");
    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.print(" *C  |  ");
    Serial.print("Umidade: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  //uint16_t temperature = dht.readTemperature();
  //uint16_t humidity = dht.readHumidity();

  /*if (!isnan(temperature) && !isnan(humidity)) {
    mb.Ireg(TEMPERATURE_REGISTER_ADDRESS, temperature);
    mb.Ireg(HUMIDITY_REGISTER_ADDRESS, humidity);

    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.print(" *C  |  ");
    Serial.print("Umidade: ");
    Serial.print(humidity);
    Serial.println(" %");
  }*/

  //delay(1000);
}