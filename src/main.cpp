#include <WiFi.h>
#include "DHTesp.h"
#include <PubSubClient.h> 
#include <NTPClient.h>

#define DHT_PIN 15
#define BUZZER 12

WiFiClient espClient;
PubSubClient mqttClient(espClient);

char tempAr[6];
DHTesp dhtSensor;

// Function declarations
void setupWifi();
void updateTemperature();
void setupMqtt();
void connectToBroker();
void receiveCallbackMyFunction(char* topic, byte* payload, unsigned int length);


void setup() {
  Serial.begin(115200);

  setupWifi();
  setupMqtt();
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

}

void loop() {
  if (!mqttClient.connected()) {
    connectToBroker();
  }

  mqttClient.loop(); // Don't forget this to maintain MQTT connection!

  updateTemperature();
  Serial.println(tempAr);

  mqttClient.publish("MATHISHA-ADMIN-TEMP", tempAr); // Optional publish
  delay(1000);  // Prevent flooding the broker
}



void buzzerOn(bool on){
  if (on){
    tone(BUZZER,256);

  }
  else{
    noTone(BUZZER);
  }
}

void setupMqtt() {
  mqttClient.setServer("test.mosquitto.org", 1883);
  mqttClient.setCallback(receiveCallbackMyFunction);
}

void receiveCallbackMyFunction(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char payloadCharAr[length];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    payloadCharAr[i] = (char)payload[i];
  }

  Serial.print("] ");

  if(strcmp(topic,"MATHISHA-ADMIN-MAIN-ON-OFF")==0){
    buzzerOn(payloadCharAr[0] == '1');
    }

  }



void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32-12345645454")) {
      Serial.println("connected");
      mqttClient.subscribe("MATHISHA-ADMIN-MAIN-ON-OFF");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void updateTemperature() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  String(data.temperature, 2).toCharArray(tempAr, 6);
}

void setupWifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println("Wokwi-GUEST");

  WiFi.begin("Wokwi-GUEST", "");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
