#include <WiFi.h>
#include "DHTesp.h"
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3c

#define BUZZER 5
#define LED_1 15
#define PB_CANCEL 25
#define PB_OK 32
#define PB_UP 33
#define PB_DOWN 35
#define DHT_PIN 12
#define LDR_PIN 34               
#define SERVO_PIN  14 

unsigned long samplingInterval = 5000;   // default ts = 5 s  
unsigned long sendingInterval  = 30000; // default tu = 30 s
unsigned long lastSampleTime = 0;  
unsigned long lastSendTime   = 0;  
uint32_t sumLight = 0;  
uint16_t sampleCount = 0;  
float thetaOffset=30;
float gammaFactor=0.75;
float Tmed=30;
float intensity=0.756;

char tempAr[6];

bool isScheduledON = false;
unsigned long scheduleOnTime;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
DHTesp dhtSensor;
Servo windowServo;

void setupWifi();
void updateTemperature();
void setupMqtt();
void connectToBroker();
void receiveCallbackMyFunction(char* topic, byte* payload, unsigned int length);
void buzzerOn(bool on);
void checkSchedule();
unsigned long getTime();

void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  setupWifi();
  setupMqtt();
  timeClient.begin();
  timeClient.setTimeOffset(19800); // GMT+5:30

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  windowServo.attach(SERVO_PIN);
  windowServo.write(thetaOffset);
}

void loop() {
  if(!mqttClient.connected()) connectToBroker();
  mqttClient.loop();

  updateTemperature();
  mqttClient.publish("MATHISHA-ADMIN-TEMP", tempAr);
  checkSchedule();

  unsigned long now = millis();
// 1) Sample every ts  
  if (now - lastSampleTime >= samplingInterval) {
    lastSampleTime = now;
    uint16_t raw = analogRead(LDR_PIN);
    sumLight += raw;                         
    sampleCount++;
  }

// 2) Send average every tu
  if (now - lastSendTime >= sendingInterval) {
    lastSendTime = now;
    float avg = (sampleCount > 0) ? float(sumLight)/sampleCount/4095.0f : 0.0f;
    intensity = 1.0f - avg;
    char buf[8];
    dtostrf(intensity, 1, 3, buf);
    mqttClient.publish("MATHISHA-light/data", buf);
    sumLight = 0;
    sampleCount = 0;

    // compute the window angle θ
    float T = dhtSensor.getTempAndHumidity().temperature;
    float ratio = float(samplingInterval) / float(sendingInterval);
    if (ratio <= 0) ratio = 1.0f;
    float theta = thetaOffset + (180.0f - thetaOffset) * intensity * gammaFactor * log(ratio) * (T / Tmed);
    theta = constrain(theta, 0.0f, 180.0f);
    windowServo.write(theta);
    mqttClient.publish("MATHISHA-theta/data", String(theta, 2).c_str()); 
  }

  delay(1000);
}

void setupMqtt() {
  mqttClient.setServer("broker.hivemq.com", 1883);
  mqttClient.setCallback(receiveCallbackMyFunction);
}

void receiveCallbackMyFunction(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char payloadCharAr[length + 1];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    payloadCharAr[i] = (char)payload[i];
  }
  payloadCharAr[length] = '\0';
  Serial.println();

  if (strcmp(topic, "MATHISHA-light/ts") == 0) {
    samplingInterval = atol(payloadCharAr) * 1000UL;
  }
  else if (strcmp(topic, "MATHISHA-light/tu") == 0) {
    sendingInterval = atol(payloadCharAr) * 1000UL;
  }
  else if (strcmp(topic, "MATHISHA-servo/offset") == 0) {
    thetaOffset = constrain(atof(payloadCharAr), 0.0f, 120.0f);
  }
  else if (strcmp(topic, "MATHISHA-servo/Gamma") == 0) {
    gammaFactor = constrain(atof(payloadCharAr), 0.0f, 1.0f);
  }
  else if (strcmp(topic, "MATHISHA-servo/Tmed") == 0) {
    Tmed = constrain(atof(payloadCharAr), 10.0f, 40.0f);
  }
  else if (strcmp(topic, "MATHISHA-ADMIN-MAIN-ON-OFF") == 0) {
    buzzerOn(payloadCharAr[0] == '1');
  }
  else if (strcmp(topic, "MATHISHA-ADMIN-SCH-ON") == 0) {
    if (payloadCharAr[0] == 'N') {
      isScheduledON = false;
    } else {
      isScheduledON = true;
      scheduleOnTime = atol(payloadCharAr);
    }
  }
}

void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32-Client")) {
      Serial.println("connected");
      mqttClient.subscribe("MATHISHA-ADMIN-MAIN-ON-OFF");
      mqttClient.subscribe("MATHISHA-ADMIN-SCH-ON");
      mqttClient.subscribe("MATHISHA-light/ts");
      mqttClient.subscribe("MATHISHA-light/tu");
      mqttClient.subscribe("MATHISHA-servo/offset");
      mqttClient.subscribe("MATHISHA-servo/gamma");
      mqttClient.subscribe("MATHISHA-servo/Tmed");
    } else {
      Serial.print("failed rc=");
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
  Serial.print("Connecting to Wokwi-GUEST\n");
  WiFi.begin("Wokwi-GUEST", "");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
}

void buzzerOn(bool on) {
  if (on) tone(BUZZER, 256);
  else noTone(BUZZER);
}

void checkSchedule() {
  if (isScheduledON && getTime() > scheduleOnTime) {
    buzzerOn(true);
    isScheduledON = false;
    mqttClient.publish("MATHISHA-ADMIN-MAIN-ON-OFF-ESP", "1");
    mqttClient.publish("MATHISHA-ADMIN-SCH-ESP-ON", "0");
    Serial.println("Scheduled ON Triggered");
  }
}

unsigned long getTime() {
  timeClient.update();
  return timeClient.getEpochTime();
}