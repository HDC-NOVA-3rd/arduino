#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiS3.h>
#include <PubSubClient.h>

// ================= WiFi / MQTT =================
const char* ssid = "iGrowth";
const char* pass = "new1234~!";

const char* mqttHost = "223.171.136.185";
const int   mqttPort = 1883;

WiFiClient net;
PubSubClient mqtt(net);

// ================= deviceId 분리 =================
const char* deviceIdGas  = "1";
const char* deviceIdHeat = "2";

String topicGas;   // hdc/device/{deviceIdGas}/safety/data
String topicHeat;  // hdc/device/{deviceIdHeat}/safety/data

// ================= DS18B20 =================
#define DS_PIN 13
OneWire oneWire(DS_PIN);
DallasTemperature sensors(&oneWire);

// ================= MQ-2 =================
// AO -> A0, DO -> D8
const int mq2AO = A0;
const int mq2DO = 8;

// UNO R4에서 analogRead는 보통 0~1023 스케일.
const float ADC_REF_VOLT = 5.0;
const float ADC_MAX = 1023.0;

// ================= Buzzer =================
const int BUZZER_PIN = 6;         // 부저 연결 핀 (D6)
const int BUZZER_FREQ = 2000;     // 2kHz 정도
const unsigned long BEEP_ON_MS  = 200;
const unsigned long BEEP_OFF_MS = 200;

bool buzzerOn = false;
unsigned long lastBeepToggleAt = 0;

// ================= Alarm Threshold =================
const float TEMP_ALARM_C = 70.0;
const int   GAS_ALARM_RAW = 500;

// ================= Publish 주기 =================
const unsigned long PUB_INTERVAL = 2000; // 2초
unsigned long lastPubAt = 0;

// ================= Connect =================
void connectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(1000);
  }
}

void connectMQTT() {
  mqtt.setServer(mqttHost, mqttPort);
  while (!mqtt.connected()) {
    String clientId = "uno-r4-";
    clientId += String(millis());

    if (mqtt.connect(clientId.c_str(), "server", "server1883")) {
      // PUB-only
    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }
}

// ================= Publish helpers =================
void publishJsonToTopic(const String& topic, const char* sensorType, float value, const char* unit) {
  String payload = String("{\"sensorType\":\"") + sensorType +
                   "\",\"value\":" + String(value, 2) +
                   ",\"unit\":\"" + unit + "\"}";
  mqtt.publish(topic.c_str(), payload.c_str());
}

// ================= Buzzer helpers (non-blocking) =================
void buzzerOff() {
  noTone(BUZZER_PIN);
  buzzerOn = false;
}

void buzzerTick(bool alarmActive, unsigned long now) {
  if (!alarmActive) {
    buzzerOff();
    return;
  }

  // alarmActive == true -> 비프 패턴 실행
  unsigned long interval = buzzerOn ? BEEP_ON_MS : BEEP_OFF_MS;
  if (now - lastBeepToggleAt >= interval) {
    lastBeepToggleAt = now;
    buzzerOn = !buzzerOn;

    if (buzzerOn) tone(BUZZER_PIN, BUZZER_FREQ);
    else          noTone(BUZZER_PIN);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(mq2DO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  sensors.begin();

  connectWiFi();
  connectMQTT();

  topicGas  = String("hdc/device/") + deviceIdGas  + "/safety/data";
  topicHeat = String("hdc/device/") + deviceIdHeat + "/safety/data";
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // ---- MQ-2 측정 ----
  int gasRaw = analogRead(mq2AO);  // 0~1023
  int gasDO  = digitalRead(mq2DO);

  float gasVolt = gasRaw * (ADC_REF_VOLT / ADC_MAX);

  // ---- Temperature 측정 ----
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // ---- Alarm 판단 ----
  bool alarmActive = (tempC >= TEMP_ALARM_C) || (gasRaw >= GAS_ALARM_RAW);

  // ---- Buzzer (non-blocking) ----
  unsigned long now = millis();
  buzzerTick(alarmActive, now);

  // ---- Serial ----
  Serial.print("MQ2 AO raw: ");
  Serial.print(gasRaw);
  Serial.print(" | volt: ");
  Serial.print(gasVolt, 3);
  Serial.print("V | DO: ");
  Serial.print(gasDO);
  Serial.print(" | ALARM: ");
  Serial.println(alarmActive ? "ON" : "OFF");

  Serial.print("Temp [C]: ");
  Serial.println(tempC);

  // ---- Publish ----
  if (now - lastPubAt >= PUB_INTERVAL) {
    lastPubAt = now;

    publishJsonToTopic(topicGas, "GAS", (float)gasRaw, "raw");
    publishJsonToTopic(topicGas, "GAS_DO", (float)gasDO, "bool");
    publishJsonToTopic(topicHeat, "HEAT", tempC, "C");
  }
}
