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
// DB에 있는 코드와 1:1로 맞추는 걸 추천
// 예: HO_101_101_SMOKE / HO_101_101_HEAT
const char* deviceIdSmoke = "1";
const char* deviceIdHeat  = "2";

String topicSmoke; // hdc/device/{deviceIdSmoke}/safety/data
String topicHeat;  // hdc/device/{deviceIdHeat}/safety/data

// ================= DS18B20 =================
#define DS_PIN 2
OneWire oneWire(DS_PIN);
DallasTemperature sensors(&oneWire);

// ================= Dust Sensor =================
// Sharp GP2Y 계열 가정
const int dust_sensor = A0;
const int sensor_led  = 12;

float dust_value = 0;
float dustDensityug = 0;

const int sampling  = 280;
const int waiting   = 40;
const int stop_time = 9680;

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
    //                                  safety(안전센서)(도메인-아파트 id-space id-ho id)로 조합한 계정
    if (mqtt.connect(clientId.c_str(), "safety1n1", "0000")) {
      // PUB-only (QoS0)

    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.println(mqtt.state());

      delay(1000);
    }
  }
}

// ================= Publish helpers =================
void publishJsonToTopic(const String& topic, const char* sensorType, float value, const char* unit) {
  // ts 없음 (서버가 수신시각 기록)
  String payload = String("{\"sensorType\":\"") + sensorType +
                   "\",\"value\":" + String(value, 2) +
                   ",\"unit\":\"" + unit + "\"}";
  mqtt.publish(topic.c_str(), payload.c_str()); // QoS0 / retain=false
}

void setup() {
  Serial.begin(9600);

  pinMode(sensor_led, OUTPUT);
  sensors.begin();

  connectWiFi();
  connectMQTT();

  topicSmoke = String("hdc/device/") + deviceIdSmoke + "/safety/data";
  topicHeat  = String("hdc/device/") + deviceIdHeat  + "/safety/data";
}

void loop() {
  // 연결 유지
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // ---- Dust 측정 ----
  digitalWrite(sensor_led, LOW);
  delayMicroseconds(sampling);

  dust_value = analogRead(dust_sensor);

  delayMicroseconds(waiting);
  digitalWrite(sensor_led, HIGH);
  delayMicroseconds(stop_time);

  // 변환식(네 식 유지)
  float voltage = dust_value * (5.0 / 1024.0); // 필요시 1024 조정
  dustDensityug = (0.17 * voltage - 0.1) * 1000.0;
  if (dustDensityug < 0) dustDensityug = 0;

  // ---- Temperature 측정 ----
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // ---- Serial ----
  Serial.print("Dust [ug/m3]: ");
  Serial.println(dustDensityug);
  Serial.print("Temp [C]: ");
  Serial.println(tempC);

  // ---- Publish ----
  unsigned long now = millis();
  if (now - lastPubAt >= PUB_INTERVAL) {
    lastPubAt = now;

    // Dust -> SMOKE 센서 디바이스로 발행
    publishJsonToTopic(topicSmoke, "SMOKE", dustDensityug, "ug/m3");

    // Temp -> HEAT 센서 디바이스로 발행
    publishJsonToTopic(topicHeat, "HEAT", tempC, "C");
  }
}
