/*
 * Smart Home Monitoring System
 * Water Tank Level Monitor - ESP32
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include <ArduinoJson.h>

// Configuration 
const char* ssid          = "K";
const char* wifi_password = "12345678";

const char* mqtt_server = "test.mosquitto.org";
const int   mqtt_port   = 1883;
const char* mqtt_user   = "";
const char* mqtt_pass   = "";

const char* topic_water_level   = "home/watertank/level";
const char* topic_water_status  = "home/watertank/status";
const char* topic_system_status = "home/system/status";

const char* node_id       = "WATER_TANK_01";
const char* node_location = "Main Water Tank";

#define TRIGGER_PIN  14
#define ECHO_PIN     12
#define MAX_DISTANCE 300
#define POWER_LED    16
#define STATUS_LED   2

// Tank Parameters (1 m tank) 
const float TANK_HEIGHT    = 100.0;
const float EMPTY_DISTANCE =  90.0;
const float FULL_DISTANCE  =  10.0;

// Timing 
const long RECONNECT_INTERVAL = 5000;

const float LVL_MID  = 75.0;   // above this → 5 s interval
const float LVL_HIGH = 90.0;   // above this → 1 s interval
const long  INT_LOW  = 7000;   // <75%  → 7 s
const long  INT_MID  = 5000;   // 75–90% → 5 s
const long  INT_HIGH = 1000;   // >90%  → 1 s

// IIR Filter Parameters 
const float IIR_ALPHA   = 0.4f; // 0.0 = max smooth, 1.0 = no filter
float       iirFiltered = -1.0f;

// Global Objects 
NewPing      sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
WiFiClient   espClient;
PubSubClient client(espClient);

unsigned long lastSensorRead = 0;
unsigned long lastReconnect  = 0;
long          sensorInterval = INT_LOW;

float   iirFilter(float sample);
long    getSensorInterval(float levelPct);
void    setupWiFi();
boolean reconnectMQTT();
void    readAndPublish();

// IIR Filter 
// First-order exponential moving average: y[n] = α·x[n] + (1-α)·y[n-1]
float iirFilter(float sample) {
  if (iirFiltered < 0.0f) iirFiltered = sample;
  else iirFiltered = IIR_ALPHA * sample + (1.0f - IIR_ALPHA) * iirFiltered;
  return iirFiltered;
}

// Dynamic Poll Interval 
long getSensorInterval(float levelPct) {
  if      (levelPct > LVL_HIGH) return INT_HIGH;
  else if (levelPct > LVL_MID)  return INT_MID;
  else                          return INT_LOW;
}

void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
  digitalWrite(STATUS_LED, HIGH);
}

// MQTT 
boolean reconnectMQTT() {
  Serial.print("Connecting to MQTT...");
  String clientId = String(node_id) + "_" + String(random(0xffff), HEX);
  if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println(" connected!");
    client.publish(topic_system_status,
                   ("{\"node\":\"" + String(node_id) + "\",\"status\":\"online\"}").c_str(),
                   true);
  } else {
    Serial.print(" failed, rc=");
    Serial.println(client.state());
  }
  return client.connected();
}

// Sensor Read & Publish 
void readAndPublish() {
  unsigned int rawCm = sonar.ping_cm();
  Serial.print("[SENSOR] Raw: "); Serial.print(rawCm); Serial.println(" cm");

  if (rawCm == 0) {
    Serial.println("[ERROR] No echo - check wiring TRIG=GPIO14, ECHO=GPIO12");
    return;
  }

  float filteredCm = iirFilter((float)rawCm);
  Serial.print("[FILTER] IIR: "); Serial.print(filteredCm, 1); Serial.println(" cm");

  float distance = constrain(filteredCm, FULL_DISTANCE, EMPTY_DISTANCE);
  float levelCm  = TANK_HEIGHT - distance;
  float levelPct = ((EMPTY_DISTANCE - distance) / (EMPTY_DISTANCE - FULL_DISTANCE)) * 100.0;
  levelPct = constrain(levelPct, 0.0, 100.0);

  sensorInterval = getSensorInterval(levelPct);
  Serial.print("[INTERVAL] Next read in "); Serial.print(sensorInterval / 1000); Serial.println(" s");

  String status;
  if      (levelPct >= 80.0) status = "FULL";
  else if (levelPct >= 50.0) status = "NORMAL";
  else if (levelPct >= 20.0) status = "LOW";
  else                       status = "CRITICAL";

  JsonDocument doc;
  doc["node_id"]     = node_id;
  doc["timestamp"]   = millis();
  doc["level_cm"]    = round(levelCm  * 10) / 10.0;
  doc["percentage"]  = round(levelPct * 10) / 10.0;
  doc["distance_cm"] = round(distance * 10) / 10.0;
  doc["status"]      = status;

  char payload[256];
  serializeJson(doc, payload);

  String distPayload = "{\"node_id\":\"" + String(node_id)
                     + "\",\"raw_cm\":"    + String(rawCm)
                     + ",\"filtered_cm\":" + String(filteredCm, 1) + "}";
  client.publish("home/watertank/distance", distPayload.c_str());

  if (client.publish(topic_water_level, payload)) {
    Serial.print("Published: "); Serial.println(payload);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
    digitalWrite(STATUS_LED, HIGH);
  }

  client.publish(topic_water_status, status.c_str());

  if (status == "CRITICAL") {
    String alert = "{\"type\":\"WATER_CRITICAL\",\"message\":\"Water below 20%\","
                   "\"node\":\"" + String(node_id) + "\","
                   "\"level\":" + String(levelPct, 1) + "}";
    client.publish("home/alerts/water", alert.c_str());
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  pinMode(POWER_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(POWER_LED, HIGH);
  digitalWrite(STATUS_LED, LOW);

  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(60);
  reconnectMQTT();
}

// Loop  
void loop() {
  if (WiFi.status() != WL_CONNECTED) setupWiFi();

  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnect > RECONNECT_INTERVAL) {
      lastReconnect = now;
      reconnectMQTT();
    }
  } else {
    client.loop();
  }

  unsigned long now = millis();
  if (now - lastSensorRead >= (unsigned long)sensorInterval) {
    lastSensorRead = now;
    readAndPublish();
  }
}