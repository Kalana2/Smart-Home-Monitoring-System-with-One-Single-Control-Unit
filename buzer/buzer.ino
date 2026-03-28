/*
 * Smart Home Monitoring System
 * Buzzer Alert Node - ESP32
 * Subscribes to home/watertank/level, beeps when water is LOW
 */

#include <WiFi.h>
#include <PubSubClient.h>

// Configuration
const char* ssid        = "K";
const char* password    = "12345678";
const char* mqtt_server = "test.mosquitto.org";
const int   mqtt_port   = 1883;

// MQTT Topics
const char* topic_water  = "home/watertank/level";
const char* topic_buzzer = "home/watertank/buzzer";
const char* topic_dist   = "home/watertank/distance";

// Pin Definitions
#define BUZZER_PIN  2
#define STATUS_LED  4

// Threshold
// Buzzer triggers when reported distance >= this value (water too low)
#define LOW_THRESHOLD_CM  20.0

// Timing
const long RECONNECT_INTERVAL = 5000;

// Global Objects
WiFiClient   espClient;
PubSubClient client(espClient);

unsigned long lastReconnect = 0;

// Function Prototypes
void    setupWiFi();
void    alertBeep();
void    onMessage(char* topic, byte* payload, unsigned int length);
boolean reconnectMQTT();

// WiFi
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
}

// Buzzer
void alertBeep() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

// MQTT Callback
void onMessage(char* topic, byte* payload, unsigned int length) {
  char buf[32];
  unsigned int len = min(length, (unsigned int)(sizeof(buf) - 1));
  memcpy(buf, payload, len);
  buf[len] = '\0';

  float dist = atof(buf);
  Serial.printf("[%s]  %.2f cm\n", topic, dist);

  if (dist <= 0) return;

  if (client.connected()) client.publish(topic_dist, buf);

  if (dist >= LOW_THRESHOLD_CM) {
    Serial.println("[ALERT] Water LOW -> buzzer ON");
    digitalWrite(STATUS_LED, HIGH);
    alertBeep();
    if (client.connected()) client.publish(topic_buzzer, "LOW");
  } else {
    Serial.println("[OK] Water OK -> buzzer OFF");
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    if (client.connected()) client.publish(topic_buzzer, "OK");
  }
}

// MQTT
boolean reconnectMQTT() {
  Serial.print("Connecting to MQTT... ");
  if (client.connect("ESP32_BUZZER_NODE")) {
    Serial.println("connected!");
    client.subscribe(topic_water);
    Serial.println("Subscribed: " + String(topic_water));
    return true;
  }
  Serial.print("failed, rc=");
  Serial.println(client.state());
  return false;
}

// Setup
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);

  setupWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(onMessage);

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
}
