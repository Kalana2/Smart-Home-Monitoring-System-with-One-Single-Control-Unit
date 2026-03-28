/*
 * Smart Home Monitoring System
 * Environment Monitor Node - ESP32
 * DHT11 (Temp/Humidity) every 10s | LDR (Light) every 1 min, with IIR filtering
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// Configuration
const char* ssid          = "SSID";
const char* wifi_password = "Password";
const char* mqtt_server   = "test.mosquitto.org";
const int   mqtt_port     = 1883;

// MQTT Topics
const char* topic_temperature = "home/environment/temperature";
const char* topic_humidity    = "home/environment/humidity";
const char* topic_light       = "home/environment/light";
const char* topic_daynight    = "home/environment/daynight";
const char* topic_system      = "home/system/status";

// Pin Definitions
#define DHTPIN   4
#define DHTTYPE  DHT11
#define LDR_PIN  34

// Timing
const long ENV_INTERVAL       = 10000;  // temp/humidity every 10 s
const long LIGHT_INTERVAL     = 60000;  // light every 1 min
const long RECONNECT_INTERVAL =  5000;

// IIR Filter Parameters
const float ALPHA_TEMP  = 0.2f;
const float ALPHA_HUM   = 0.2f;
const float ALPHA_LIGHT = 0.3f;

float filtTemp  = -1.0f;  // -1 = uninitialised, seeded on first reading
float filtHum   = -1.0f;
float filtLight = -1.0f;

// Threshold
const float DAY_THRESHOLD = 20.0;  // light % below this -> NIGHT

// Global Objects
WiFiClient   espClient;
PubSubClient client(espClient);
DHT          dht(DHTPIN, DHTTYPE);

unsigned long lastEnvRead   = 0;
unsigned long lastLightRead = 0;
unsigned long lastReconnect = 0;

// Function Prototypes
float   iirFilter(float sample, float& state, float alpha);
void    setupWiFi();
boolean reconnectMQTT();
void    readTempHumidity();
void    readLight();

// IIR Filter
// Generic single-channel filter; state is seeded automatically on first call.
float iirFilter(float sample, float& state, float alpha) {
  if (state < 0.0f) state = sample;
  else              state = alpha * sample + (1.0f - alpha) * state;
  return state;
}

// WiFi
void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
}

// MQTT
boolean reconnectMQTT() {
  int attempts = 0;
  while (!client.connected() && attempts < 5) {
    Serial.print("Connecting to MQTT... ");
    if (client.connect("ESP32_ENV")) {
      client.publish(topic_system, "ESP32_ENV online", true);
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" - retrying in 2s");
      delay(2000);
    }
    attempts++;
  }
  return client.connected();
}

// Temp & Humidity Read (every 10 s)
void readTempHumidity() {
  float rawTemp = dht.readTemperature();
  float rawHum  = dht.readHumidity();

  Serial.print("[SENSOR] Raw  T="); Serial.print(rawTemp);
  Serial.print(" H="); Serial.println(rawHum);

  if (isnan(rawTemp) || isnan(rawHum)) {
    Serial.println("[ERROR] DHT read failed!");
    client.publish(topic_system, "DHT_ERROR", false);
    return;
  }

  float temperature = iirFilter(rawTemp, filtTemp, ALPHA_TEMP);
  float humidity    = iirFilter(rawHum,  filtHum,  ALPHA_HUM);

  Serial.print("[FILTER] IIR  T="); Serial.print(temperature, 1);
  Serial.print(" H="); Serial.println(humidity, 1);

  char tempStr[10], humStr[10];
  dtostrf(temperature, 5, 1, tempStr);
  dtostrf(humidity,    5, 1, humStr);

  client.publish(topic_temperature, tempStr);
  client.publish(topic_humidity,    humStr);
}

// Light Read (every 1 min)
void readLight() {
  int rawLight = analogRead(LDR_PIN);
  Serial.print("[SENSOR] Raw  L="); Serial.println(rawLight);

  float lightPct  = iirFilter((rawLight / 4095.0f) * 100.0f, filtLight, ALPHA_LIGHT);
  const char* dayNight = (lightPct < DAY_THRESHOLD) ? "NIGHT" : "DAY";

  Serial.print("[FILTER] IIR  L="); Serial.print(lightPct, 1); Serial.println("%");

  char lightStr[10];
  dtostrf(lightPct, 5, 1, lightStr);

  client.publish(topic_light,    lightStr);
  client.publish(topic_daynight, dayNight);

  Serial.print("Published: L="); Serial.print(lightStr);
  Serial.print("% State="); Serial.println(dayNight);
}

// Setup
void setup() {
  Serial.begin(115200);
  delay(100);

  dht.begin();
  setupWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(60);
  reconnectMQTT();

  lastEnvRead   = millis() - ENV_INTERVAL;    // trigger first DHT read immediately
  lastLightRead = millis() - LIGHT_INTERVAL;  // trigger first light read immediately
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

  if (millis() - lastEnvRead >= (unsigned long)ENV_INTERVAL) {
    lastEnvRead = millis();
    readTempHumidity();
  }

  if (millis() - lastLightRead >= (unsigned long)LIGHT_INTERVAL) {
    lastLightRead = millis();
    readLight();
  }
}
