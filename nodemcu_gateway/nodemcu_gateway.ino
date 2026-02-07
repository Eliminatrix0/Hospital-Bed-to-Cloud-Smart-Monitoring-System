/*
  NodeMCU (ESP-12E) - XBee Receiver to ThingsBoard Gateway
  UPDATED VERSION - Matches STM32 Full Output
  Input Format: "DHT:24.0,Hum:30.0,BMP:984.7,DS:74.9,Stat:No Fever,HR:0,SpO2:0,Wt:0.00"
*/

#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>

// ================== USER CONFIGURATION ==================
const char* WIFI_SSID     = "Akash";
const char* WIFI_PASS     = "123456780";

const char* TB_SERVER     = "115.240.192.141";        // Your ThingsBoard server
const char* TB_TOKEN      = "5FYbMeTjDgmFR8AD9X2w";    // Device Access Token    qaCXHnjHl2nxqh1LhdNB
const int   TB_PORT       = 1883;

#define XBEE_RX_PIN D5   // XBee TX -> NodeMCU D5
#define XBEE_TX_PIN D6   // XBee RX -> NodeMCU D6
// =======================================================

WiFiClient espClient;
PubSubClient client(espClient);
SoftwareSerial xbeeSerial(XBEE_RX_PIN, XBEE_TX_PIN);

char jsonBuffer[512]; 

// ------------------- WiFi Setup -------------------
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 50) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi FAILED. Restarting...");
    delay(2000);
    ESP.restart();
  }
}

// ------------------- MQTT Reconnect -------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("NodeMCU_Gateway", TB_TOKEN, NULL)) {
      Serial.println(" Connected!");
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5s...");
      delay(5000);
    }
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  xbeeSerial.begin(9600);

  Serial.println("\n=== NodeMCU Gateway Started ===");
  setup_wifi();
  client.setServer(TB_SERVER, TB_PORT);
}

// ------------------- Main Loop -------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED) setup_wifi();
  if (!client.connected()) reconnect();
  client.loop();

  if (xbeeSerial.available()) {
    String payload = xbeeSerial.readStringUntil('\n');
    payload.trim();
    
    // Ignore empty lines or tiny garbage data
    if (payload.length() < 10) return; 

    Serial.print("Received: ");
    Serial.println(payload);

    // Variables for parsing
    float dht_temp = 0;
    float dht_hum = 0;     // Added
    float pressure = 0;
    float ds_temp_f = 0;
    char stat_str[20];     // Added for "Fever"/"No Fever"
    float hr = 0;
    float spo2 = 0;        // Added
    float weight = 0;

    // Convert String to Char Array for sscanf
    char line[128];
    strncpy(line, payload.c_str(), sizeof(line));

    // --- PARSING LOGIC UPDATED ---
    // Format: "DHT:24.0,Hum:30.0,BMP:984.7,DS:74.9,Stat:No Fever,HR:0,SpO2:0,Wt:0.00"
    // %[^,] means "read string until a comma is found" (handles spaces in "No Fever")
    int fields = sscanf(line, "DHT:%f,Hum:%f,BMP:%f,DS:%f,Stat:%[^,],HR:%f,SpO2:%f,Wt:%f", 
                        &dht_temp, &dht_hum, &pressure, &ds_temp_f, stat_str, &hr, &spo2, &weight);

    // We now expect 8 fields to match
    if (fields == 8) {
      Serial.println("Parsing SUCCESS!");

      // --- CALCULATE DERIVED VALUES ---
      // STM32 sends Fahrenheit, we calculate Celsius here
      float ds_temp_c = (ds_temp_f - 32.0) * 5.0 / 9.0;

      // --- BUILD JSON ---
      snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{"
        "\"dht_temperature\":%.1f,"
        "\"dht_humidity\":%.1f,"
        "\"pressure_hpa\":%.2f,"
        "\"body_temp_f\":%.1f,"
        "\"body_temp_c\":%.1f,"
        "\"fever\":\"%s\","
        "\"heart_rate\":%.0f,"
        "\"spo2\":%.0f," 
        "\"weight_kg\":%.2f"
        "}",
        dht_temp, dht_hum, pressure, ds_temp_f, ds_temp_c, stat_str, hr, spo2, weight);

      Serial.print("Sending JSON: ");
      Serial.println(jsonBuffer);

      if (client.publish("v1/devices/me/telemetry", jsonBuffer)) {
        Serial.println("-> Published OK");
      } else {
        Serial.println("-> Publish FAILED");
      }
    } else {
      Serial.print("Parsing FAILED. Matched fields: ");
      Serial.println(fields);
      Serial.println("Expected: 8. Check format matches exactly.");
    }
    Serial.println("-----------------------------");
  }
}
