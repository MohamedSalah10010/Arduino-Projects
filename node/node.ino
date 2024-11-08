#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>  // Include the ArduinoJson library

const char* ssid = "S";         // Replace with your Wi-Fi SSID
const char* password = "123456788"; // Replace with your Wi-Fi Password

const char* serverName = "http://192.168.1.9:5000/data"; // Replace with your actual server's IP address

WiFiClient wifiClient;  // Create a WiFiClient object

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    http.begin(wifiClient, serverName);  // Updated method call with WiFiClient object
    http.addHeader("Content-Type", "application/json");  // Set content type to JSON

    float batteryVoltage = 53;
    //float batteryTemperature = random(200, 300) / 10.0; // Range: 20.0 to 30.0 °C
    int stateOfCharge = 96 ;
    float rangeAvailable = 62 ;
//    float shuntVoltage = random(0, 20) / 1000.0; // Range: 0.00 to 0.02 
     float current = random(100, 200) / 10.0; // Range: 1.00 to 2.00 A
    float velocity = random(400, 600) / 10.0; // Range: 40.0 to 60.0 km/h
    float distanceCovered = 3 ; // Range: 100.0 to 200.0 km
    int spo2 = random(95, 101); // Range: 95 to 100%
    int heartRate = random(60, 101); // Range: 60 to 100 bpm
    String timeStamp = "2024-08-12 10:30:00"; // Replace with actual time if available

    // Create a JSON object
    StaticJsonDocument<512> doc;
    doc["battery"]["voltage"] = batteryVoltage;
//    doc["battery"]["temperature"] = batteryTemperature;
    doc["battery"]["state_of_charge"] = stateOfCharge;
    doc["battery"]["range_available"] = rangeAvailable;
//    doc["current"]["shunt_voltage"] = shuntVoltage;
    doc["current"]["current"] = current;
    doc["motion"]["velocity"] = velocity;
    doc["motion"]["distance_covered"] = distanceCovered;
    doc["health_sensor"]["spo2"] = spo2;
    doc["health_sensor"]["heart_rate"] = heartRate;
    doc["time"] = timeStamp;

    // Convert JSON object to string
    String jsonString;
    serializeJson(doc, jsonString);

    // Send the JSON request
    int httpResponseCode = http.POST(jsonString);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();  // End the HTTP connection
  }

  delay(500); // Send data every 5 seconds
}