#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

// Replace with your network credentials
const char* ssid = "iPhony";
const char* password = "A23No26dA202";

// Flask server URL
const char* serverUrl = "http://172.20.10.13:8000/led";

// GPIO pin where the LED is connected
const int ledPin = D6; // Adjust as per your setup

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Initialize LED to off

  Serial.begin(115200);
  delay(100);

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());
}

void loop() {
  Serial.println("\nLoop");
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Begin HTTP request with WiFi client and URL
    WiFiClient client;
    // client.setInsecure();
    if (http.begin(client, serverUrl)) {
      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println("Server Response: " + payload);

        // Check the LED state from the server's response
        if (payload.indexOf("\"led\":\"on\"") != -1) {
          digitalWrite(ledPin, HIGH); // Turn LED on
        } else if (payload.indexOf("\"led\":\"off\"") != -1) {
          digitalWrite(ledPin, LOW); // Turn LED off
        }
      } else {
        Serial.printf("HTTP GET failed: %d\n", httpCode);
      }
      http.end(); // End the HTTP request
    } else {
      Serial.println("Unable to connect to the server.");
    }
  } else {
    Serial.println("WiFi not connected. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }

  delay(1000); // Check server every seconds
}
