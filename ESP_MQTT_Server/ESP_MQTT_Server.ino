#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Replace with your network credentials
const char* ssid = "iPhony";
const char* password = "A23No26dA202";

// MQTT Broker settings
// const char* mqtt_server = "172.20.10.13";
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // Default MQTT port
const char* mqtt_topic_led = "led/control"; // Topic for controlling the LED

// GPIO pin where the LED is connected
const int ledPin = D6; // Adjust as per your setup

WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());
}

// Callback function to handle incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  // Check the LED control message
  if (String(topic) == mqtt_topic_led) {
    if (message == "ON") {
      digitalWrite(ledPin, HIGH); // Turn LED on
      Serial.println("LED ON");
    } else if (message == "OFF") {
      digitalWrite(ledPin, LOW); // Turn LED off
      Serial.println("LED OFF");
    }
  }
}

// Function to connect to the MQTT broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String client_id = "ESP8266Client-" + String(random(0xffff));
    if (client.connect(client_id.c_str())) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_led); // Subscribe to the LED control topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Initialize LED to off

  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Handle MQTT messages
}