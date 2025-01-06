#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <time.h>

// Influxdb
#if defined(ESP32)
  #include <WiFiMulti.h>
  #define DEVICE "ESP32"
#elif defined(ESP8266)
  #include <ESP8266WiFiMulti.h>
  #define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

// WiFi AP SSID
#define WIFI_SSID "iPhony"
// WiFi password
#define WIFI_PASSWORD "A23No26dA202"

// InfluxDB configurations
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "iLhW7vP0lyO3iU7ZaGAhO_WwbmpJSjwFBgeB-f7qiQ-QTa-Fx2NQFIIZh3-A6_kGcyCIzpOoDynWvQth78CfHg=="
#define INFLUXDB_ORG "2e94ef7be60d4ca8"
#define INFLUXDB_BUCKET "Sensor-Data"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensorReadings("measurements");

// Thresholds
#define SEALEVELPRESSURE_HPA (1013.25)

#define THRESHOLD_TEMP 25
#define THRESHOLD_HUMIDITY 35
#define THRESHOLD_PRESSURE 1000 

// Replace with your network credentials
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

// MQTT Broker settings
const char *mqtt_broker = "broker.emqx.io";  // EMQX broker endpoint
const int mqtt_port = 8883;  // MQTT port (TLS)
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";

// Topics
const char *mqtt_topic_temp = "anami/bme280/temperature";
const char *mqtt_topic_pressure = "anami/bme280/pressure";
const char *mqtt_topic_humidity = "anami/bme280/humidity";
const char *mqtt_topic_alarm = "anami/bme280/alarm/status";
const char *mqtt_topic = "anami/bme280/alarm";

// Alarm Control
unsigned long overrideStartTime = 0;
const unsigned long overrideDuration = 30 * 1000; // 30 seconds in milliseconds
bool manualOverride = false;
bool alarmOn = false;

// NTP Server settings
const char *ntp_server = "pool.ntp.org";     // Default NTP server
const long gmt_offset_sec =  2 * 3600;       // GMT offset in seconds (adjust for your time zone)
const int daylight_offset_sec = 0;        // Daylight saving time offset in seconds

// GPIO pin where the LED is connected
const int ledPin = D6;

Adafruit_BME280 bme; // I2C

// WiFi and MQTT client initialization
BearSSL::WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);

// Load DigiCert Global Root CA ca_cert
static const char ca_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

// Function declarations
void connectToWiFi();
void connectToMQTT();
void syncTime();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void readAndPublishSensorData();
void initInfluxdbClient();
void addReadings(float temperature, float pressure, float humidity);

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Initialize LED to off
  Serial.begin(115200);
  connectToWiFi();
  syncTime();  // X.509 validation requires synchronization time
  initInfluxdbClient();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  connectToMQTT();

  // Initialize BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) {
      delay(1000); // Halt execution
    }
  } else {
    Serial.println("BME280 sensor initialized successfully.");
  }
}

void connectToWiFi() {
  // Setup wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void syncTime() {
    configTime(gmt_offset_sec, daylight_offset_sec, ntp_server);
    Serial.print("Waiting for NTP time sync: ");
    while (time(nullptr) < 8 * 3600 * 2) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("Time synchronized");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        Serial.print("Current time: ");
        Serial.println(asctime(&timeinfo));
    } else {
        Serial.println("Failed to obtain local time");
    }
}

void initInfluxdbClient() {
  // Check server connection
    if (client.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client.getServerUrl());
    } else {
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client.getLastErrorMessage());
    }

    // Add tags
    sensorReadings.addTag("device", DEVICE);
    sensorReadings.addTag("location", "garden");
    sensorReadings.addTag("sensor", "bme280");
}

void connectToMQTT() {
    BearSSL::X509List serverTrustedCA(ca_cert);
    espClient.setTrustAnchors(&serverTrustedCA);  // Set the trusted root certificate
    while (!mqtt_client.connected()) {
        String client_id = "esp8266-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
            mqtt_client.subscribe(mqtt_topic);
            mqtt_client.subscribe(mqtt_topic_alarm);
        } else {
            char err_buf[128];
            espClient.getLastSSLError(err_buf, sizeof(err_buf));
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.println(mqtt_client.state());
            Serial.print("SSL error: ");
            Serial.println(err_buf);
            delay(2000);
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received on topic ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  // Check the LED control message
  if (String(topic) == mqtt_topic) {
    if (message == "ON") {
      digitalWrite(ledPin, HIGH); // Turn LED on
      Serial.println("Alarm ON");

      if (!alarmOn) {
        mqtt_client.publish("anami/bme280/alarm/status", "ON");
      }
      alarmOn = true;
    } else if (message == "OFF" && manualOverride == false && alarmOn) {
      alarmOn = false;
      mqtt_client.publish("anami/bme280/alarm/status", "OFF (Override)");

      digitalWrite(ledPin, LOW); // Turn LED off
      Serial.println("LED OFF (Manual Override)");
      manualOverride = true;
      overrideStartTime = millis(); // Start override timer
    }
  }
}

void addReadings(float temperature, float pressure, float humidity) {
  // Add readings as fields to point
  sensorReadings.addField("temperature", temperature);
  sensorReadings.addField("humidity", humidity);
  sensorReadings.addField("pressure", pressure);

  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(client.pointToLineProtocol(sensorReadings));
  
  // Write point into buffer
  client.writePoint(sensorReadings);

  // Clear fields for next usage. Tags remain the same.
  sensorReadings.clearFields();
}

void publishSensorData() {
  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();

  // Publish data to separate MQTT topics
  mqtt_client.publish(mqtt_topic_temp, String(temperature).c_str());
  mqtt_client.publish(mqtt_topic_pressure, String(pressure).c_str());
  mqtt_client.publish(mqtt_topic_humidity, String(humidity).c_str());

  // Add reading to InfluxDB
  addReadings(temperature, pressure, humidity);
}

void loop() {
    if (!mqtt_client.connected()) {
        connectToMQTT();
    }
    mqtt_client.loop();

    // Handle override mode
    if (manualOverride) {
        if (millis() - overrideStartTime > overrideDuration) {
            manualOverride = false; // Exit override mode
        }
    } else {
        // Automatic control logic
        float temperature = bme.readTemperature();
        float humidity = bme.readHumidity();
        float pressure = bme.readPressure() / 100.0F;

        if (temperature > THRESHOLD_TEMP || humidity > THRESHOLD_HUMIDITY || pressure > THRESHOLD_PRESSURE) {
            digitalWrite(ledPin, HIGH);

            if (!alarmOn) {
              Serial.println("Threshold exceeded, alarm ON");
              mqtt_client.publish("anami/bme280/alarm/status", "ON: Threshold exceeded");
            }
            alarmOn = true;
        } else {
            digitalWrite(ledPin, LOW);

            if (alarmOn) {
              Serial.println("Normal conditions, alarm Off");
              mqtt_client.publish("anami/bme280/alarm/status", "OFF: Normal Conditions");
            }
            alarmOn = false;
        }
    }

    // Publish sensor values to MQTT
    publishSensorData();
}
