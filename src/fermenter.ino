#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <MHZ.h>

// Define pins for sensors
#define DHTPIN 21 // DHT Digital Input Pin
#define DHTTYPE DHT11 // DHT11 or DHT22, depends on your sensor
#define CO2_IN 5 // PWM pin for CO2 sensor
#define MH_Z19_RX 16  // RX for MH-Z19C 
#define MH_Z19_TX 17  // TX for MH-Z19C
#define MQPin 33  // TX for MH-Z19C

// Initialize DHT and CO2 sensor
DHT dht(DHTPIN, DHTTYPE);
MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ::MHZ19C);

// WiFi credentials
const char* ssid = "Big";
const char* password = "13591359";

// MQTT settings
const char* mqttServer = "demo.thingsboard.io";
const int mqttPort = 1883;
const char* mqttUser = "W7FByEMblN9TsgtYmBB4"; // MQTT username (ThingsBoard access token)
const char* mqttTopic = "v1/devices/me/telemetry";

// MQTT client setup
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Connect to MQTT broker
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, NULL)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  // Initialize CO2 sensor
  Serial.println("MHZ 19C");
  co2.setDebug(true); // Enable debug for additional information

  if (co2.isPreHeating()) {
    Serial.print("Preheating");
    while (co2.isPreHeating()) {
      Serial.print(".");
      delay(5000);
    }
    Serial.println();
  }
}

void loop() {
  // Read data from DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read data from CO2 sensor
  int ppm_pwm = co2.readCO2PWM();     // Read CO2 concentration over PWM
  int co2Temperature = co2.getLastTemperature(); // Temperature from CO2 sensor
  int analogValue = analogRead(MQPin);

  // Check if readings are valid
  if (ppm_pwm < 0) ppm_pwm = 0;      // Handle invalid PWM reading
  if (co2Temperature < 0) co2Temperature = 0; // Handle invalid temperature reading

  // Prepare MQTT payload
  String mqttPayload = "{\"temperature\":" + String(temperature) +
                       ", \"humidity\":" + String(humidity) +
                       ", \"co2_pwm\":" + String(ppm_pwm) +
                       ", \"co2_temperature\":" + String(co2Temperature) + "}";

  // Publish the MQTT message
  client.publish(mqttTopic, mqttPayload.c_str());
  Serial.print("MQTT Published: ");
  Serial.println(mqttPayload);

  // Display sensor data in Serial Monitor
  Serial.print("DHT Temperature: "); Serial.print(temperature); Serial.println(" *C");
  Serial.print("DHT Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("CO2 PWM: "); Serial.print(ppm_pwm); Serial.println(" ppm");
  Serial.print("CO2 Sensor Temperature: "); Serial.print(co2Temperature); Serial.println(" *C");
  Serial.print("Analog Value: ");
  Serial.println(analogValue);
  delay(2000); // Delay to control the data sending frequency
}
