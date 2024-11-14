#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include "MHZ19.h"
// Define pins for sensors
#define DHTPIN 21 // DHT Digital Input Pin
#define DHTTYPE DHT11 // DHT11 or DHT22, depends on your sensor
#define CO2_IN 5 // PWM pin for CO2 sensor
#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 17  
#define MQPin 36  // TX for MH-Z19C
MHZ19 myMHZ19;                                             // Constructor for library

#if defined(ESP32)
HardwareSerial mySerial(2);                                // On ESP32 we do not require the SoftwareSerial library, since we have 2 USARTS available
#else
#include <SoftwareSerial.h>                                //  Remove if using HardwareSerial or non-uno compatible device
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial
#endif


unsigned long getDataTimer = 0;
// MQ135 sensor setup
#define placa "ESP32"
#define Voltage_Resolution 3.3
#define ADC_Bit_Resolution 12
#define type "MQ-135"
#define RatioMQ135CleanAir 3.6

// Initialize DHT and CO2 sensor
DHT dht(DHTPIN, DHTTYPE);
// MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ::MHZ19C);
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, MQPin, type);

// WiFi credentials
const char* ssid = "Big";
const char* password = "13591359";
const char* ssid2 = "JOJOCAFE_2.4G";
const char* password2 = "13591359";
const char* ssid3 = "ATA_2g";
const char* password3 = "wer1233211";

// MQTT settings
const char* mqttServer = "demo.thingsboard.io";
const int mqttPort = 1883;
const char* mqttUser = "W7FByEMblN9TsgtYmBB4"; // MQTT username (ThingsBoard access token)
const char* mqttTopic = "v1/devices/me/telemetry";

// MQTT client setup
WiFiClient espClient;
PubSubClient client(espClient);

// Task handles
TaskHandle_t co2TaskHandle = NULL;

// Function to alternate SSID
void alternateSSID() {
  if (strcmp(ssid, "Big") == 0) {
    ssid = "JOJOCAFE_2.4G";
    password = "13591359";
  } else if (strcmp(ssid, "JOJOCAFE_2.4G") == 0) {
    ssid = "ATA_2g";
    password = "wer1233211";
  } else {
    ssid = "Big";
    password = "13591359";
  }
}

void setup() {
  Serial.begin(9600);
  dht.begin();

  // Alternate SSID
  alternateSSID();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 3) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi, rotating SSID...");
    alternateSSID();
    WiFi.begin(ssid, password);
    attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 3) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi after rotating SSID.");
    return;
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
      mySerial.begin(9600);                               // (Uno example) device to MH-Z19 serial start
    myMHZ19.begin(mySerial);                                // *Serial(Stream) reference must be passed to library begin().

    myMHZ19.autoCalibration();        

  // Initialize MQ135 sensor
  MQ135.setRegressionMethod(1);
  MQ135.init();
  Serial.print("Calibrating MQ135 sensor, please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println(" done!");

  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }

  // Create CO2 task
  // xTaskCreatePinnedToCore(co2Task, "CO2 Task", 10000, NULL, 1, &co2TaskHandle, 1);
}

// void co2Task(void *pvParameters) {
//   while (true) {
//     int ppm_pwm = co2.readCO2PWM();     // Read CO2 concentration over PWM

//     if (ppm_pwm < 0) ppm_pwm = 0;      // Handle invalid PWM reading
//     // if (co2Temperature < 0) co2Temperature = 0; // Handle invalid temperature reading

//     Serial.print("CO2 PWM: "); Serial.print(ppm_pwm); Serial.println(" ppm");
//     Serial.print("CO2 Sensor Temperature: "); 
//     String co2Payload = "{\"co2_pwm\":" + String(ppm_pwm) + "}";
//     client.publish(mqttTopic, co2Payload.c_str());
//     Serial.print("MQTT Published: ");
//     Serial.println(co2Payload);
//     // Serial.print(co2Temperature); Serial.println(" *C");

//     vTaskDelay(2000 / portTICK_PERIOD_MS);
//   }
// }

void loop() {
  // Read data from DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    // Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read data from MQ135 sensor
  MQ135.update();
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = myMHZ19.getCO2();      
        int8_t Temp;
        Temp = myMHZ19.getTemperature();           

  // Prepare MQTT payload
  String mqttPayload = "{\"temperature\":" + String(temperature) +
                       ", \"humidity\":" + String(humidity) +
                       ", \"co2\":" + String(CO2) + "}";

  // Publish the MQTT message
  client.publish(mqttTopic, mqttPayload.c_str());
  Serial.print("MQTT Published: ");
  Serial.println(mqttPayload);

  // Display sensor data in Serial Monitor
  Serial.print("DHT Temperature: "); Serial.print(temperature); Serial.println(" *C");
  Serial.print("DHT Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("MH CO2: "); Serial.print(CO2 ); Serial.println(" ppm");
  Serial.print("MH TEMP: "); Serial.print(Temp ); Serial.println(" ppm");

  delay(5000); // Delay to control the data sending frequency
}