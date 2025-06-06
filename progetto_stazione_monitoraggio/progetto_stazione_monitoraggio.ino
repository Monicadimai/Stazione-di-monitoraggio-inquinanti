#include <Wire.h>
#include <BME280I2C.h>
#include <sps30.h>
#include "cozir.h"
#include "SoftwareSerial.h"
#include "Arduino.h"

#include <ArduinoMqttClient.h>

COZIR czr (&Serial1);  //  RX, TX, optional inverse logic


#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
  #include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
  #include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
  #include <WiFiS3.h>
#endif

#include "arduino_secrets.h"  // Definisci SECRET_SSID e SECRET_PASS

#define SERIAL_BAUD 115200
#define DO_COZIR_CONFIGURATION false // metti a true solo la prima volta per configurare il range

struct valori{
    float temp;
    float hum;
    float pres;
    float PM1_0;
    float PM2_5;
    float PM4_0;
    float PM10_0;
    int CO2;
    float NC0_5;
    float NC1_0;
    float NC2_5;
    float NC4_0;
    float NC10_0;
    float typical_particle_size;

};

valori misure;

struct sps30_measurement m;
uint16_t data_ready;
int16_t ret;
uint32_t c;

int sogliaCO2= 2000;
int sogliaTemp=35;
int sogliaPress=1030;
int sogliaHum=70;
int sogliaParticleSize=2.5;
int sogliaPM1_0=20;
int sogliaPM2_5=25;
int sogliaPM4_0=50;
int sogliaPM10_0=50;
int sogliaNC0_5=700;
int sogliaNC1_0=500;
int sogliaNC2_5=400;
int sogliaNC4_0=300;
int sogliaNC10_0=150;

BME280I2C bme;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "broker.hivemq.com";
int port = 1883;

const char topic1[] = "arduino/BME280";
const char topic2[] = "arduino/SPS30";
const char topic3[] = "arduino/CO2";
const char topic4[] = "arduino/ALLARME";

const long interval = 1000;
unsigned long previousMillis = 0;

void setup() {
  Wire.begin();

  Serial1.begin(9600);
  Serial.begin(115200);
  while (!Serial) {}

  czr.init();

  czr.setOperatingMode(CZR_POLLING);  // Torna in modalità polling
  delay(500);

  Serial.print("COZIR_LIB_VERSION: ");
  Serial.println(COZIR_LIB_VERSION);
  Serial.println();

  // Inizializzazione BME280
  while (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch (bme.chipModel()) {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor (no humidity).");
      break;
    default:
      Serial.println("Unknown sensor detected!");
  }

  // Inizializzazione SPS30
  sensirion_i2c_init();
  while (sps30_probe() != 0) {
    Serial.println("SPS sensor probing failed");
    delay(500);
  }
  Serial.println("SPS sensor probing successful");

  sps30_set_fan_auto_cleaning_interval_days(4);
  sps30_start_measurement();
  Serial.println("SPS30 measurements started");

  // Connessione WiFi
  Serial.print("Connecting to SSID: ");
  Serial.println(SECRET_SSID);
  while (WiFi.begin(SECRET_SSID, SECRET_PASS) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("\nConnected to WiFi");

  // Connessione broker MQTT
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);
  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed. Error = ");
    Serial.println(mqttClient.connectError());
    Serial.println("Retrying in 5 seconds..");
    delay(5000);
  }
  Serial.println("Connected to MQTT broker");
}

void loop() {
   mqttClient.poll();

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
     previousMillis = currentMillis;

     readData();
     printData(&Serial);

     //TOPIC BME380
     mqttClient.beginMessage(topic1);
     String payload1 = "{";
     payload1 += "\"TEMPERATURE\":" + String(misure.temp, 2) + ",";
     payload1 += "\"HUMIDITY\":" + String(misure.hum, 2) + ",";
     payload1 += "\"PRESSURE\":" + String(misure.pres, 2);
     payload1 += "}";
     Serial.println(payload1); 
     mqttClient.print(payload1);
     mqttClient.endMessage();
    //TOPIC SPS30

     mqttClient.beginMessage(topic2);
     String payload2 = "{";
     payload2 += "\"PM1_0\":" + String(misure.PM1_0, 3) + ",";
     payload2 += "\"PM2_5\":" + String(misure.PM2_5, 3) + ",";
     payload2 += "\"PM4_0\":" + String(misure.PM4_0, 3) + ",";
     payload2 += "\"PM10_0\":" + String(misure.PM10_0, 3) + ",";
     payload2 += "\"NC0_5\":" + String(misure.NC0_5, 3) + ",";
     payload2 += "\"NC1_0\":" + String(misure.NC1_0, 3) + ",";
     payload2 += "\"NC2_5\":" + String(misure.NC2_5, 3) + ",";
     payload2 += "\"NC4_0\":" + String(misure.NC4_0, 3) + ",";
     payload2 += "\"NC10_0\":" + String(misure.NC10_0, 3) + ",";
     payload2 += "\"Typical_particle_size\":" + String(misure.typical_particle_size, 2);
     payload2 += "}";
     Serial.println(payload2); 
     mqttClient.print(payload2);
     mqttClient.endMessage();

    Serial.print("CO2 =\t");
    Serial.println(misure.CO2);
    delay(1000);
    //TOPIC CO2
    mqttClient.beginMessage(topic3);
    mqttClient.print(misure.CO2);
    mqttClient.endMessage();

    checkAlarm();
    
   }
}

void checkAlarm()
{
  String payload = "{";
  payload += "\"PM1_0\":" + String(misure.PM1_0 > sogliaPM1_0) + ",";
  payload += "\"PM2_5\":" + String(misure.PM2_5 > sogliaPM2_5) + ",";
  payload += "\"PM4_0\":" + String(misure.PM4_0 > sogliaPM4_0) + ",";
  payload += "\"NC10_0\":" + String(misure.PM10_0 > sogliaPM10_0) + ",";
  payload += "\"NC0_5\":" + String(misure.NC0_5 > sogliaNC0_5) + ",";
  payload += "\"NC1_0\":" + String(misure.NC1_0 > sogliaNC1_0) + ",";
  payload += "\"NC2_5\":" + String(misure.NC2_5 > sogliaNC2_5) + ",";
  payload += "\"NC4_0\":" + String(misure.NC4_0 > sogliaNC4_0) + ",";
  payload += "\"NC10_0\":" + String(misure.NC10_0 > sogliaNC10_0) + ",";
  payload += "\"TEMPERATURE\":" + String(misure.temp > sogliaTemp) + ",";
  payload += "\"PRESSURE\":" + String(misure.pres> sogliaPress) + ",";
  payload += "\"HUMIDITY\":" + String(misure.hum > sogliaHum) + ",";
  payload += "\"CO2\":" + String(misure.CO2 > sogliaCO2) + ",";
  payload += "\"Typical_particle_size\":" + String(misure.typical_particle_size > sogliaParticleSize);
  payload += "}";

  mqttClient.beginMessage(topic4);
  mqttClient.print(payload);
  mqttClient.endMessage();
}
  
void readData(){

 //lettura BME280
  bme.read(misure.pres, misure.temp, misure.hum, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
  misure.pres/=100;

  //lettura SPS30
  do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.println("Error reading data-ready flag");
      return;
    } else if (!data_ready) {
      delay(100);
    }
  } while (!data_ready);

  ret=sps30_read_measurement(&m);
  misure.PM1_0=m.mc_1p0;
  misure.PM2_5=m.mc_2p5;
  misure.PM4_0=m.mc_4p0;
  misure.PM10_0=m.mc_10p0;

  misure.NC0_5=m.nc_0p5;
  misure.NC1_0=m.nc_1p0;
  misure.NC2_5=m.nc_2p5;
  misure.NC4_0=m.nc_4p0;
  misure.NC10_0=m.nc_10p0;

  misure.typical_particle_size=m.typical_particle_size;
  
  misure.CO2=czr.CO2();

  if (ret < 0) {
    Serial.println("Error reading measurement");
  } else {
  }
}

void printData(Stream* client){
  client->print("Temp: ");
  client->print(misure.temp);
  client->print(" °C\tHumidity: ");
  client->print(misure.hum);
  client->print(" %\tPressure: ");
  client->print(misure.pres);
  client->println("hPa");
}

