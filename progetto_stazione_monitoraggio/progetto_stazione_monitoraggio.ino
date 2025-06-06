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

int sogliaCO2= 2000;
int sogliaTemp=35;
int sogliaPress=1030;
int sogliaHum=70;
int sogliaPM2_5=25;

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

  czr.init();


  Serial.begin(115200);
  Serial.print("COZIR_LIB_VERSION: ");
  Serial.println(COZIR_LIB_VERSION);
  Serial.println();

  
  czr.setOperatingMode(CZR_POLLING);
  delay(1000);

  while (!Serial) {}

  //BME280 INIT
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

   //SPS30 INIT
   sensirion_i2c_init();
   while (sps30_probe() != 0) {
     Serial.println("SPS sensor probing failed");
     delay(500);
   }
  Serial.println("SPS sensor probing successful");

   sps30_set_fan_auto_cleaning_interval_days(4);
   sps30_start_measurement();
   Serial.println("SPS30 measurements started");

   //WIFI CONNECTION
  Serial.print("Connecting to SSID: ");
   Serial.println(SECRET_SSID);
   while (WiFi.begin(SECRET_SSID, SECRET_PASS) != WL_CONNECTED) {
     Serial.print(".");
     delay(5000);
   }
   Serial.println("\nConnected to WiFi");

   //BROKER CONNECTION
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
     payload2 += "\"PM1_0\":" + String(m.mc_1p0, 3) + ",";
     payload2 += "\"PM2_5\":" + String(m.mc_2p5, 3) + ",";
     payload2 += "\"PM4_0\":" + String(m.mc_4p0, 3) + ",";
     payload2 += "\"PM10_0\":" + String(m.mc_10p0, 3) + ",";
     payload2 += "\"NC0_5\":" + String(m.nc_0p5, 3) + ",";
     payload2 += "\"NC1_0\":" + String(m.nc_1p0, 3) + ",";
     payload2 += "\"NC2_5\":" + String(m.nc_2p5, 3) + ",";
     payload2 += "\"NC4_0\":" + String(m.nc_4p0, 3) + ",";
     payload2 += "\"NC10_0\":" + String(m.nc_10p0, 3) + ",";
     payload2 += "\"Typical_particle_size\":" + String(m.typical_particle_size, 2);
     payload2 += "}";
     Serial.println(payload2); 
     mqttClient.print(payload2);
     mqttClient.endMessage();

    uint32_t c = czr.CO2();
    misure.CO2=czr.CO2();
    Serial.print("CO2 =\t");
    Serial.println(c);
    delay(1000);
     //TOPIC CO2
     mqttClient.beginMessage(topic3);
     mqttClient.print(c);
    mqttClient.endMessage();
 
 
 
 
    checkAlarm();
    
   }
}

void checkAlarm()
{
  String payload = "{";
  payload += "\"PM1_0\":" + String(misure.PM1_0 > 10) + ",";
  payload += "\"PM2_5\":" + String(misure.PM2_5 > 10) + ",";
  payload += "\"PM4_0\":" + String(misure.PM4_0 > 10) + ",";
  payload += "\"NC10_0\":" + String(misure.PM10_0 > 10) + ",";
  payload += "\"NC0_5\":" + String(misure.NC0_5 > 10) + ",";
  payload += "\"NC1_0\":" + String(misure.NC1_0 > 10) + ",";
  payload += "\"NC2_5\":" + String(misure.NC2_5 > 10) + ",";
  payload += "\"NC4_0\":" + String(misure.NC4_0 > 10) + ",";
  payload += "\"NC10_0\":" + String(misure.NC10_0 > 10) + ",";
  payload += "\"TEMPERATURE\":" + String(misure.temp > 10) + ",";
  payload += "\"PRESSURE\":" + String(misure.pres> 10) + ",";
  payload += "\"HUMIDITY\":" + String(misure.hum > 10) + ",";
  payload += "\"CO2\":" + String(misure.CO2 > 10) + ",";
  payload += "\"Typical_particle_size\":" + String(misure.typical_particle_size > 10);
  payload += "}";

  mqttClient.beginMessage(topic4);
     mqttClient.print(payload);
    mqttClient.endMessage();
}
  
void readData(){

 //lettura BME280
  bme.read(misure.pres, misure.temp, misure.hum, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);

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

  ret = sps30_read_measurement(&m);
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
  client->println("Pa");
}

void sendCommand(String cmd) {
  Serial1.print(cmd);     // Usa direttamente Serial1
  Serial1.print("\r");    // Carriage return richiesto
  delay(500);             // Attendi la risposta del sensore

  // Stampa la risposta ricevuta (opzionale, per debug)
  while (Serial1.available()) {
    Serial.write(Serial1.read());
}
}
