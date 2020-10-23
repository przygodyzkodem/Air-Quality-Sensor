/*
 * 
 * BME280 implementation based on sample project from www.randomnerdtutorials.com
 * Wiring on NodeMCU (for standard BME/BMP280 module):
 * 
 * BME280 <-> NodeMCU
 *    Vin <-> 3.3V
 *    GND <-> GND
 *    SCL <-> D1 (GPIO5)
 *    SDA <-> D2 (GPIO4)
 *    
 * PMS7003 implementation based on PMS Library by Marcin Kacki 
 * and code examples from https://github.com/fu-hsi/PMS
 * 
 * PMS 7003 Pinout: https://www.espruino.com/refimages/PMS7003_PMS7003_pins.gif
 * Wiring on NodeMCU (for PMS7003 module):
 * 
 *          PMS7003 <-> NodeMCU
 *          
 *    Pin 1,2 (+5V) <-> +5V
 *    Pin 3,4 (GND) <-> GND
 *    Pin 7 (RX)    <-> D4 (TX/GPIO2)
 *    Pin 9 (TX)    <-> D3 (RX/GPIO0)
 *    
 *    Note: For +5V you should use external 5V power supply
 *    and connect it's ground with NodeMCU GND. 
 *    You should hear and feel PMS7003 fan working 
 *    when it has enough voltage and current.
 */

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#define BLYNK_TOKEN_LENGTH 33

 
//PMS7003 Config
#include "PMS.h"
#include <SoftwareSerial.h>

//BME280 Config
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
/*#include <SPI.h>
#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15*/

//Blynk Config
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <WiFiManager.h>
#include <EEPROM.h>

//PMS7003 Config
SoftwareSerial pmsSerial(D3, D4);

PMS pms(pmsSerial);
PMS::DATA data;

//BME280 Config
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

//Blynk Config
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char blynk_token[] = "";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "wifi_name";
char pass[] = "wifi_password";

WiFiManager wifiManager;
WiFiManagerParameter blynkToken("Blynk", "blynk token", blynk_token, BLYNK_TOKEN_LENGTH);

float temperature;
float pressure;
float humidity;
int pm1_0;
int pm2_5;
int pm10_0; 

void setup() {
  Serial.begin(115200);
 
  EEPROM.begin(512);
  char blynkTokenEEPROM[BLYNK_TOKEN_LENGTH] = "";
  EEPROM.get(0, blynkTokenEEPROM);
  Serial.print("Blynk token read from EEPROM: ");
  Serial.println(blynkTokenEEPROM);
  Serial.println("Blynk token from WebPanel: ");
  Serial.print(blynkToken.getValue());
  Serial.println();

  connectWiFi();

  if (blynkToken.getValue()[0] != blynk_token[0]) {
    Serial.println("Using Blynk token from WebPanel");
    Blynk.config(blynkToken.getValue());  
  } else {
    Serial.print("Using Blynk Token from EEPROM");
    Blynk.config(blynkTokenEEPROM);
  }
  
  if(!Blynk.connect()) {
    Serial.println("Blynk connection timed out.");
  }

  if (blynkToken.getValue()[0] != blynk_token[0]) {
    for (int i = 0; i < BLYNK_TOKEN_LENGTH; i++) {
      blynkTokenEEPROM[i] = blynkToken.getValue()[i];
    }
    Serial.print("Overwriting blynk token in EEPROM: ");
    Serial.println(blynkTokenEEPROM);
    EEPROM.put(0, blynkTokenEEPROM);
    EEPROM.commit();
  } 
  
  pmsSerial.begin(9600); //PMS7003 initialization

  //BME280 Initialization
  Serial.println(F("BME280 test"));

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();
}

void loop() {
  Blynk.run(); 
  if (pms.read(data))
  {
    printPMS7003Readings();
    printBME280Readings();
    delay(delayTime);
  }
}

void connectWiFi() {
  //wifiManager.resetSettings(); //WiFiManager remembers previously connected APs
  wifiManager.addParameter(&blynkToken);
  wifiManager.autoConnect("AirQualitySensor");
}

void printBME280Readings() {
  temperature = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");
  Blynk.virtualWrite(V1, temperature);

  pressure = bme.readPressure() / 100.0F;
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Blynk.virtualWrite(V2, pressure);

  humidity = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
  Blynk.virtualWrite(V3, humidity);

  Serial.println();
}

void printPMS7003Readings() {
  pm1_0 = data.PM_AE_UG_1_0;
  Serial.print("PM 1.0 (ug/m3): ");
  Serial.println(pm1_0);
  Blynk.virtualWrite(V4, pm1_0);

  pm2_5 = data.PM_AE_UG_2_5;
  Serial.print("PM 2.5 (ug/m3): ");
  Serial.println(pm2_5);
  Blynk.virtualWrite(V5, pm2_5);

  pm10_0 = data.PM_AE_UG_10_0;
  Serial.print("PM 10.0 (ug/m3): ");
  Serial.println(pm10_0);
  Blynk.virtualWrite(V6, pm10_0);

  Serial.println();
}
