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

void setup() {
  Serial.begin(115200);   
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
  if (pms.read(data))
  {
    printPMS7003Readings();
    printBME280Readings();
    delay(delayTime);
  }
}

void printBME280Readings() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void printPMS7003Readings() {
  Serial.print("PM 1.0 (ug/m3): ");
  Serial.println(data.PM_AE_UG_1_0);

  Serial.print("PM 2.5 (ug/m3): ");
  Serial.println(data.PM_AE_UG_2_5);

  Serial.print("PM 10.0 (ug/m3): ");
  Serial.println(data.PM_AE_UG_10_0);

  Serial.println();
}
