/*
_____________________________________________________________
Code for the PICOLO Flight Computer
Code by: Radhakrishna Vojjala
Modified by: Nishanth Kavuru
Date of last modification: 6 Apr 2025
Version 3.5
_____________________________________________________________

*/

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO055.h> 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MS5611.h>   // Download from: https://github.com/jarzebski/Arduino-MS5611
#include "Thermistor.h"
#include "Variables.h"
#include "StemmaQtOLED.h" // Custom Library to control the StemmQT OLED screen
#include "Adafruit_HX711.h"
#include <Servo.h>

#define GPS_RUN_RATE  2.0 // Max GPS update speed in Hz. May not update at this speed.
#define DATA_RATE 1.0 // Max rate of data aqusition in Hz. Set to 100 or some huge number to remove the limiter
#define VERSION "1.0"
#define ESC_PIN 14  // ESC connected to GP14

// Config variables.

// Define the pins for the HX711 communication
const uint8_t DATA_PIN = 12;  // Can use any pins!
const uint8_t CLOCK_PIN = 13; // Can use any pins!

Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);

Servo esc;
int throttle = 0;  // Stores throttle value
bool motorOn = false;  // Motor state

bool usingM8N = true; // true for M8N, false for M9N

bool AltMAX = false; // Indicates whether maximum altitude for recording data has been reached or not

bool usingM8N = true; // true for M8N, false for M9N

// File header. Edit to add columns for other sensors.

String header = "hh:mm:ss,T(s),T(ms),Hz,Fix Type,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,ExtT(F),ExtT(C),IntT(F),IntT(C),Pa,kPa,ATM,PSI,MSTemp(C),MSTemp(F),Alt SL Ft,Alt SL M,Alt Rel Ft,Alt Rel M,VertVel(ft/s),VertVel(m/s),Accel(x),Accel(y),Accel(z),Deg/S(x),Deg/S(y),Deg/S(z),Ori(x),Ori(y),Ori(z),Mag_T(x),Mag_T(y),Mag_T(z)z`, Thrust, Throttle, Version:" + String(VERSION);

void setup() {

  systemSetup();


  Serial.begin(115200);

// Thrust Setup
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) {
    delay(10);
  }

  Serial.println("Adafruit HX711 Test!");

  // Initialize the HX711
  hx711.begin();

  // read and toss 3 values each
  Serial.println("Tareing....");
  for (uint8_t t=0; t<3; t++) {
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
  }

// ESC Setup
  esc.attach(ESC_PIN);

  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000);  // Min throttle for arming
  delay(3000);

  Serial.println("Press 'S' once to switch on, type in your throttle, and press 'S' again to switch off");
}

void loop() {
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Serial Output Thrust Value
  // Read from Channel A with Gain 128, can also try CHAN_A_GAIN_64 or CHAN_B_GAIN_32
  // since the read is blocking this will not be more than 10 or 80 SPS (L or H switch)
  int32_t weightA128 = hx711.readChannel(CHAN_A_GAIN_128);

// Set Throttle and Serial Output Throttle Value
  if ((int)gpsAltFt % 2500 == 0 && AltMAX == false) {
    // Tare
    for (uint8_t t=0; t<3; t++) {
      hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
      hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    }
    // Set throttle to 50%
    throttle = 50;
    int pulse = map(throttle, 0, 100, 1000, 2000);
    esc.writeMicroseconds(pulse);
    delay(10000);
    pulse = map(0, 0, 100, 1000, 2000);
    esc.writeMicroseconds(pulse);
  }
  // Last propeller thrust trial will be held at 100,000ft
  else if ((int)gpsAltFt == 100000 && AltMAX == false) {
    // Tare
    for (uint8_t t=0; t<3; t++) {
      hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
      hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    }
    // Set throttle to 50%
    throttle = 50;
    int pulse = map(throttle, 0, 100, 1000, 2000);
    esc.writeMicroseconds(pulse);
    delay(10000);
    pulse = map(0, 0, 100, 1000, 2000);
    esc.writeMicroseconds(pulse);
    AltMAX = true;
  }

  // Only send throttle if motor is ON
  int pulse = motorOn ? map(throttle, 0, 100, 1000, 2000) : 1000;
  esc.writeMicroseconds(pulse);

  delay(100);
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  if ((millis() - nowTimeMS) >= loopTime) {
    systemUpdate();

    // assembling the data srting;

    data = "";
    OLEDstr = "";
    
    data += HHMMSS;
    data += ",";
    data += String(nowTimeS);
    data += ",";
    data += String(nowTimeMS);
    data += ",";
    data += String(freq);
    data += ",";
    data += fixTypeGPS;
    data += ",";
    data += String(pvtStatus);
    data += ",";
    data += String(SIV);
    data += ",";
    data += String(gpsMonth);
    data += "/";
    data += String(gpsDay);
    data += "/";
    data += String(gpsYear);
    data += ",";
    data += String(gpsHour);
    data += ":";
    data += String(gpsMinute);
    data += ":";
    data += String(gpsSecond);
    data += ".";

    if (gpsMillisecond < 10) {
      data += "00";
      data += String(gpsMillisecond);
      data += ",";
    }
    else if (gpsMillisecond < 100) {
      data += "0";
      data += String(gpsMillisecond);
      data += ",";
    }
    else{
      data += String(gpsMillisecond); 
      data += ",";
    }

    char paddedNumber[8]; // Buffer to hold the padded number (7 digits + null terminator)
    data += String(gpsLatInt);
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLatDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    data += ",";
    OLEDstr += "Lat: " + String(gpsLatInt) + "." + String(paddedNumber) + "\n";

    data += String(gpsLonInt); 
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLonDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    data += ",";
    OLEDstr += "Lon: " + String(gpsLonInt) + "." + String(paddedNumber) + "\n";

    data += String(gpsAltFt);
    data += ",";
    OLEDstr += "GPSft: " + String(gpsAltFt) + "\n";
    data += String(gpsAltM);
    data += ",";
    data += String(gpsHorizAcc);
    data += ",";
    data += String(gpsVertAcc);
    data += ",";
    data += String(gpsVertVelFt);
    data += ",";
    data += String(gpsVertVelM);
    data += ",";
    data += String(ecefStatus);
    data += ",";
    data += String(ecefX);
    data += ",";
    data += String(ecefY); 
    data += ",";
    data += String(ecefZ);
    data += ","; 
    data += String(velocityNED[0]);
    data += ",";
    data += String(velocityNED[1]); 
    data += ",";
    data += String(velocityNED[2]);
    data += ","; 
    data += String(gpsGndSpeed);
    data += ",";
    data += String(gpsHeading);
    data += ",";
    data += String(gpsPDOP);
    data += ",";
    data += String(outTempF);
    data += ",";
    data += String(outTempC);
    data += ",";
    data += String(inTempF);
    data += ",";
    data += String(inTempC);
    data += ",";
    data += String(pressPa);
    data += ",";
    data += String(presskPa);
    data += ",";
    data += String(pressATM);
    data += ",";
    data += String(pressPSI);
    data += ",";
    data += String(MStempC);
    data += ",";
    data += String(MStempF);
    data += ",";
    data += String(absAltFt);
    data += ",";
    OLEDstr += "MSft: " + String(absAltFt) + "\n";
    data += String(absAltM);
    data += ",";
    data += String(relAltFt);
    data += ",";
    data += String(relAltM);
    data += ",";
    data += String(vertVelFt);
    data += ",";
    data += String(vertVelM);
    data += ",";
    data += String(accelerometer[0]);
    data += ",";
    data += String(accelerometer[1]);
    data += ",";
    data += String(accelerometer[2]);
    data += ",";
    data += String(gyroscope[0]);
    data += ",";
    data += String(gyroscope[1]);
    data += ",";
    data += String(gyroscope[2]);
    data += ",";
    data += String(orientation[2]);
    data += ",";
    data += String(orientation[1]);
    data += ",";
    data += String(orientation[0]);
    data += ",";
    data += String(magnetometer[0]);
    data += ",";
    data += String(magnetometer[1]);
    data += ",";
    data += String(magnetometer[2]);
    data += ",";

    data += String(weightA128);
    data += ",";
    data += String(throttle);
    data += ",";


    /*
      data form additional sensors
    */

    Serial.println(data);
    SDstatus = logData(data, dataFilename);
    
    if (!SDstatus) {      
      digitalWrite(ERR_LED_PIN, HIGH);
      Serial.println("SD failed!");
    }

    OLEDstr += "Sats: " + String(SIV) + "  Hz: " + String(freq) + "\n";
    OLEDstr += "Ext: " + String(outTempF) + " F\nInt: " + String(inTempF) + " F\nMS: " + String(MStempF) + " F";
    printOLED(OLEDstr);

    prevTime = nowTimeMS;
  }
}
