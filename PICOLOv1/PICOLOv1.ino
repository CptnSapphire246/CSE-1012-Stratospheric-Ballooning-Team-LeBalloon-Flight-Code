/*
_____________________________________________________________
Code for the PICOLO Flight Computer
Code by: Radhakrishna Vojjala
Modified by: Nishanth Kavuru
Date of last modification: 27 Apr 2025
Version 5.5B
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
#include <cmath>

#define GPS_RUN_RATE  2.0 // Max GPS update speed in Hz. May not update at this speed.
#define DATA_RATE 1.0 // Max rate of data aqusition in Hz. Set to 100 or some huge number to remove the limiter
#define VERSION "1.0"
#define ESC_PIN 14  // ESC connected to GP14

// File header. Edit to add columns for other sensors.
String header = "hh:mm:ss,T(s),T(ms),Hz,Fix Type,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,ExtT(F),ExtT(C),IntT(F),IntT(C),Pa,kPa,ATM,PSI,MSTemp(C),MSTemp(F),Alt SL Ft,Alt SL M,Alt Rel Ft,Alt Rel M,VertVel(ft/s),VertVel(m/s),Accel(x),Accel(y),Accel(z),Deg/S(x),Deg/S(y),Deg/S(z),Ori(x),Ori(y),Ori(z),Mag_T(x),Mag_T(y),Mag_T(z)z`, Thrust, Throttle, Version:" + String(VERSION);

// Config variables.

// Define the pins for the HX711 communication
const uint8_t DATA_PIN = 12;  // Can use any pins!
const uint8_t CLOCK_PIN = 13; // Can use any pins!

Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);

Servo esc;
int throttle = 0;  // Stores throttle value
int cc = 0; 
bool motorOn = false;  // Motor state
bool AltMAX = false; // Indicates whether maximum altitude for recording data has been reached or not
bool usingM8N = true; // true for M8N, false for M9N
int pulse;

// Set the pins for the LEDs
int color = 26;

void setup() {
  systemSetup();
  Serial.begin(115200);

// Thrust Setup
  // wait for serial port to connect. Needed for native USB port only
  
  // Initialize the HX711
  Serial.println("Adafruit HX711 Test!");
  hx711.begin();
  Serial.println("Tareing...."); // read and toss 3 values each
  for (uint8_t t=0; t<3; t++) {
    printOLED("Taring\nAttempt: " + String(t+1));
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
  }

// ESC Setup
  esc.attach(ESC_PIN);
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1000);  // Min throttle for arming
  delay(3000);
}

unsigned long lastThrottleUpdate = 0;
const unsigned long throttleInterval = 2000; // time in ms between throttle changes
int currentThrottleIndex = 0;
bool cycleInProgress = false;

int count = 0;
float currpress = 0;
float prevpress = 0;
bool propON = false;
int propTime = 0;

void loop() {
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  int32_t weightA128 = hx711.readChannel(CHAN_A_GAIN_128);

  long msec = millis();
  long s = msec/1000;
  currpress = pressPa;

  if (currpress > prevpress && count < 10 && AltMAX == false && s >= 0 && absAltFt > 70000.00) {
    count++;
  }
  else if (count >= 10 && AltMAX == false) {
    AltMAX = true;
  }
  else if (AltMAX == false) {
    count = 0;
  }

  // Set Throttle and Serial Output Throttle Value
  if (s > 40 && propON == false && cc < 1) {
    propON = true;
    throttle = 5;
    propTime = 0;
  }

  if (propON == true) { 
    // Set throttle to x%
    pulse = map(throttle, 0, 100, 1000, 2000);
    esc.writeMicroseconds(pulse);
    propTime++;
  }
  
  // Default case
  else {
    int pulse = map(0, 0, 100, 1000, 2000);
    esc.writeMicroseconds(pulse);
  }

  if (propTime > 30 && propON == true) {
    pulse = map(0, 0, 100, 1000, 2000);
    throttle += 5;
    propTime = 0;
    --color;
  }

  if (color < 7) {
    color = 26;
    propON = false;
    cc = 1;
    throttle = 0;
  }

  prevpress = presskPa;

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
    //OLEDstr += "Lat: " + String(gpsLatInt) + "." + String(paddedNumber) + "\n";

    data += String(gpsLonInt); 
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLonDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    data += ",";
    //OLEDstr += "Lon: " + String(gpsLonInt) + "." + String(paddedNumber) + "\n";

    data += String(gpsAltFt);
    data += ",";
    // OLEDstr += "GPSft: " + String(gpsAltFt) + "\n";
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
    OLEDstr += "PresskPa: " + String(presskPa) + "\n";
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

    // data from thrust sensor and ESC (throttle percentage)
    data += String(weightA128);
    data += ",";
    OLEDstr += "Thrust: " + String(weightA128) + "\n";
    data += String(throttle);
    data += ",";
    OLEDstr += "Throttle(%): " + String(throttle) + "\n";

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