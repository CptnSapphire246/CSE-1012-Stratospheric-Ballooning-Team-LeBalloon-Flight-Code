/*
_____________________________________________________________
Code for the PICOLO Flight Computer
Code by: Radhakrishna Vojjala
Date of last modification: 23 Jan 2025
Version 1.0
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

#define GPS_RUN_RATE  2.0 // Max GPS update speed in Hz. May not update at this speed.
#define DATA_RATE 1.0 // Max rate of data aqusition in Hz. Set to 100 or some huge number to remove the limiter
#define VERSION "1.0"

// Config variables.

bool usingM8N = true; // true for M8N, false for M9N

// File header. Edit to add columns for other sensors.

String header = "hh:mm:ss,T(s),T(ms),Hz,Fix Type,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,ExtT(F),ExtT(C),IntT(F),IntT(C),Pa,kPa,ATM,PSI,MSTemp(C),MSTemp(F),Alt SL Ft,Alt SL M,Alt Rel Ft,Alt Rel M,VertVel(ft/s),VertVel(m/s),Accel(x),Accel(y),Accel(z),Deg/S(x),Deg/S(y),Deg/S(z),Ori(x),Ori(y),Ori(z),Mag_T(x),Mag_T(y),Mag_T(z)z`,Version:" + String(VERSION);

void setup() {

  systemSetup();
}

void loop() {

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
