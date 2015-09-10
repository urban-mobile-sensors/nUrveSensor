// Basic Includes
#include "SoftwareSerial.h"
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_Sensor.h"      // Adafruit, standard library for all sensors
#include "SD.h"

// Sensor Specific Includes
#include "Adafruit_HTU21DF.h"     // Temperature and Humidity Sensor - HTU21D-F
#include "Adafruit_TSL2561_U.h"   // Light Sensor - TSL2561
#include "Adafruit_ADXL345_U.h"   // Accelerometer - ADXL345
#include "Adafruit_GPS.h"         // GPS
const int chipSelect = 10;        // For SD Data Logger

// For GPS
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

// For Sensors
Adafruit_HTU21DF htu = Adafruit_HTU21DF();    
//HTUD21DF I2C Address: 0x40 (cannot be changed)
Adafruit_TSL2561_Unified  tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
//TSL2561 I2C Address: 0X39 (can be changed)
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// ADXL345 I2C Address: ????

#define GPSECHO false       // Do we need this for production?
boolean usingInterrupt = false;   // Do we need this for production?
void useInterrupt(boolean);     // Func prototype keeps Arduino 0023 happy. Do we need this for production?

// Define all the variables that we're going to need:
int IterationCounter = 0;
float GPS_Lat = -9999;
float GPS_Lon = -9999;
float GPS_Time = -9999;
float GPS_Date = -9999;
float GPS_Alt = -9999;
float GPS_Speed = -9999;
float GPS_Angle = -9999;
float GPS_Sats = -9999;
float AMB_Temp = -9999;
float AMB_Humd = -9999;
float AMB_Lux = -9999;
float AMB_Snd = -9999;
float RDQ_AcX = -9999;
float RDQ_AcY = -9999;
float RDQ_AcZ = -9999;

const int AMB_SND_sampleWindow = 500; // Sample window width in mS (50 mS = 20Hz)
unsigned int AMB_SND_sample;
uint32_t timer = millis(); // PART OF GPS

void setup() {
  Serial.begin(115200);     // initialize serial and wait for the port to open:
  setupGPS();               // Set up GPS 
  setupDataLogger();        // Set up Data Logger
  setupComms();             // Set up Comms | PLACEHOLDER
  setupAmbientTemp();       // Set up Ambient Sensor 1 - Temp & Humidity
  setupAmbientLux();        // Set up Ambient Sensor 2 - Luminosity
  setupAmbientSound();      // Set up Ambient Sensor 3 - Sound
  setupRoadSensor();        // Set up Road Sensor
}

void loop() {
  IterationCounter ++;
  // GPS CODE
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    if (GPS.fix) {
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.println(GPS.longitudeDegrees, 6);
      Serial.println(GPS.speed);
      Serial.println(GPS.angle);
      Serial.println(GPS.altitude);
      Serial.println((int)GPS.satellites);
    }
  }
  // END OF GPS CODE
  
}
