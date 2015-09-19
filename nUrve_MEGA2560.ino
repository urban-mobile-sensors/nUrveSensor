// Basic Includes
#include "SoftwareSerial.h"
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_Sensor.h"      // Adafruit, standard library for all sensors
#include "SD.h" // We're using a special SD Library from Adafruit. More information can be found here:  https://github.com/adafruit/SD


// Sensor Specific Includes
#include "Adafruit_HTU21DF.h"     // Temperature and Humidity Sensor - HTU21D-F
#include "Adafruit_TSL2561_U.h"   // Light Sensor - TSL2561
#include "Adafruit_ADXL345_U.h"   // Accelerometer - ADXL345
#include "Adafruit_GPS.h"         // GPS

//General vars
int IterationCounter = 0;

// DataLogger vars
const int chipSelect = 4;
File dataFile;

// GPS vars
#define GPSECHO false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
Adafruit_GPS GPS(&Serial1);

//  Sensor vars
Adafruit_HTU21DF htu = Adafruit_HTU21DF();                                            //HTUD21DF I2C Address: 0x40 (cannot be changed)
Adafruit_TSL2561_Unified  tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  //TSL2561 I2C Address: 0X39 (can be changed)

// Accelerometer vars
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);                     // ADXL345 I2C Address: ????

// Define all the variables that we're going to need:
String GPS_Date = "0000-00-00";
String GPS_Time = "HH:MM:SS.MS";
float GPS_Lat = -9999;
float GPS_Lon = -9999;
float GPS_Altitude = -9999;
float GPS_Speed = -9999;
float GPS_SpeedMin = -9999;
float GPS_SpeedMax = -9999;
float GPS_SpeedMea = -9999;
float GPS_SpeedMed = -9999;
float GPS_Angle = -9999;
float GPS_Sats = -9999;
float GPS_Fix = -9999;
float GPS_Quality = -9999;
float AMB_Temp = -9999;
float AMB_TempMin = -9999;
float AMB_TempMax = -9999;
float AMB_TempMea = -9999;
float AMB_TempMed = -9999;
float AMB_Humd = -9999;
float AMB_HumdMin = -9999;
float AMB_HumdMax = -9999;
float AMB_HumdMea = -9999;
float AMB_HumdMed = -9999;
float AMB_Lux = -9999;
float AMB_LuxMin = -9999;
float AMB_LuxMax = -9999;
float AMB_LuxMea = -9999;
float AMB_LuxMed = -9999;
float AMB_Snd = -9999;
float AMB_SndMin = -9999;
float AMB_SndMax = -9999;
float AMB_SndMea = -9999;
float AMB_SndMed = -9999;
float RDQ_AcX = -9999;
float RDQ_AcXMin = -9999;
float RDQ_AcXMax = -9999;
float RDQ_AcXMea = -9999;
float RDQ_AcXMed = -9999;
float RDQ_AcY = -9999;
float RDQ_AcYMin = -9999;
float RDQ_AcYMax = -9999;
float RDQ_AcYMea = -9999;
float RDQ_AcYMed = -9999;
float RDQ_AcZ = -9999;
float RDQ_AcZMin = -9999;
float RDQ_AcZMax = -9999;
float RDQ_AcZMea = -9999;
float RDQ_AcZMed = -9999;

const int AMB_SND_sampleWindow = 500; // Sample window width in mS (50 mS = 20Hz)
unsigned int AMB_SND_sample;

void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(115200);
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|               Urban Mobile Sensors, LLC - nUrve Sensor V1               |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| w: urbanmobilesensors.com | t: @urbansensors | e: info@urbansensors.com |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");

  setupDataLogger();        // Set up Data Logger
  setupGPS();               // Set up GPS 
//  setupComms();             // Set up Comms | PLACEHOLDER
  setupAmbientTemp();       // Set up Ambient Sensor 1 - Temp & Humidity
  setupAmbientLux();        // Set up Ambient Sensor 2 - Luminosity
  setupAmbientSound();      // Set up Ambient Sensor 3 - Sound
  setupRoadSensor();        // Set up Road Sensor
}


// ADDITIONAL CODE
// GPS
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
// END OF GPS
// END OF ADDITIONAL CODE

void loop() {
  IterationCounter ++;
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|                            - Start of Loop -                            |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Iteration: #");Serial.println(IterationCounter);
  
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Board Information: [TO DO]\n");
  Serial.println("| ...: ");

  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Communication Data: [TO DO]\n");
  Serial.print("| SSID: ");Serial.println("");
  Serial.print("| RSSI: ");Serial.println("");
  Serial.print("| Nets: ");Serial.println("");

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
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    if (GPS.fix) {
//      GPS_Date = Serial.println(GPS.year);
//      GPS_Time = "HH:MM:SS.MS";
      GPS_Lat = GPS.latitudeDegrees;
      GPS_Lon = GPS.longitudeDegrees;
      GPS_Speed = GPS.speed;
      GPS_Angle = GPS.angle;
      GPS_Altitude = GPS.altitude;
      GPS_Sats = GPS.satellites;
    }
  }
  // STILL TO DO HERE:
  // 1) Get Date as a single properly formatted string YYYY-MM-DD
  // 2) Get Time as a single properly formatted string HH:MM:SS.mmm
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| GPS Information:\n");
  Serial.print("| Date: ");Serial.println(GPS_Date);
  Serial.print("20");Serial.print(GPS.year, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.println(GPS.day, DEC);
  Serial.print("| Time: ");Serial.println(GPS_Time);
  Serial.print(GPS.hour, DEC); Serial.print(':');Serial.print(GPS.minute, DEC); Serial.print(':');Serial.print(GPS.seconds, DEC); Serial.print('.');Serial.println(GPS.milliseconds);
  Serial.print("| Lat: ");Serial.println(GPS_Lat,6);
  Serial.print("| Lon: ");Serial.println(GPS_Lon,6);
  Serial.print("| Speed (knots): ");Serial.println(GPS_Speed,2);
  Serial.print("| Altitude: ");Serial.println(GPS_Altitude,2);
  Serial.print("| Angle: ");Serial.println(GPS_Angle,2);
  Serial.print("| Sats: ");Serial.println(GPS_Sats,0);
  Serial.print("| Fix: ");Serial.println(GPS.fix,2);  // TO BE DONE
  Serial.print("| Quality: ");Serial.println(GPS.fixquality,2);  // TO BE DONE

  // END OF GPS CODE

  // =============================================================================================
  // AMBIENT SENSORS
  // =============================================================================================
  // Ambient set temperature and humidity
  AMB_Temp = htu.readTemperature();
  AMB_Humd = htu.readHumidity();

  // Ambient - Set Light Values
  sensors_event_t event;
  tsl.getEvent(&event);
  if (event.light) {AMB_Lux = event.light;} else {AMB_Lux = -9998;}

   // Ambient - Set Sound Values
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;

   // collect data for 50 mS
   while (millis() - startMillis < AMB_SND_sampleWindow)
   {
      AMB_SND_sample = analogRead(8);
      if (AMB_SND_sample < 1024)  // toss out spurious readings
      {
         if (AMB_SND_sample > signalMax)
         {
            signalMax = AMB_SND_sample;  // save just the max levels
         }
         else if (AMB_SND_sample < signalMin)
         {
            signalMin = AMB_SND_sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
   AMB_Snd = volts;
  
  // Output of data - For development purposes only
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Ambient Information:\n");
  Serial.print("| Temp: ");Serial.print(AMB_Temp,2);Serial.println("C");
  Serial.print("| Humid: ");Serial.print(AMB_Humd,2);Serial.println("%");
  Serial.print("| Lux: ");Serial.print(AMB_Lux,2);Serial.println("Lux");
  Serial.print("| Sound: ");Serial.print(AMB_Snd);Serial.println("vlts");

  // =============================================================================================
  // ACCELEROMTER CODE
  // =============================================================================================
  accel.getEvent(&event);
  RDQ_AcX = event.acceleration.x;
  RDQ_AcY = event.acceleration.y;
  RDQ_AcZ = event.acceleration.z;
  // Display the results (acceleration is measured in m/s^2)
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Accelerometer Information:\n");
  Serial.print("| X: "); Serial.print(RDQ_AcX); Serial.print("m/s^2\n");
  Serial.print("| Y: "); Serial.print(RDQ_AcY); Serial.print("m/s^2\n");
  Serial.print("| Z: "); Serial.print(RDQ_AcZ); Serial.print("m/s^2\n");

  // =============================================================================================
  // OTHER SENSORS
  // =============================================================================================
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Other Information... For later                                          |\n");

  // =============================================================================================
  // DATA LOGGING
  // =============================================================================================
  // make a string for assembling the data to log:
  String dataString = "";

  // Build the String
/*  
 *   
 for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ","; 
    }
  }
 */
 // We need to address issues with writing the header in each file.

  if(IterationCounter == 1) {
    //dataFile.println("ID,DATESTAMP,TIMESTAMP,GPS_LAT,GPS_LON,GPS_Speed,GPS_Alt,GPS_Sats,GPS_Fix,GPS_Quality,AMB_Temp,AMB_Humd,AMB_Lux,AMB_Snd,RDQ_AcX,RDQ_AcY,RDQ_AcZ");
    dataFile.println("ID,DATESTAMP,TIMESTAMP,GPS_LAT,GPS_LON,GPS_Speed,GPS_SpeedMin,GPS_SpeedMax,GPS_SpeedMea,GPS_SpeedMed,GPS_Alt,GPS_Sats,GPS_Fix,GPS_Quality,AMB_Temp,AMB_TempMin,AMB_TempMax,AMB_TempMea,AMB_TempMed,AMB_Humd,AMB_HumdMin,AMB_HumdMax,AMB_HumdMea,AMB_HumdMed,AMB_Lux,AMB_LuxMin,AMB_LuxMax,AMB_LuxMea,AMB_LuxMed,AMB_Snd,AMB_SndMin,AMB_SndMax,AMB_SndMea,AMB_SndMed,RDQ_AcX,RDQ_AcXMin,RDQ_AcXMax,RDQ_AcXMea,RDQ_AcXMed,RDQ_AcY,RDQ_AcYMin,RDQ_AcYMax,RDQ_AcYMea,RDQ_AcYMed,RDQ_AcZ,RDQ_AcZMin,RDQ_AcZMax,RDQ_AcZMea,RDQ_AcZMed");
  }
// LEFT HERE... I HAVE TO FIGURE OUT HOW TO FORMAT THE DATE AND TIME PROPERLY
//  String timestamp_len = sprintf(timestamp, "20%02d-%02d-%02dT%02d:%02d:%02dZ", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
//String FormattedDate = Serial.print("20");Serial.print(GPS.year, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.println(GPS.day, DEC);
//  String FormattedTime = Serial.print(GPS.hour, DEC); Serial.print(':');Serial.print(GPS.minute, DEC); Serial.print(':');Serial.print(GPS.seconds, DEC); Serial.print('.');Serial.println(GPS.milliseconds);
  
  // DATASTRING PRINTING
  dataFile.print("ID: ");dataFile.print(IterationCounter);dataFile.print(",");
  // START OF DATE STIME SNAFU
  dataFile.print("20");dataFile.print(GPS.year, DEC);dataFile.print(":");dataFile.print(GPS.month, DEC);dataFile.print(":");dataFile.print(GPS.month, DEC);dataFile.print(":");dataFile.print(GPS.day, DEC);dataFile.print(",");
  dataFile.print(GPS.hour, DEC); dataFile.print(':');dataFile.print(GPS.minute, DEC); dataFile.print(':');dataFile.print(GPS.seconds, DEC); dataFile.print('.');dataFile.print(GPS.milliseconds);dataFile.print(",");
  // END OF DATE TIME SNAFU
  dataFile.print(GPS_Lat,6);dataFile.print(",");
  dataFile.print(GPS_Lon,6);dataFile.print(",");
  dataFile.print(GPS_Speed,2);dataFile.print(",");
  dataFile.print(GPS_SpeedMin,2);dataFile.print(",");
  dataFile.print(GPS_SpeedMax,2);dataFile.print(",");
  dataFile.print(GPS_SpeedMea,2);dataFile.print(",");
  dataFile.print(GPS_SpeedMed,2);dataFile.print(",");
  dataFile.print(GPS_Altitude,2);dataFile.print(",");
  dataFile.print(GPS_Sats,2);dataFile.print(",");
  dataFile.print(GPS_Fix,2);dataFile.print(",");
  dataFile.print(GPS_Quality,2);dataFile.print(",");
  dataFile.print(AMB_Temp,2);dataFile.print(",");
  dataFile.print(AMB_TempMin,2);dataFile.print(",");
  dataFile.print(AMB_TempMax,2);dataFile.print(",");
  dataFile.print(AMB_TempMea,2);dataFile.print(",");
  dataFile.print(AMB_TempMed,2);dataFile.print(",");
  dataFile.print(AMB_Humd,2);dataFile.print(",");
  dataFile.print(AMB_HumdMin,2);dataFile.print(",");
  dataFile.print(AMB_HumdMax,2);dataFile.print(",");
  dataFile.print(AMB_HumdMea,2);dataFile.print(",");
  dataFile.print(AMB_HumdMed,2);dataFile.print(",");
  dataFile.print(AMB_Lux,2);dataFile.print(",");
  dataFile.print(AMB_LuxMin,2);dataFile.print(",");
  dataFile.print(AMB_LuxMax,2);dataFile.print(",");
  dataFile.print(AMB_LuxMea,2);dataFile.print(",");
  dataFile.print(AMB_LuxMed,2);dataFile.print(",");
  dataFile.print(AMB_Snd,2);dataFile.print(",");
  dataFile.print(AMB_SndMin,2);dataFile.print(",");
  dataFile.print(AMB_SndMax,2);dataFile.print(",");
  dataFile.print(AMB_SndMea,2);dataFile.print(",");
  dataFile.print(AMB_SndMed,2);dataFile.print(",");
  dataFile.print(RDQ_AcX,4);dataFile.print(",");
  dataFile.print(RDQ_AcXMin,4);dataFile.print(",");
  dataFile.print(RDQ_AcXMax,4);dataFile.print(",");
  dataFile.print(RDQ_AcXMea,4);dataFile.print(",");
  dataFile.print(RDQ_AcXMed,4);dataFile.print(",");
  dataFile.print(RDQ_AcY,4);dataFile.print(",");
  dataFile.print(RDQ_AcYMin,4);dataFile.print(",");
  dataFile.print(RDQ_AcYMax,4);dataFile.print(",");
  dataFile.print(RDQ_AcYMea,4);dataFile.print(",");
  dataFile.print(RDQ_AcYMed,4);dataFile.print(",");
  dataFile.print(RDQ_AcZ,4);dataFile.print(",");
  dataFile.print(RDQ_AcZMin,4);dataFile.print(",");
  dataFile.print(RDQ_AcZMax,4);dataFile.print(",");
  dataFile.print(RDQ_AcZMea,4);dataFile.print(",");
  dataFile.print(RDQ_AcZMed,4);
  dataFile.println("");
  // END OF DATASTRING

  // print to the serial port too:
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Output to data logger\n");
  Serial.print("| ");Serial.println(dataString);
  dataFile.flush();
  
  
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|                             - End of Loop -                             |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  delay(500); // delay is measured in milliseconds - 1000 ms= 1 s
}
