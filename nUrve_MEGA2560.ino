/* 
 * +-------------------------------------------------------------------------+
 * |               Urban Mobile Sensors, LLC - nUrve Sensor V1               |
 * +-------------------------------------------------------------------------+
 * | w: urbanmobilesensors.com | t: @urbansensors | e: info@urbansensors.com |
 * | g: https://github.com/urban-mobile-sensors/nUrveSensor                  |
 * +-------------------------------------------------------------------------+
 * | This code run on the first generation or the nUrve Sensors. The nUrve   |
 * | sensor collects multiple dimensions of ambient and road quality data.   |
 * | More information can be found on the urban mobile sensors website.      |
 * |                                                                         |
 * | Currently this code only works on Arduino MEGA based architectures.     |
 * | It can be ported over to Due based architecture, but cannot be run on   |
 * | UNO, or other boards that have 2K of SRAM.                              |
 * +-------------------------------------------------------------------------+
 * | STATISTICS:                                                             |
 * | Dynamic Memory:   3,478 bytes (42%) (Target: 35%)                       | 
 * | Storage Space:   32,732 bytes (12%)                                     |
 * | Compiled for:     Arduino MEGA 2560                                     |
 * +-------------------------------------------------------------------------+
 * | CONTACT INFORMATION:                                                    |
 * | Primary author:  Nadir Ait-Laoussine                                    |
 * | Primary contact: nadir at urbanmobilesensors dot com                    |
 * | Contributors:    Andrew Leonard                                         |
 * |                  Mickey Chea                                            |
 * |                  Chris Templeman                                        |
 * | Note;            This code is based on contributions by members listed  |
 * |                  above, and many examples available on adafruit and     |
 * |                  sparkfun's website.                                    |
 * |                  In addition special mentions go out to:                |
 * |                  Chip McClelland (https://www.hackster.io/chipmc)       |
 * |                  Kina Smith (http://www.kinasmith.com)                  |
 * | Build #:         24                                                     |
 * | Last update:     2015-10-08                                             |
 * +-------------------------------------------------------------------------+
 * | NEEDS                                                                   |
 * | a) Remove all serial.print - they take up SRAM | 20150919
 * | b) Calculate min/max/avg | 20150920
 * | c) Figure out how to make write to the data card faster (one string?) | 20150919
 * | d) Calculate median (more complex)
 * | e) Clean up code
 * +-------------------------------------------------------------------------+
 */
/* +-------------------------------------------------------------------------+
 * | INCLUDES / EXTERNAL LIBRARY REFERENCES                                  |
 * +-------------------------------------------------------------------------+
 */
#include "SoftwareSerial.h"
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_Sensor.h"    // Adafruit, standard library for all sensors
#include "SD.h"                 // We're using a special SD Library from Adafruit. 
                                // More information can be found here:  https://github.com/adafruit/SD
#include "Adafruit_HTU21DF.h"   // Temperature and Humidity Sensor - HTU21D-F
#include "Adafruit_TSL2561_U.h" // Light Sensor - TSL2561
#include "Adafruit_ADXL345_U.h" // Accelerometer - ADXL345
#include "Adafruit_GPS.h"       // GPS Sensor

/* +-------------------------------------------------------------------------+
 * | FONA DEV - IN HEAVY DEVELOPMENT                                       |
 * +-------------------------------------------------------------------------+
*/
#define FONA_RX  50//50 //connect to FONA RX  // was 3
#define FONA_TX  51//51 //connect to FONA TX  // was 4
#define FONA_RST 22//22 //connect to FONA RST  // ??
#define FONA_KEY 21// //connect to FONA KEY // was 6
#define FONA_PS  7// //connect to FONA PS  // was 7
int keyTime = 2000; // Time needed to turn ON or OFF the FONA


///
//String APN = "__PUT YOUR APN HERE!!__"; //Set APN for Mobile Service
//
//SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX); //initialize software serial
//String response; //global variable for pulling AT command responses from inside functions (there has to be a better way to do this)
//unsigned long ATtimeOut = 10000; // How long we will give an AT command to comeplete
//int SLEEP_MINUTES = 5; //Sleep time
//
////Sparkfun URL Building
//const String publicKey = "__PUT YOUR PUBLIC KEY HERE__"; //Public Key for data stream
//const String privateKey = "__PUT YOUR PRIVATE KEY HERE__"; //Private Key for data stream
//const byte NUM_FIELDS = 4; //number of fields in data stream
//const String fieldNames[NUM_FIELDS] = {"latitude", "longitude", "date", "time"}; //actual data fields
//String fieldData[NUM_FIELDS]; //holder for the data values
//
////Holders for Position Data
//String Lat;
//String Lon;
//String Date;
//String Time;
/* +-------------------------------------------------------------------------+
 * | / FONA DEV - IN HEAVY DEVELOPMENT                                       |
 * +-------------------------------------------------------------------------+
*/


/* +-------------------------------------------------------------------------+
 * | GLOBAL VARIABLES                                                        |
 * +-------------------------------------------------------------------------+
 * | We set global variables here to create default values. General          |
 * | convention that we use:                                                 |
 * |  -9999 : Variable created, but no data associated                       |
 * |  -9998 | Variable create, but error in reading sets it to -9998         |
 * +-------------------------------------------------------------------------+
 */
//General vars
int IterationCounter = 0;     // We use this for development purposes, may not need it in production

// DataLogger vars
const int chipSelect = 4;     // We're using Analog 4
File dataFile_Master;
File dataFile_Temp;                // This is the variable name for the data file

// GPS vars
#define GPSECHO false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
Adafruit_GPS GPS(&Serial1); // It's on Serial 0 (RX0, TX0)

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
  Serial.println("And we're off!");
  setupDataLogger();        // Set up Data Logger
  setupGPS();               // Set up GPS 
//  setupComms();             // Set up Comms | PLACEHOLDER
//  setupAmbientTemp();       // Set up Ambient Sensor 1 - Temp & Humidity
//  setupAmbientLux();        // Set up Ambient Sensor 2 - Luminosity
//  setupAmbientSound();      // Set up Ambient Sensor 3 - Sound
  setupRoadSensor();        // Set up Road Sensor
  Serial.println("All systems go!");
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
 Serial.print("| Iteration: #");Serial.println(IterationCounter);
/*  
 *   
  Serial.print("| Board Information: [TO DO]\n");
  Serial.print("| Communication Data: [TO DO]\n");
  Serial.print("| SSID: ");Serial.println("");
  Serial.print("| RSSI: ");Serial.println("");
  Serial.print("| Nets: ");Serial.println("");
*/
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

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > 1000) { 
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
/*
  Serial.print("| GPS Information:\n");
  Serial.print("| Date: ");Serial.println(GPS_Date);
  Serial.print("20");Serial.print(GPS.year, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.print(GPS.day, DEC);Serial.print(":");Serial.println(GPS.day, DEC);
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
*/
// END OF GPS CODE

  // =============================================================================================
  // AMBIENT SENSORS
  // =============================================================================================
  // Ambient set temperature and humidity
  // AMB_Temp = htu.readTemperature();
  // AMB_Humd = htu.readHumidity();
  // Ambient - Set Light Values
  sensors_event_t event;
  // tsl.getEvent(&event);
  //if (event.light) {AMB_Lux = event.light;} else {AMB_Lux = -9998;}

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
  /*
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Ambient Information:\n");
  Serial.print("| Temp: ");Serial.print(AMB_Temp,2);Serial.println("C");
  Serial.print("| Humid: ");Serial.print(AMB_Humd,2);Serial.println("%");
  Serial.print("| Lux: ");Serial.print(AMB_Lux,2);Serial.println("Lux");
  Serial.print("| Sound: ");Serial.print(AMB_Snd);Serial.println("vlts");
   */

  // =============================================================================================
  // ACCELEROMTER CODE
  // =============================================================================================
  accel.getEvent(&event);
  RDQ_AcX = event.acceleration.x;
  RDQ_AcY = event.acceleration.y;
  RDQ_AcZ = event.acceleration.z;
  // Display the results (acceleration is measured in m/s^2)
 /* 
  *  
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Accelerometer Information:\n");
  Serial.print("| X: "); Serial.print(RDQ_AcX); Serial.print("m/s^2\n");
  Serial.print("| Y: "); Serial.print(RDQ_AcY); Serial.print("m/s^2\n");
  Serial.print("| Z: "); Serial.print(RDQ_AcZ); Serial.print("m/s^2\n");
*/
  /* +===========================================================================================+
   * | OTHER SENSORS                                                                             |
   * +===========================================================================================+ 
   * | Currently there are no other sensors                                                      |
   * +===========================================================================================+ 
   */

  /* +===========================================================================================+
   * | DATA LOGGING                                                                              |
   * +===========================================================================================+ 
   * | Process:                                                                                  |
   * | 1) Open the data card (This should be in the setup)
   * | 2) If this is the first iteration, check to see if the file is there on the data card (use date format)
   *   -  If file is there, then go on (This should be in the setup)
   *      If file is not there, create and add header row
   *   3) Regardless of iteration, start to write data
   *   4) close data card (flush)
   *      - Check to see what exactly dataflush does. Does it push to file and write and empty variable, or does it clear session.
   * +===========================================================================================+ 
   */
  // make a string for assembling the data to log:
  String dataString = "";

/*  
 *   WE MAY NO LONGER NEED THIS AS WE'VE  DEFINED IT IN THE SETUP
 if(IterationCounter == 1) {
    // LATER: Add check to see if the file is there
    dataFile_Master.println("ID,DATESTAMP,TIMESTAMP,GPS_LAT,GPS_LON,GPS_Speed,GPS_SpeedMin,GPS_SpeedMax,GPS_SpeedMea,GPS_SpeedMed,GPS_Alt,GPS_Sats,GPS_Fix,GPS_Quality,AMB_Temp,AMB_TempMin,AMB_TempMax,AMB_TempMea,AMB_TempMed,AMB_Humd,AMB_HumdMin,AMB_HumdMax,AMB_HumdMea,AMB_HumdMed,AMB_Lux,AMB_LuxMin,AMB_LuxMax,AMB_LuxMea,AMB_LuxMed,AMB_Snd,AMB_SndMin,AMB_SndMax,AMB_SndMea,AMB_SndMed,RDQ_AcX,RDQ_AcXMin,RDQ_AcXMax,RDQ_AcXMea,RDQ_AcXMed,RDQ_AcY,RDQ_AcYMin,RDQ_AcYMax,RDQ_AcYMea,RDQ_AcYMed,RDQ_AcZ,RDQ_AcZMin,RDQ_AcZMax,RDQ_AcZMea,RDQ_AcZMed");
  }
 */

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

// LEFT HERE... I HAVE TO FIGURE OUT HOW TO FORMAT THE DATE AND TIME PROPERLY
//  String timestamp_len = sprintf(timestamp, "20%02d-%02d-%02dT%02d:%02d:%02dZ", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
//String FormattedDate = Serial.print("20");Serial.print(GPS.year, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.print(GPS.month, DEC);Serial.print(":");Serial.println(GPS.day, DEC);
//  String FormattedTime = Serial.print(GPS.hour, DEC); Serial.print(':');Serial.print(GPS.minute, DEC); Serial.print(':');Serial.print(GPS.seconds, DEC); Serial.print('.');Serial.println(GPS.milliseconds);
  
  // DATASTRING PRINTING
  dataFile_Master.print("ID: ");dataFile_Master.print(IterationCounter);dataFile_Master.print(",");
  // START OF DATE STIME SNAFU
  dataFile_Master.print("20");dataFile_Master.print(GPS.year, DEC);dataFile_Master.print(":");dataFile_Master.print(GPS.month, DEC);dataFile_Master.print(":");dataFile_Master.print(GPS.month, DEC);dataFile_Master.print(":");dataFile_Master.print(GPS.day, DEC);dataFile_Master.print(",");
  dataFile_Master.print(GPS.hour, DEC); dataFile_Master.print(':');dataFile_Master.print(GPS.minute, DEC); dataFile_Master.print(':');dataFile_Master.print(GPS.seconds, DEC); dataFile_Master.print('.');dataFile_Master.print(GPS.milliseconds);dataFile_Master.print(",");
  // END OF DATE TIME SNAFU
  dataFile_Master.print(GPS_Lat,6);dataFile_Master.print(",");
  dataFile_Master.print(GPS_Lon,6);dataFile_Master.print(",");
  dataFile_Master.print(GPS_Speed,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_SpeedMin,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_SpeedMax,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_SpeedMea,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_SpeedMed,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_Altitude,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_Sats,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_Fix,2);dataFile_Master.print(",");
  dataFile_Master.print(GPS_Quality,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_Temp,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_TempMin,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_TempMax,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_TempMea,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_TempMed,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_Humd,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_HumdMin,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_HumdMax,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_HumdMea,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_HumdMed,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_Lux,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_LuxMin,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_LuxMax,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_LuxMea,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_LuxMed,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_Snd,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_SndMin,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_SndMax,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_SndMea,2);dataFile_Master.print(",");
  dataFile_Master.print(AMB_SndMed,2);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcX,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcXMin,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcXMax,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcXMea,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcXMed,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcY,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcYMin,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcYMax,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcYMea,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcYMed,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcZ,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcZMin,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcZMax,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcZMea,4);dataFile_Master.print(",");
  dataFile_Master.print(RDQ_AcZMed,4);
  dataFile_Master.println("");
  // END OF DATASTRING

  dataFile_Master.flush();

  // THIS IS THE BACKUP LOG. THIS LOG IS WHAT GETS SENT THROUGH THE FONA
  // DATASTRING PRINTING
  dataFile_Temp.print("ID: ");dataFile_Temp.print(IterationCounter);dataFile_Temp.print(",");
  // START OF DATE STIME SNAFU
  dataFile_Temp.print("20");dataFile_Temp.print(GPS.year, DEC);dataFile_Temp.print(":");dataFile_Temp.print(GPS.month, DEC);dataFile_Temp.print(":");dataFile_Temp.print(GPS.month, DEC);dataFile_Temp.print(":");dataFile_Temp.print(GPS.day, DEC);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS.hour, DEC); dataFile_Temp.print(':');dataFile_Temp.print(GPS.minute, DEC); dataFile_Temp.print(':');dataFile_Temp.print(GPS.seconds, DEC); dataFile_Temp.print('.');dataFile_Temp.print(GPS.milliseconds);dataFile_Temp.print(",");
  // END OF DATE TIME SNAFU
  dataFile_Temp.print(GPS_Lat,6);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_Lon,6);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_Speed,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_SpeedMin,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_SpeedMax,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_SpeedMea,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_SpeedMed,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_Altitude,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_Sats,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_Fix,2);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_Quality,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_Temp,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_TempMin,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_TempMax,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_TempMea,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_TempMed,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_Humd,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_HumdMin,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_HumdMax,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_HumdMea,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_HumdMed,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_Lux,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_LuxMin,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_LuxMax,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_LuxMea,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_LuxMed,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_Snd,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_SndMin,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_SndMax,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_SndMea,2);dataFile_Temp.print(",");
  dataFile_Temp.print(AMB_SndMed,2);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcX,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcXMin,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcXMax,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcXMea,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcXMed,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcY,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcYMin,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcYMax,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcYMea,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcYMed,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcZ,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcZMin,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcZMax,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcZMea,4);dataFile_Temp.print(",");
  dataFile_Temp.print(RDQ_AcZMed,4);
  dataFile_Temp.println("");
  // END OF DATASTRING

  dataFile_Temp.flush();

//  PostDataDev();
  
  delay(1000); // delay is measured in milliseconds - 1000 ms= 1 s
}
