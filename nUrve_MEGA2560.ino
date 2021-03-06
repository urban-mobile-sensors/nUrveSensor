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
 * |                                                                         |
 * | Note on data sizing:                                                    |
 * | As written, the code records data once a second and writes to an SD     |
 * | card. We write to the master field and to a temporary file. The temp    |
 * | is uploaded through a cellular data connection.                         |
 * | Early testing shows that the script writes about 23.65MB a day, or      |
 * | 710MB a month (30 days). Assumptions: writing 24/7. Therefore you need  |                    
 * | a minimum of 2GB on the SD Card, we recommend 4GB.                      |
 * +-------------------------------------------------------------------------+
 * | STATISTICS:                                                             |
 * | Dynamic Memory:   3,521 bytes (42%) (Target: 35%; Max: 60%)             | 
 * | Storage Space:   34,960 bytes (13%)                                     |
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
 * | Build #:         30                                                     |
 * | Last update:     2015-10-09                                             |
 * +-------------------------------------------------------------------------+
 * | NEEDS                                                                   |
 * | NEXT VIP:
 * | + Calculate min/max/avg for light/temp/humid/sound in same way as ADXL 
 * | + In set up or board, go through and add basic info that we'll need
 * | ------------------------------------------------------------------------|
 * | + Remove all serial.print - they take up SRAM (before roll out)
 * | + Figure out how to make write to the data card faster (one string?) | 20150919
 * | + Calculate median (more complex)
 * | + Clean up code
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
//#define FONA_RX  50//50 //connect to FONA RX  // was 3
//#define FONA_TX  51//51 //connect to FONA TX  // was 4
//#define FONA_RST 22//22 //connect to FONA RST  // ??
//#define FONA_KEY 21// //connect to FONA KEY // was 6
//#define FONA_PS  7// //connect to FONA PS  // was 7
//int keyTime = 2000; // Time needed to turn ON or OFF the FONA
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
int IterationCounter = 0;                   // We use this for development purposes, may not need it in production

// DataLogger vars
const int chipSelect = 4;                   // We're using Analog 4
File dataFile_Master;                       // Master dataFile    - This is the master logfile
File dataFile_Temp;                         // Temporary dataFile - This is a temporary logfile the variable name for the data file

// GPS vars
#define GPSECHO false                       // We do not want GPS echo
boolean usingInterrupt = false;             // We are not using interrupt
void useInterrupt(boolean);                 // Not sure what this does, but seems to barf if we don't have it - Func prototype keeps Arduino 0023 happy
Adafruit_GPS GPS(&Serial1);                 // TX/RX on MEGA is on 2560 (RX0, TX0)

//  Sensor vars
Adafruit_HTU21DF htu = Adafruit_HTU21DF();  //HTUD21DF I2C Address: 0x40 (cannot be changed)
Adafruit_TSL2561_Unified  tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  //TSL2561 I2C Address: 0X39 (can be changed)

// Accelerometer vars
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);                     // ADXL345 I2C Address: ????

// Define all the variables that we're going to need:
String GPS_Date = "1900-01-01";
String GPS_Time = "00:00:00.00";
String GPS_DateTimeStamp = "1900-01-01 00:00:00.00";
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
float RDQ_AcXMin = 9999;
float RDQ_AcXMax = -9999;
float RDQ_AcXTot = 0;
float RDQ_AcXMea = -9999;
float RDQ_AcXMed = -9999;
float RDQ_AcY = -9999;
float RDQ_AcYMin = 9999;
float RDQ_AcYMax = -9999;
float RDQ_AcYTot = 0;
float RDQ_AcYMea = -9999;
float RDQ_AcYMed = -9999;
float RDQ_AcZ = -9999;
float RDQ_AcZMin = 9999;
float RDQ_AcZMax = -9999;
float RDQ_AcZTot = 0;
float RDQ_AcZMea = -9999;
float RDQ_AcZMed = -9999;

const int SamplingWindow = 500; // Sample window width in mS (50 mS = 20Hz)
unsigned int AMB_SND_sample; //??

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
  Serial.println("PLACEHOLDER FOR FIRST RUN");
  // FIRST RUN, SETS ALL VARS
  Serial.println("... nothing for now");
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

uint32_t timer = millis(); // This is a timer
// END OF GPS
// END OF ADDITIONAL CODE

void loop() {
  IterationCounter ++;
  Serial.print("| Iteration: #");Serial.println(IterationCounter);
  //Serial.print("| Board Information: [TO DO]\n");Serial.print("| Communication Data: [TO DO]\n");Serial.print("| SSID: ");Serial.println("");Serial.print("| RSSI: ");Serial.println("");Serial.print("| Nets: ");Serial.println("");

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
      GPS_DateTimeStamp = "20" + String(GPS.year) + "-" + String(GPS.month) + "-" + String(GPS.day) + " " + String(GPS.hour, DEC) + ":" + String(GPS.minute, DEC) + ":" + String(GPS.seconds, DEC) + "." + String(GPS.milliseconds);
      // FUTURE: Timezone
      GPS_Lat = GPS.latitudeDegrees;
      GPS_Lon = GPS.longitudeDegrees;
      GPS_Speed = GPS.speed;
      GPS_Angle = GPS.angle;
      GPS_Altitude = GPS.altitude;
      GPS_Sats = GPS.satellites;
    }
  }
//  Serial.print("| GPS Information:\n");
//  Serial.print("| DateTimestamp: ");Serial.println(GPS_DateTimeStamp);
//  Serial.print("| Lat: ");Serial.println(GPS_Lat,6);
//  Serial.print("| Lon: ");Serial.println(GPS_Lon,6);
//  Serial.print("| Speed (knots): ");Serial.println(GPS_Speed,2);
//  Serial.print("| Altitude: ");Serial.println(GPS_Altitude,2);
//  Serial.print("| Angle: ");Serial.println(GPS_Angle,2);
//  Serial.print("| Sats: ");Serial.println(GPS_Sats,0);
//  Serial.print("| Fix: ");Serial.println(GPS.fix,2);  // TO BE DONE
//  Serial.print("| Quality: ");Serial.println(GPS.fixquality,2);  // TO BE DONE
// END OF GPS CODE

  // =============================================================================================
  // BEGIN OF MEASURING
  // =============================================================================================
  // We are measuring everithing inside a giant loop. We basically record for 500 ms, and then right out all the values to the SD Card after this loop
  // This applies for all ambient sensors (Temp, Humid, Light, Sound), and the Accelerometer

  // Reset all variables to default values
  RDQ_AcX = -9999; RDQ_AcXMin = 9999; RDQ_AcXMax = -9999; RDQ_AcXTot = 0; RDQ_AcXMea = -9999; RDQ_AcXMed = -9999;  //RDQ X
  RDQ_AcY = -9999; RDQ_AcYMin = 9999; RDQ_AcYMax = -9999; RDQ_AcYTot = 0; RDQ_AcYMea = -9999; RDQ_AcYMed = -9999;  //RDQ Y
  RDQ_AcZ = -9999; RDQ_AcZMin = 9999; RDQ_AcZMax = -9999; RDQ_AcZTot = 0; RDQ_AcZMea = -9999; RDQ_AcZMed = -9999;  //RDQ Z

  
  unsigned long startMillis= millis();  // Start of sample window
  sensors_event_t event;  // Collect the event
 
  while (millis() - startMillis < SamplingWindow) {
    accel.getEvent(&event);

    // =============================================================================================
    // ACCELEROMTER
    // =============================================================================================
    // Set variables
    RDQ_AcX = event.acceleration.x;
    RDQ_AcY = event.acceleration.y;
    RDQ_AcZ = event.acceleration.z;

    // AcX
    if (RDQ_AcX > RDQ_AcXMax)
      {
        RDQ_AcXMax = RDQ_AcX;  // We found the local max, save it!
      }
    if (RDQ_AcX < RDQ_AcXMin)
      {
        RDQ_AcXMin = RDQ_AcX;  // We found the local min, save it!
      }
    RDQ_AcXTot = RDQ_AcXTot + RDQ_AcX; // We're summing up all the values to get the total values for AcX, we'll use that to get the mean.

    // AcY
    if (RDQ_AcY > RDQ_AcYMax)
      {
        RDQ_AcYMax = RDQ_AcY;  // We found the local max, save it!
      }
    if (RDQ_AcY < RDQ_AcYMin)
      {
        RDQ_AcYMin = RDQ_AcY;  // We found the local min, save it!
      }
    RDQ_AcYTot = RDQ_AcYTot + RDQ_AcY; // We're summing up all the values to get the total values for AcX, we'll use that to get the mean.

    // AcZ
    if (RDQ_AcZ > RDQ_AcZMax)
      {
        RDQ_AcZMax = RDQ_AcZ;  // We found the local max, save it!
      }
    if (RDQ_AcZ < RDQ_AcZMin)
      {
        RDQ_AcZMin = RDQ_AcZ;  // We found the local min, save it!
      }
    RDQ_AcZTot = RDQ_AcZTot + RDQ_AcZ; // We're summing up all the values to get the total values for AcX, we'll use that to get the mean.
    }

    RDQ_AcXMea = RDQ_AcXTot / SamplingWindow; // Calculate mean (tot/count)
    RDQ_AcYMea = RDQ_AcYTot / SamplingWindow; // Calculate mean (tot/count)
    RDQ_AcZMea = RDQ_AcZTot / SamplingWindow; // Calculate mean (tot/count)
    
// SAMPLING WINDOW CODE DONE UP TO HERE
 
// =============================================================================================
  // AMBIENT SENSORS
  // =============================================================================================
  // Ambient set temperature and humidity
  // AMB_Temp = htu.readTemperature();
  // AMB_Humd = htu.readHumidity();
  // Ambient - Set Light Values
//  sensors_event_t event;
  // tsl.getEvent(&event);
  //if (event.light) {AMB_Lux = event.light;} else {AMB_Lux = -9998;}

   // Ambient - Set Sound Values
   unsigned long AstartMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;
   unsigned int signalTot = 0;
   int signalAvg = 0;
   int signalMed = 0;
   int signalCount = 0;
   

   // Collect data for 500 mS - That's what SamplingWindow is set to
   while (millis() - AstartMillis < SamplingWindow)
   {
      // Collect Data
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
   double volts_min = (signalMin * 3.3) / 1024;
   double volts_max = (signalMax * 3.3) / 1024;
   double volts_avg = ((signalTot/signalCount) * 3.3) / 1024;
   double volts_med = 0; // Right now we do nothing
   AMB_Snd = volts;         // Spot Sound
   AMB_SndMin = volts_min;  // 
   AMB_SndMax = volts_max;  //
   AMB_SndMea = volts_avg;  //
   AMB_SndMed = AMB_SndMed; // Right now we do nothing

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
//  accel.getEvent(&event);
//  RDQ_AcX = event.acceleration.x;
//  RDQ_AcY = event.acceleration.y;
//  RDQ_AcZ = event.acceleration.z;
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
  
  // DATASTRING PRINTING
  dataFile_Master = SD.open("m_log.csv", FILE_WRITE);
  dataFile_Master.print(IterationCounter);dataFile_Master.print(",");
  dataFile_Master.print(GPS_DateTimeStamp);dataFile_Master.print(",");
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
  dataFile_Master.close();

  // THIS IS THE BACKUP LOG. THIS LOG IS WHAT GETS SENT THROUGH THE FONA
  // DATASTRING PRINTING
  dataFile_Temp = SD.open("t_log.csv", FILE_WRITE);
  dataFile_Temp.print("ID: ");dataFile_Temp.print(IterationCounter);dataFile_Temp.print(",");
  dataFile_Temp.print(GPS_DateTimeStamp);dataFile_Temp.print(",");
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
  dataFile_Temp.close();

//  PostDataDev();
  
  delay(700); // delay is measured in milliseconds - 1000 ms= 1 s
}
