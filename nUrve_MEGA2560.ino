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

//General vars
int IterationCounter = 0;

// DataLogger vars
const int chipSelect = 10; // For SD Data Logger

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

void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(115200);
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|               Urban Mobile Sensors, LLC - nUrve Sensor V1               |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| w: urbanmobilesensors.com | t: @urbansensors | e: info@urbansensors.com |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");

//  setupDataLogger();        // Set up Data Logger
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
//  AMB_Temp = htu.readTemperature();
//  AMB_Humd = htu.readHumidity();

  // Ambient - Set Light Values
  sensors_event_t event;
/*  
 *   
 tsl.getEvent(&event);
  if (event.light) {AMB_Lux = event.light;} else {AMB_Lux = -9998;}
*/
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
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|                             - End of Loop -                             |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  delay(1000); // delay is measured in milliseconds - 1000 ms= 1 s
}
