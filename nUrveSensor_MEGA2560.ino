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

//  Sensor vars
Adafruit_HTU21DF htu = Adafruit_HTU21DF();                                            //HTUD21DF I2C Address: 0x40 (cannot be changed)
Adafruit_TSL2561_Unified  tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  //TSL2561 I2C Address: 0X39 (can be changed)

// Accelerometer vars
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);                     // ADXL345 I2C Address: ????

// === CLEAN UP BELOW ===
// For GPS
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);


#define GPSECHO false       // Do we need this for production?
boolean usingInterrupt = false;   // Do we need this for production?
void useInterrupt(boolean);     // Func prototype keeps Arduino 0023 happy. Do we need this for production?

// === CLEAN UP ABOVE ===

// Define all the variables that we're going to need:
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

void loop() {
  IterationCounter ++;
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|                            - Start of Loop -                            |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Iteration: #");Serial.println(IterationCounter);
  
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
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.println(GPS.longitudeDegrees, 6);
      Serial.println(GPS.speed);
      Serial.println(GPS.angle);
      Serial.println(GPS.altitude);
      Serial.println((int)GPS.satellites);
    }
  }
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
