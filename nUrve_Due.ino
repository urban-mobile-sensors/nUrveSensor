// Basic Includes
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_Sensor.h"    // Adafruit, standard library for all sensors

// Sensor Specific Includes
#include "Adafruit_HTU21DF.h"   // Temperature and Humidity Sensor - HTU21D-F - This has been modified to remove the call to the delay library which is not compatible with the Due
#include "Adafruit_TSL2561_U.h" // Light Sensor - TSL2561
//#include ...  // Sound Sensor - No library for sound sensors
#include "Adafruit_ADXL345_U.h"

// Define sensor placeholders:
Adafruit_HTU21DF htu = Adafruit_HTU21DF();                                     //HTUD21DF is the temperature & humidity sensor. Using I2C bus, current address: 0x40 (cannot be changed)
Adafruit_TSL2561_Unified  tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);    //TSL2561 is the the luminosity sensors. Using I2C bus, current address: 0X39 (can be changed)
// Sound Sensors   //Currently there is no library reference for the Sound Sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Define output variables
// These are all the variables for all the sensors that we are going to be using.
// Order is:
//  1) Comms
//  2) Position
//  3) Ambient
//  4) Road Quality
//  5) Air Quality Sensor [FUTURE]
//  6) Dust Sensors [FUTURE]
//  7) Proximity Sensors [FUTURE]
//  8) Other Sensors [FUTURE]
// +--> 1) Comms
// +--> 2) Position
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

// Miscellaneous - Development Variables
int IterationCounter = 0;
  
void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(115200);
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|               Urban Mobile Sensors, LLC - nUrve Sensor V1               |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| w: urbanmobilesensors.com | t: @urbansensors | e: info@urbansensors.com |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");

  //Setup the various shields/sensors
  setupAmbientTemp();
  setupAmbientLux();
  setupAmbientSound();
  setupRoadSensor();
  Serial.print("| ALl Systems Go!\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  IterationCounter ++;
  Serial.print("| Round: ");Serial.println(IterationCounter);

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
  Serial.print("| Sound: ");Serial.print(AMB_Snd);Serial.println("db");

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
  /*
  Serial.print("| Other Information... For later                                          |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("|                             - End of Loop -                             |\n");
  Serial.print("+-------------------------------------------------------------------------+\n");
  */
  Serial.print("+-------------------------------------------------------------------------+\n");
  delay(1000); // delay is measured in milliseconds - 1000 ms= 1 s
}
