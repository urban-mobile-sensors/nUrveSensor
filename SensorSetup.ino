/* This file is meant to set up an configure all the sensors when the device first turns on.
 * Process: 
 * 1) Basic Stuff
 * 2) Communication Stuff
 * 3) Location Tracking
 * 4) Ambient Sensors
 * 5) Road Quality Sensor
 * 6) Air Quality Sensor - Gas
 * 7) Air Quality Sensor - Particulates
 * 8) Other / Future
 */
void setupComms() {
//  Serial.println("LATER: Comms");
}
void setupDataLogger() {
//  Serial.println("LATER: Data logger");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
}
void setupGPS() {
//  Serial.println("Adafruit GPS library basic test!");
//  GPS.begin(9600);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
//  GPS.sendCommand(PGCMD_ANTENNA);
//  useInterrupt(true);
  delay(1000);
 }
/* +---------------------------------------------------+
 * | 4A) Ambient Sensors - Temperature & Humidity      |
 * +---------------------------------------------------+*/
void setupAmbientTemp() {
  if (!htu.begin()) {
    Serial.println("[] ERROR: Could not find the Temperature & Humidity sensor. Please check your wiring!");
    while (1);
  }
}
/* +---------------------------------------------------+
 * | 4B) Ambient Sensors - Luminosity                  |
 * +---------------------------------------------------+*/
void setupAmbientLux() {
  if(!tsl.begin()) {
    Serial.println("[] ERROR: Could not find the Luminosity Sensor. Please check your wiring!");
    while (1);
  } 
}
/* +---------------------------------------------------+
 * | 4C) Ambient Sensors - Sound                       |
 * +---------------------------------------------------+*/
// Electret is wired to pin...
void setupAmbientSound() {
  // There is nothing for the sound sensor
}
/* +---------------------------------------------------+
 * | 5) Road Quality - Accelerometer                   |
 * +---------------------------------------------------+*/
void setupRoadSensor() {
//  Serial.println("| ADXL_345: Test & Config...");
 if(!accel.begin())
  {
    Serial.println("[] ERROR: Could not find the ADXL345. Please check your wiring!");
    while(1);
  }
}
