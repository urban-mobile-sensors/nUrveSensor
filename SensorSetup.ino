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

/* +-------------------------------------------------------------------------+
 * | DATA LOGGER                                                             |
 * +-------------------------------------------------------------------------+
 * | 1) Define Pins 
 * | 2) Check the card, can we read/access it?
 * | 3) Check the file, if it's there, do nothing, if it's not write the header |
 * +-------------------------------------------------------------------------+
 */
void setupDataLogger() {
  // Serial.println("| SD Break-out: Test & Config...");
  // 1) Set up pinMode // WHAT EXACTLY IS THAT????
  pinMode(SS, OUTPUT);
  
  // 2) See if the card is present and can be initialized:
  if (!SD.begin(10,11,12,13)) {
    Serial.println("[] ERROR: Card failed, or not present");
    while (1) ;
  }
  Serial.println("| Success - Card initialized.");
  
  // 3) Check to see if the data file is here is there.
  // If not, create it and add header row...
  // Keep data data file open
  if (!SD.exists("datalog.txt")) {
    // Create file
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    // Add header row
    dataFile.println("ID,DATESTAMP,TIMESTAMP,GPS_LAT,GPS_LON,GPS_Speed,GPS_SpeedMin,GPS_SpeedMax,GPS_SpeedMea,GPS_SpeedMed,GPS_Alt,GPS_Sats,GPS_Fix,GPS_Quality,AMB_Temp,AMB_TempMin,AMB_TempMax,AMB_TempMea,AMB_TempMed,AMB_Humd,AMB_HumdMin,AMB_HumdMax,AMB_HumdMea,AMB_HumdMed,AMB_Lux,AMB_LuxMin,AMB_LuxMax,AMB_LuxMea,AMB_LuxMed,AMB_Snd,AMB_SndMin,AMB_SndMax,AMB_SndMea,AMB_SndMed,RDQ_AcX,RDQ_AcXMin,RDQ_AcXMax,RDQ_AcXMea,RDQ_AcXMed,RDQ_AcY,RDQ_AcYMin,RDQ_AcYMax,RDQ_AcYMea,RDQ_AcYMed,RDQ_AcZ,RDQ_AcZMin,RDQ_AcZMax,RDQ_AcZMea,RDQ_AcZMed");
  }
}

void setupGPS() {
  Serial.print("+-------------------------------------------------------------------------+\n");
  Serial.print("| Adafruit GPS library basic test. \n");  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  Serial.print("+-------------------------------------------------------------------------+\n");
 }
/* +---------------------------------------------------+
 * | 4A) Ambient Sensors - Temperature & Humidity      |
 * +---------------------------------------------------+*/
void setupAmbientTemp() {
  Serial.println("| HTU21D-F: Test & Config...");
  if (!htu.begin()) {
    Serial.println("[] ERROR: Could not find the Temperature & Humidity sensor. Please check your wiring!");
    //while (1);
  } else {
    Serial.println("| HTU21D-F: All Clear! - Temp & Humidity Sensor found and calibrated."); 
  }
  Serial.print("+-------------------------------------------------------------------------+\n");
}
/* +---------------------------------------------------+
 * | 4B) Ambient Sensors - Luminosity                  |
 * +---------------------------------------------------+*/
void setupAmbientLux() {
  Serial.println("| TSL2561: Test & Config...");
  if(!tsl.begin()) {
    Serial.println("[] ERROR: Could not find the Luminosity Sensor. Please check your wiring!");
    //while (1);
  } else {
    tsl.setGain(TSL2561_GAIN_1X);                            /* Range is 1X to 16X */
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
    Serial.println("| TSL2561: All Clear! - Light Sensor found and calibrated.");
  }
  Serial.print("+-------------------------------------------------------------------------+\n");
}
/* +---------------------------------------------------+
 * | 4C) Ambient Sensors - Sound                       |
 * +---------------------------------------------------+*/
// Electret is wired to pin...
void setupAmbientSound() {
  // There is nothing for the sound sensor (Electret Amplifier MAX9814
  Serial.println("| ELectret: Test & Config...");
  Serial.println("| Electret: All Clear! No calibration needed.");
  Serial.print("+-------------------------------------------------------------------------+\n");
}
/* +---------------------------------------------------+
 * | 5) Road Quality - Accelerometer                   |
 * +---------------------------------------------------+*/
void setupRoadSensor() {
 Serial.println("| ADXL345: Test & Config...");
 if(!accel.begin())
  {
    Serial.println("[] ERROR: Could not find the ADXL345. Please check your wiring!");
 //   while(1);
  } else {
    accel.setRange(ADXL345_RANGE_16_G);
    Serial.println("| ADXL345: All Clear! - ADXL345 found and calibrated.");
  }
  Serial.print("+-------------------------------------------------------------------------+\n");
}
