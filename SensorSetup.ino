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
  if (!SD.exists("m_log.csv")) {
    // Create file
    dataFile_Master = SD.open("m_log.csv", FILE_WRITE);
    // Add header row
    dataFile_Master.println("ID,DATESTAMP,TIMESTAMP,GPS_LAT,GPS_LON,GPS_Speed,GPS_SpeedMin,GPS_SpeedMax,GPS_SpeedMea,GPS_SpeedMed,GPS_Alt,GPS_Sats,GPS_Fix,GPS_Quality,AMB_Temp,AMB_TempMin,AMB_TempMax,AMB_TempMea,AMB_TempMed,AMB_Humd,AMB_HumdMin,AMB_HumdMax,AMB_HumdMea,AMB_HumdMed,AMB_Lux,AMB_LuxMin,AMB_LuxMax,AMB_LuxMea,AMB_LuxMed,AMB_Snd,AMB_SndMin,AMB_SndMax,AMB_SndMea,AMB_SndMed,RDQ_AcX,RDQ_AcXMin,RDQ_AcXMax,RDQ_AcXMea,RDQ_AcXMed,RDQ_AcY,RDQ_AcYMin,RDQ_AcYMax,RDQ_AcYMea,RDQ_AcYMed,RDQ_AcZ,RDQ_AcZMin,RDQ_AcZMax,RDQ_AcZMea,RDQ_AcZMed");
  }
  if (!SD.exists("t_log.csv")) {
    // Create file
    dataFile_Temp = SD.open("t_log.csv", FILE_WRITE);
    // Add header row
    dataFile_Temp.println("ID,DATESTAMP,TIMESTAMP,GPS_LAT,GPS_LON,GPS_Speed,GPS_SpeedMin,GPS_SpeedMax,GPS_SpeedMea,GPS_SpeedMed,GPS_Alt,GPS_Sats,GPS_Fix,GPS_Quality,AMB_Temp,AMB_TempMin,AMB_TempMax,AMB_TempMea,AMB_TempMed,AMB_Humd,AMB_HumdMin,AMB_HumdMax,AMB_HumdMea,AMB_HumdMed,AMB_Lux,AMB_LuxMin,AMB_LuxMax,AMB_LuxMea,AMB_LuxMed,AMB_Snd,AMB_SndMin,AMB_SndMax,AMB_SndMea,AMB_SndMed,RDQ_AcX,RDQ_AcXMin,RDQ_AcXMax,RDQ_AcXMea,RDQ_AcXMed,RDQ_AcY,RDQ_AcYMin,RDQ_AcYMax,RDQ_AcYMea,RDQ_AcYMed,RDQ_AcZ,RDQ_AcZMin,RDQ_AcZMax,RDQ_AcZMea,RDQ_AcZMed");
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
    accel.setRange(ADXL345_RANGE_8_G);
    Serial.println("| ADXL345: All Clear! - ADXL345 found and calibrated.");
  }
  Serial.print("+-------------------------------------------------------------------------+\n");
}


// FONA SETUP - IN HEAVY DEV
//void setupComms() {
//  pinMode(FONA_PS, INPUT);
//  pinMode(FONA_KEY,OUTPUT);   
//  digitalWrite(FONA_KEY, HIGH);
//  Serial.println("Started setup");
//  fonaSS.begin(9600);
//    /*=============Get Location and Post ONCE on Boot=============*/
//    /*
//    turnOnFONA(); //turn on FONA
//    Serial.println("Initializing: please wait 10 sec...");
//    delay(10000);
//    setupGPRS(); //Setup a GPRS context
//    if(getLocation()) { //try to get location, if OK do the rest
//        //Print Lat/Lon Values
//        Serial.print(Lat);
//        Serial.print(" : ");
//        Serial.print(Lon);
//        Serial.print(" at ");
//        Serial.print(Time);
//        Serial.print(" ");
//        Serial.print(Date);
//        Serial.println();
//        //Move data into data array. (note: this is an extra step, but helps the legibility of what is going on)
//        fieldData[0] = Lat; 
//        fieldData[1] = Lon;
//        fieldData[2] = Date;
//        fieldData[3] = Time;
//    }
//    makeRequest(); //Make GET request
//    turnOffFONA(); //turn off FONA
//    flushFONA();
//    */
//    Serial.println("Setup DONE!");
//}
//
//boolean sendATCommand(char Command[]) { //Send an AT command and wait for a response
//    int complete = 0; // have we collected the whole response?
//    char c; //capture serial stream
//    String content; //place to save serial stream
//    unsigned long commandClock = millis(); //timeout Clock
//    fonaSS.println(Command); //Print Command
//    while(!complete && commandClock <= millis() + ATtimeOut) { //wait until the command is complete
//        while(!fonaSS.available() && commandClock <= millis()+ATtimeOut); //wait until the Serial Port is opened
//        while(fonaSS.available()) { //Collect the response
//            c = fonaSS.read(); //capture it
//            if(c == 0x0A || c == 0x0D); //disregard all new lines and carrige returns (makes the String matching eaiser to do)
//            else content.concat(c); //concatonate the stream into a String
//        }
//        //Serial.println(content); //Debug
//        response = content; //Save it out to a global Variable (How do you return a String from a Function?)
//        complete = 1;  //Lable as Done.
//    }
//    if (complete ==1) return 1; //Is it done? return a 1
//    else return 0; //otherwise don't (this will trigger if the command times out) 
//    /*
//        Note: This function may not work perfectly...but it works pretty well. I'm not totally sure how well the timeout function works. It'll be worth testing. 
//        Another bug is that if you send a command that returns with two responses, an OK, and then something else, it will ignore the something else and just say DONE as soon as the first response happens. 
//        For example, HTTPACTION=0, returns with an OK when it's intiialized, then a second response when the action is complete. OR HTTPREAD does the same. That is poorly handled here, hence all the delays up above. 
//    */
//}
