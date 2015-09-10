
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

//#include <CO_MQ7.h>

//CO_MQ7 MQ7(9, 13);  // 12 = digital Pin connected to "tog" from sensor board
                     // 13 = digital Pin connected to LED Power Indicator

int CoSensorOutput = 0; //analog Pin connected to "out" from sensor board
int CoData = 0;  


// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7
// If using hardware serial:
//   Connect the GPS TX (transmit) pin to Arduino RX1 (Digital 0)
//   Connect the GPS RX (receive) pin to matching TX1 (Digital 1)

//#define BUFFSIZE 90
#define led1Pin 2
#define led2Pin 3
//char buffer[BUFFSIZE];
//uint8_t bufferidx = 0;
//uint8_t i;

const int chipSelect = 10;
File dataFile;


int sensor1;
int sensor2;
int sensor3; 
int sensor4;
int coPin = A0;

int proxLeft;
int proxRight;

int topo;

int sound;
int tempPin = A6;
int light;

//const int xpin = A6;                  // x-axis of the accelerometer
const int ypin = A7;                  // y-axis
//const int zpin = A8;                  // z-axis (only on 3-axis models)

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(led1Pin, HIGH);
  
  while(1);
}

//// read a Hex value and return the decimal equivalent
//uint8_t parseHex(char c) {
//  if (c < '0')
//    return 0;
//  if (c <= '9')
//    return c - '0';
//  if (c < 'A')
//    return 0;
//  if (c <= 'F')
//    return (c - 'A')+10;
//}
//
//// blink out an error code
//void error(uint8_t errno) {
///*
//  if (SD.errorCode()) {
//    putstring("SD error: ");
//    Serial.print(card.errorCode(), HEX);
//    Serial.print(',');
//    Serial.println(card.errorData(), HEX);
//  }
//  */
//  while(1) {
//    for (i=0; i<errno; i++) {
//      digitalWrite(led1Pin, HIGH);
//      digitalWrite(led2Pin, HIGH);
//      delay(100);
//      digitalWrite(led1Pin, LOW);
//      digitalWrite(led2Pin, LOW);
//      delay(100);
//    }
//    for (; i<10; i++) {
//      delay(200);
//    }
//  }
//}

Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

void setup()
{
  
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  

   // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
//    while (!Serial){
//  ;    //wait for serial port to connect.  Needed for Leonardo only (micro runs like Leonardo)
//  }
   Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
//  Serial.println("card initialized.");
  
  // create new file
  char filename [] = "BOX02_00.csv";
  for (uint8_t i = 0; i<1000; i++){
    filename [6] = i/10 + '0';
    filename [7] = i%10 + '0';
    if (!SD.exists(filename)){
    //only open a new file if it doesn't exist
    dataFile = SD.open(filename, FILE_WRITE);
    break;
    }
  }

  if (!dataFile){
    error ("couldn't create file");
    }

//    Serial.println("BOXID, VERSION, MONTH, DAY, YEAR, , TIME, FIX, QUALITY, LAT, LATDIR, LON, LONDIR, SPEED, ANGLE, ALTITUDE, SATELLITES, TOPO, SENSE1, SENSE2, SENSE3, SENSE4, PROXLEFT, PROXRIGHT, TEMP, LIGHT, SOUND");

    dataFile.println("BOXID, VERSION, MONTH, DAY, YEAR, TIME, FIX, QUALITY, LAT, LATDIR, LON, LONDIR, SPEED, ANGLE, ALTITUDE, SATELLITES, TOPO, SENSE1, SENSE2, SENSE3, SENSE4, PROXLEFT, PROXRIGHT, TEMP, LIGHT, SOUND");

    
//    Serial.print("Logging to: ");
//    Serial.println(filename);

}

uint32_t timer = millis();
void loop()
{   
 
//  MQ7.CoPwrCycler();  

  
  sensor3 = analogRead(A1);
  sensor2 = analogRead(A2);
  sensor1 = analogRead(A3);       // read analog input pin 0
  
  proxLeft = analogRead(A4);
  proxRight = analogRead(A5);
  
  topo = analogRead(ypin);
  
  sound = analogRead(A8);
  light = analogRead(A11);
  
      int reading = analogRead(tempPin);
      float voltage = reading *4.7;
      voltage /= 1024.0;
      float temp = (voltage - 0.5) * 100;
  
  
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c); 
  
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
  if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer
  
/*
    Serial.print("BOX02");
    Serial.print(",");
    Serial.print("UMS_6_4");
    Serial.print(",");
    Serial.print(GPS.month, DEC);
    Serial.print(",");
    Serial.print(GPS.day, DEC); 
    Serial.print(",");    
    Serial.print(GPS.year, DEC);
    Serial.print(',');
    
    Serial.print(GPS.hour, DEC);    // time
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print(',');
    Serial.print((int)GPS.fix); Serial.print(',');                   // Fix
    Serial.print((int)GPS.fixquality); Serial.print(',');            //Fix Quality
      Serial.print(GPS.latitude, 4);
      Serial.print(", "); 
      Serial.print(GPS.lat);     //Lat
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4);
      Serial.print(", "); 
      Serial.print(GPS.lon);     //Lon
      Serial.print(", "); 
      Serial.print(GPS.speed); Serial.print(", ");                  //Speed
      Serial.print(GPS.angle); Serial.print(", ");                  //Angle
      Serial.print(GPS.altitude); Serial.print(", ");             //Altitude
      Serial.print((int)GPS.satellites);Serial.print(", ");       //Number of Sattelites
     // print the sensor values:
//  Serial.print(analogRead(xpin));
//  // print a tab between values:
//  Serial.print(",");
  Serial.print(topo);
  // print a tab between values:
//  Serial.print(",");
//  Serial.print(analogRead(zpin));
  Serial.print(",");
  Serial.print(sensor1);
  Serial.print (",");
  Serial.print(sensor2);
  Serial.print (",");
  Serial.print(sensor3);
  Serial.print(",");
    if(MQ7.CurrentState() == false){   //we are at 1.4v, read sensor data!
    CoData = analogRead(CoSensorOutput);
    Serial.print(CoData);
  }
  else{                            //sensor is at 5v, heating time
    Serial.print("9999");
  }  
  Serial.print (",");
  Serial.print(proxLeft);
  Serial.print (",");
  Serial.print(proxRight);
  Serial.print(",");
  Serial.print(temp);
  Serial.print(",");
  Serial.print(light);
  Serial.print(",");
  Serial.println(sound);
*/

    dataFile.print("BOX02");
    dataFile.print(",");
    dataFile.print("UMS_6_4_BOX2");
    dataFile.print(",");
    dataFile.print(GPS.month, DEC);
    dataFile.print(",");
    dataFile.print(GPS.day, DEC);    //date
    dataFile.print(",");
    dataFile.print(GPS.year, DEC);dataFile.print(',');
    dataFile.print(GPS.hour, DEC);     // time
    dataFile.print(':');
    dataFile.print(GPS.minute, DEC);
    dataFile.print(':');
    dataFile.print(GPS.seconds, DEC);    
    dataFile.print(',');
    dataFile.print((int)GPS.fix); dataFile.print(',');                   // Fix
    dataFile.print((int)GPS.fixquality); dataFile.print(',');            //Fix Quality
      dataFile.print(GPS.latitude, 4);
      dataFile.print(",");
      dataFile.print(GPS.lat);     //Lat
      dataFile.print(", "); 
      dataFile.print(GPS.longitude, 4); 
      dataFile.print(",");
      dataFile.print(GPS.lon);     //Lon
      dataFile.print(", "); 
      dataFile.print(GPS.speed); dataFile.print(", ");                  //Speed
      dataFile.print(GPS.angle); dataFile.print(", ");                  //Angle
      dataFile.print(GPS.altitude); dataFile.print(", ");             //Altitude
      dataFile.print((int)GPS.satellites);dataFile.print(", ");       //Number of Sattelites
     // print the sensor values:
//  dataFile.print(analogRead(xpin));
//  // print a tab between values:
//  dataFile.print(",");
  dataFile.print(topo);
  // print a tab between values:
//  dataFile.print(",");
//  dataFile.print(analogRead(zpin));
  dataFile.print(",");
  dataFile.print(sensor1);
  dataFile.print (",");
  dataFile.print(sensor2);
  dataFile.print (",");
  dataFile.print(sensor3);
  dataFile.print (",");
  
//      if(MQ7.CurrentState() == false){   //we are at 1.4v, read sensor data!
    CoData = analogRead(CoSensorOutput);
    dataFile.print(CoData);
      }
  else{                            //sensor is at 5v, heating time
    dataFile.print("9999");
  }
  dataFile.print (",");
  dataFile.print(proxLeft);
  dataFile.print (",");
  dataFile.print(proxRight);
  dataFile.print(",");
  dataFile.print(temp);
  dataFile.print(",");
  dataFile.print(light);
  dataFile.print(",");
  dataFile.println(sound);
  }
  
//    dataFile.flush();
//    delay(1000);
