#include <GPS_Nav.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_LSM303_U.h>  // mag+accel
#include <Adafruit_Sensor.h>    // sensor objects
#include <Wire.h>               // including this prevents a compilation error
#include <compass.h>
#include <porto_const.h>

//#define DEBUGGING
#define USE_SOFTWARE_SERIAL_FOR_GPS
#define GPS_ECHO true

// Initialize the compass

// Assign a unique ID to this sensor at the same time 
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Set up pins for the LEDs
#define latchPin 5
#define clockPin 6
#define dataPin 4

#define brightnessPin 11

#define delay_ms 5

#define GPS_DELAY 5000
#define DISPLAY_DELAY 5

// Constants for direction and range
byte leds;
 

void updateShiftRegister()
{
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, LSBFIRST, leds);
   digitalWrite(latchPin, HIGH);
}


// initialize GPS stuff

const char NORTH = 'N';
const char SOUTH = 'S';
const char EAST = 'E';
const char WEST = 'W';

// uncomment below for software serial
#ifdef USE_SOFTWARE_SERIAL_FOR_GPS
SoftwareSerial gps_serial(9, 10);
Adafruit_GPS GPS(&gps_serial);
#else
Adafruit_GPS GPS(&Serial1);
HardwareSerial gps_serial = Serial1;
#endif

// timer for the gps update
uint32_t gps_timer = millis()-2000;
// timer for the compass update
uint32_t compass_timer = millis()-2000;

float heading_to_porto;

const int FIX_LED = 11;
const int QUALITY_LED = 12;

gps_results gps_info;

void gps_setup() {

  #ifdef DEBUGGING
  while(!Serial);
  Serial.println("Starting debugging");
  #endif

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // this is for recommended minimum (RMC) and System Fix Data(GGA)
  // I'm not currently using anything that's in RMC that's not in GGA, with the possible
  // exception of 'fix', and I figure I might as well use quality as a proxy for that.

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  gps_info.has_fixed = false;
}


void setup() {

  // put your setup code here, to run once:
  gps_setup();


  // for compass lights
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);

  /* Enable auto-gain */
  //mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    //eventually, output a looping error blinker here
  }

  //setup the info pins
  pinMode(FIX_LED, OUTPUT);
  pinMode(QUALITY_LED, OUTPUT);
}

void parseGPS(void)
{
  #ifdef DEBUGGING
  Serial.println("attempting to parse GPS");
  char c;
  #endif
  while (!GPS.newNMEAreceived()) {
    // TODO add a timeout
    #ifdef DEBUGGING
    c = GPS.read();
    if (GPS_ECHO) {
      Serial.print(c);
    }
    #else
    GPS.read();
    #endif

  }
  // if a sentence is received, we can check the checksum, parse it...
   
  if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
    gps_info.parsed = false;  // we can fail to parse a sentence in which case we should just wait for another
    #ifdef DEBUGGING
    Serial.println("GPS parsing failure");
    Serial.println(GPS.lastNMEA());
    #endif
    return; // This will keep the last result in the gps info struct
  }
  else {
    #ifdef DEBUGGING
    Serial.println("GPS parsed");
    Serial.println(GPS.lastNMEA());
    #endif
    gps_info.parsed = true;
  }

  // next check for fix and quality
  gps_info.fix = GPS.fix;
  gps_info.quality = GPS.fixquality;

  if (!gps_info.fix && !gps_info.quality) {

    #ifdef DEBUGGING
    Serial.println("No GPS fix");
    #endif

    return;
  }

  else {
    #ifdef DEBUGGING
    Serial.println("Setting has fixed to True");
    #endif
    gps_info.has_fixed = true;
  }

  // These might not really need to be doubles
  double latitude;
  double longitude;

  if (GPS.lat == SOUTH)
    latitude = -convertDegMinToDecDeg(GPS.latitude);
  else
    latitude = convertDegMinToDecDeg(GPS.latitude);

  if (GPS.lon == WEST)
    longitude = -convertDegMinToDecDeg(GPS.longitude);
  else
    longitude = convertDegMinToDecDeg(GPS.longitude);

  

  #ifdef DEBUGGING
  if (latitude == 0.0 || longitude == 0.0) {
    Serial.println("Latitude or longitude is being set to 0.0, this shouldn't happen.");
  }  
  #endif

  gps_info.position = {latitude, longitude};
}


float updateGPS(void) {

  // Parse the GPS results
  parseGPS();

  closest_point_result closest_porto;
  float heading_to_porto;

  closest_porto = getClosestPointSpherical(gps_info.position, portos, portos_count);
  //closest_porto = getClosestPointSpherical(gps_info.position, TEST_EAST, 1);

  heading_to_porto = getBearingSpherical(gps_info.position, closest_porto.position);

  #ifdef DEBUGGING
  Serial.println("GPS info");
  Serial.print("\t Quality: ");
  Serial.println(gps_info.quality);
  Serial.print("\t Fix: ");
  Serial.println(gps_info.fix);
  Serial.print("\t Position: ");
  Serial.print(gps_info.position.latitude, 15);
  Serial.print(", ");
  Serial.println(gps_info.position.longitude, 15);

  Serial.print("Destination position: ");
  Serial.print(closest_porto.position.latitude, 15);
  Serial.print(", ");
  Serial.println(closest_porto.position.longitude, 15);
  Serial.print("Destination distance (km):");
  Serial.println(closest_porto.distance);

  Serial.println();
  #endif

  return heading_to_porto;
}


void updateBearingLeds(float heading_to_porto) {
  float relative_heading;
  float heading;

  // TODO make this decay
  if (gps_info.has_fixed) {     

    /* Get a new magnetometer sensor event */ 
    sensors_event_t event; 
    mag.getEvent(&event);

    // Calculate the angle of the vector y,x
    heading = atan2(event.magnetic.x,event.magnetic.y); // RADIANS

    // Get the relative heading to the porto
    relative_heading = heading_to_porto - heading;

    #ifdef DEBUGGING

    Serial.print("mag heading: ");
    Serial.println(heading);
    Serial.print("mag Heading(degrees): ");
    Serial.println(degrees(heading));
    Serial.print("mag Heading(rad*pi): ");
    Serial.print(heading/M_PI);
    Serial.println("*Pi");

    Serial.print("Heading to porto: ");
    Serial.println(heading_to_porto);
    Serial.print("Heading to porto(degrees): ");
    Serial.println(degrees(heading_to_porto));
    Serial.print("Heading to porto(rad*pi): ");
    Serial.print(heading_to_porto/M_PI);
    Serial.println("*Pi");

    Serial.print("Relative heading: ");
    Serial.println(relative_heading);
    Serial.print("Relative heading(degrees):");
    Serial.println(degrees(relative_heading));
    Serial.print("Relative Heading(rad*pi): ");
    Serial.print(relative_heading/M_PI);
    Serial.println("*Pi");

    #ifdef DEBUGGING
    Serial.println();
    Serial.println();
    #endif

    #endif
  }

  // actually update the LEDs

  leds = 0;
  bitSet(leds, 7);
  if (gps_info.has_fixed)
    setDirectionByte(leds, relative_heading);

  
}

void fadeOut(void) {
  for (int i=255;i>0;i--){
    setBrightness(i);
    delay(delay_ms);
  }
}

void fadeIn(void) {
   for (int i=0;i<255;i++){
    setBrightness(i);
    delay(delay_ms);
  }
}

void setBrightness(byte brightness) // 0 to 255
{
  analogWrite(brightnessPin, 255-brightness);
}


void loop() 
{
  //update the gps info
  if (millis() - gps_timer > GPS_DELAY) {
    fadeOut();
    heading_to_porto = updateGPS();
    gps_timer = millis();
    fadeIn();
  }

  //update the display
  if (millis() - compass_timer > DISPLAY_DELAY) 
  { 

    updateBearingLeds(heading_to_porto);
    updateShiftRegister();
    compass_timer = millis(); // reset the timer

  }
  //updateShiftRegister();

  //delay(0);
}


