#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
SoftwareSerial ss(5,6); //RX, TX

void gps_data();
float gps_path_angle();
double distance_between_waypoint();
static void smartDelay(unsigned long ms);
const int GPS_BAUD = 9600;
float DEST_LAT = 12.917113, DEST_LON = 77.575290;
float Source_LAT = 12.917103, Source_LON = 77.575293;


void setup()
{

  Serial.begin(GPS_BAUD);
  ss.begin(GPS_BAUD);
  Serial.println("Scanning for satellites...");
}

void loop() 
{
  gps_data();
  if(gps.location.isUpdated())
  { 
    Serial.print("Satellite count: ");  
    Serial.println(gps.satellites.value(), gps.satellites.isValid());
    Serial.print("Latitude: ");  
    Serial.println(gps.location.lat(),6);
    Serial.print("Longitude: ");  
    Serial.println(gps.location.lng(),6);
    Serial.print("Distance: ");
    Serial.println(TinyGPSPlus::distanceBetween(Source_LAT, 
      Source_LON,
      DEST_LAT, 
      DEST_LON));
    gps_path_angle();
    distance_between_waypoint();
    Serial.println("");
    delay(1000);
  }
}

void gps_data()
{
  while(ss.available())
    gps.encode(ss.read());
}

float gps_path_angle()
{
      double path_angle = TinyGPSPlus::courseTo(gps.location.lat(),
      gps.location.lng(),
      DEST_LAT,
      DEST_LON);
      Serial.print("path angle: ");
      Serial.println(path_angle);
      return path_angle;
}

double distance_between_waypoint()
{
      double path_distance = TinyGPSPlus::distanceBetween(gps.location.lat(),
      gps.location.lng(),
      DEST_LAT, 
      DEST_LON);
      Serial.print("path distance: ");
      Serial.println(path_distance);
      return path_distance;
}
