#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MechaQMC5883.h>
void motorspeed( int choose_error);

TinyGPSPlus gps;
MechaQMC5883 magnetometer;
SoftwareSerial ss(5, 6); //TX,rx

const int GPS_BAUD = 9600;
float DEST_LAT = 12.917113, DEST_LON = 77.575290;
float Source_LAT = 12.917103, Source_LON = 77.575293;
int x, y, z, azimuth;
int direction2waypoint, distance2waypoint, compass_heading;
int path_distance, path_heading, current_heading, heading_difference,TurningError;

void setup()
{
    Wire.begin();

  Serial.begin(GPS_BAUD);
  ss.begin(GPS_BAUD);
  magnetometer.init();
  magnetometer.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  Serial.println("Scanning for satellites...");
}

void loop()
{
  gps_data();
  if (gps.location.isUpdated())
  {
    Serial.print("Satellite count: ");
    Serial.println(gps.satellites.value(), gps.satellites.isValid());
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Distance: ");
    Serial.println(TinyGPSPlus::distanceBetween(Source_LAT, Source_LON, DEST_LAT, DEST_LON));
    move_to_waypoint();
    delay(2000);
    Serial.println("");
  }
}

void gps_data()
{
  while (ss.available())
    gps.encode(ss.read());
}

int direction_to_waypoint()
{
  int direction2waypoint = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), DEST_LAT, DEST_LON);
  //Serial.print("Target direction: ");
  //Serial.println(direction2waypoint);
  return (direction2waypoint);
}

int distance_to_waypoint()
{
  int distance2waypoint = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), DEST_LAT, DEST_LON);
  //Serial.print("Target distance: ");
  //Serial.println(distance2waypoint);
  return (distance2waypoint);
}

int compass_data()
{
  magnetometer.read(&x, &y, &z, &azimuth);
  int compass_heading = magnetometer.azimuth(&y,&x);
  return compass_heading;
}

int desired_turn(int gps_heading, int comp_heading)
{
  int heading_error = gps_heading - comp_heading;
  heading_error = heading_error;
  if (heading_error < -180)
    heading_error += 360;
  if (heading_error > 180)
    heading_error -= 360;
  

  return (heading_error);
}

int move_to_waypoint()
{
  path_distance = distance_to_waypoint();
  Serial.print("path_distance: ");
  Serial.println(path_distance);
  if (path_distance <= 2)
  {
    idle();
    delay(2000);
  }
  
  path_heading = direction_to_waypoint();
  Serial.print("path_heading: ");
  Serial.println(path_heading);

  current_heading = compass_data();
  Serial.print("compass_heading: ");
  Serial.println(current_heading);

  heading_difference = desired_turn(path_heading, current_heading);
  Serial.print("Heading difference: ");
  Serial.println(heading_difference);

  TurningError = heading_difference;
  if (TurningError >= 5 &&  TurningError <= 10) {
    /*analogWrite(motorLpwm, 180);
    analogWrite(motorRpwm, 50);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("left turn");
  }
  else if (TurningError >= 11 &&  TurningError <= 60) {
    /*analogWrite(motorLpwm, 255);
    analogWrite(motorRpwm, 50);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("left turn1");
  }


  else if (TurningError >= -10 &&  TurningError <= -5) {
    /*analogWrite(motorLpwm, 50);
    analogWrite(motorRpwm, 180);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("right turn");
  }
  else if (TurningError >= -60 &&  TurningError <= -11 ) {
    /*analogWrite(motorLpwm, 50);
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("right turn1");
  }


  else if (TurningError <= -60) {
    /*analogWrite(motorLpwm, 255); //anticlockwise
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 0);
    digitalWrite(motorLpin2, 1);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("right turn2");
  }
  else if (TurningError >= 60) {
    /*analogWrite(motorLpwm, 255); //clockwise
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 0);
    digitalWrite(motorRpin2, 1);
    delay(1000);*/
    Serial.println("left turn3");
  }

  else if (TurningError < 5 && TurningError > -5)
  { //go forward
    /*analogWrite(motorLpwm, 255);
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    Serial.println("Left motor speed 255");
    Serial.println("Right motor speed 255");
    delay(1000);*/
    Serial.println("Forward");
  }
  
}


void motorSpeed( int choose_error) {
  int  TurningError = choose_error;

  if (TurningError >= 5 &&  TurningError <= 10) {
    /*analogWrite(motorLpwm, 180);
    analogWrite(motorRpwm, 50);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("left turn");
  }
  else if (TurningError >= 11 &&  TurningError <= 60) {
    /*analogWrite(motorLpwm, 255);
    analogWrite(motorRpwm, 50);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("left turn1");
  }


  else if (TurningError >= -10 &&  TurningError <= -5) {
    /*analogWrite(motorLpwm, 50);
    analogWrite(motorRpwm, 180);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("right turn");
  }
  else if (TurningError >= -60 &&  TurningError <= -11 ) {
    /*analogWrite(motorLpwm, 50);
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("right turn1");
  }


  else if (TurningError <= -60) {
    /*analogWrite(motorLpwm, 255); //anticlockwise
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 0);
    digitalWrite(motorLpin2, 1);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    delay(1000);*/
    Serial.println("right turn2");
  }
  else if (TurningError >= 60) {
    /*analogWrite(motorLpwm, 255); //clockwise
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 0);
    digitalWrite(motorRpin2, 1);
    delay(1000);*/
    Serial.println("left turn3");
  }

  else if (TurningError < 5 && TurningError > -5)
  { //go forward
    /*analogWrite(motorLpwm, 255);
    analogWrite(motorRpwm, 255);
    digitalWrite(motorLpin1, 1);
    digitalWrite(motorLpin2, 0);
    digitalWrite(motorRpin1, 1);
    digitalWrite(motorRpin2, 0);
    Serial.println("Left motor speed 255");
    Serial.println("Right motor speed 255");
    delay(1000);*/
    Serial.println("Forward");
  }

  }
  
  void idle() {
  /*analogWrite(motorLpwm, 0);
  analogWrite(motorRpwm, 0);
  digitalWrite(motorLpin1, 1);
  digitalWrite(motorLpin2, 1);
  digitalWrite(motorRpin1, 1);
  digitalWrite(motorRpin2, 1);*/
  Serial.println("Stop");
  }
