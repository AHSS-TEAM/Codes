#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MechaQMC5883.h>

#define FORWARD_VEL     140
#define SLOW_TURN_VEL   100
#define PIVOT_WHEEL_VEL 40

TinyGPSPlus gps;
MechaQMC5883 magnetometer;
SoftwareSerial ss(5, 6); //TX,RX

#define MAX_HEADING_ANGLE  180
#define MIN_HEADING_ANGLE  35

#define DIST_TOLERANCE 1

const int GPS_BAUD = 9600;
//float Source_LAT = 12.917147, Source_LON = 77.575347;
float DEST_LAT = 12.917130, DEST_LON = 77.575317;
float current_lat, current_lon;
const float mag_declination = -1.23;
int x, y, z, azimuth;
float path_distance, path_heading, current_heading, compass_heading, heading_error;

int left_motor_pin1 = 10; // IN1 to the driver
int left_motor_pin2 = 11; // IN2 to the driver
int right_motor_pin1 = 2; // IN3 to the driver
int right_motor_pin2 = 3; // IN4 to the driver

void Forward(unsigned char vel);
void Turn_Left(unsigned char vel);
void Turn_Right(unsigned char vel);
void Stop(void);

void setup()
{
  pinMode (left_motor_pin1, OUTPUT);
  pinMode (left_motor_pin2, OUTPUT);
  pinMode (right_motor_pin1, OUTPUT);
  pinMode (right_motor_pin2, OUTPUT);

  Stop();

  Wire.begin();
  Serial.begin(GPS_BAUD);
  ss.begin(GPS_BAUD);
  magnetometer.init();
  magnetometer.setMode(Mode_Continuous, ODR_200Hz, RNG_2G, OSR_256);
  Serial.println("Scanning for satellites......");
  Serial.println("Latitude, Longitude, Target distance, Target heading, Current heading, Error, status");
}

void loop()
{
  current_heading = compass_data();
  //Serial.println(current_heading);
  //delay(1000);
  gps_data();
  if (gps.location.isUpdated())
  {
    //Serial.print("Current latitude: ");
    Serial.print(gps.location.lat(), 6);
    delay(100);
    //Serial.print("Current longitude: ");
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
    delay(100);
    navigation();
    control_navigation(path_heading, current_heading);
    Serial.println("");
    delay(100);
  }
}

void gps_data()
{
  while (ss.available())
  {
    gps.encode(ss.read());
  }
}

int compass_data()
{
  magnetometer.read(&x, &y, &z, &compass_heading);
  compass_heading = magnetometer.azimuth(&y, &x);
  compass_heading = compass_heading - mag_declination;
  return compass_heading;
}

void navigation()
{
  gps_data();
  path_distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),  DEST_LAT, DEST_LON);
  //Serial.print("Taraget distance: ");
  Serial.print(",");
  Serial.print(path_distance); delay(100);

  if (path_distance < DIST_TOLERANCE)
  {
    Serial.println("stop");
    Stop(); //stops bot since it reached the point
    delay(5000);
    //waypoint++;
  }
  gps_data();
  path_heading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), DEST_LAT, DEST_LON);
  //Serial.print("Taraget heading: ");
  Serial.print(",");
  Serial.print(path_heading); delay(100);

  //Serial.print("current heading: ");
  Serial.print(",");
  Serial.print(current_heading); delay(100);
}

void control_navigation(float target_heading, float present_heading)
{
  heading_error = target_heading - current_heading;
  if (heading_error < -180)
  {
    heading_error = heading_error + 360;
  }
  if (heading_error > 180)
  {
    heading_error = heading_error - 360;
  }

  //Serial.print("Error: ");
  Serial.print(",");

  Serial.print(heading_error);
  delay(1000);

  // The error is between -10 and +10 (for ANGLE_RANGE_DIV = 0.25):
  if (heading_error >= -25 && heading_error <= 15 )
  {
    // Forward
    Forward(FORWARD_VEL);
  }

  // The error is between +10 and +180 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error > 15 && heading_error <= 180)
  {
    // Turn right
    Turn_Right(SLOW_TURN_VEL);
    //Forward(FORWARD_VEL);
  }

  // The error is between -10 and -180 (for ANGLE_RANGE_DIV = 0.25):
  else if (heading_error < -25 && heading_error >= -180)
  {
    // Turn left
    Turn_Left(SLOW_TURN_VEL);
    //Forward(FORWARD_VEL);
  }
  
  else
  {
    Serial.println("jai");
  }
}

/* Moves Forward */
void Forward(unsigned char vel)
{
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, vel);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, vel);
}

/* Turns the car left slowly */
void Turn_Left(unsigned char vel)
{
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, vel);
  analogWrite (left_motor_pin1, 0);
  analogWrite (left_motor_pin2, PIVOT_WHEEL_VEL);
}

/* Turns the car right slowly */
void Turn_Right(unsigned char vel)
{
  analogWrite (right_motor_pin1, 0) ;
  analogWrite (right_motor_pin2, PIVOT_WHEEL_VEL);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, vel);
}

/* Stops the car */
void Stop(void)
{
  analogWrite (right_motor_pin1, LOW);
  analogWrite (right_motor_pin2, LOW);
  analogWrite (left_motor_pin1, LOW);
  analogWrite (left_motor_pin2, LOW);
}
