// Libraries
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <MechaQMC5883.h>

#define TOLERANCE_RADIUS 1.0  // Tolerance radius for reaching the goal

// Robot car velocities. Change if you want to adjust the car's behavior
// (range of allowed velocities: 0 ~ 255)
#define FORWARD_VEL     230
#define SLOW_TURN_VEL   220
#define PIVOT_WHEEL_VEL 50

// Constants for defining error ranges for the stepped proportional control
#define MAX_HEADING_ANGLE  180
#define MIN_HEADING_ANGLE  15
#define ANGLE_RANGE_DIV 0.25

// Calibration constants for the motors in differential drive config.
// e. g. if the left motor is slower that the right one, you can add some
// % to the left and reduce the same % to the right, and viceversa.
#define K_RIGHT_MOTOR 1.0
#define K_LEFT_MOTOR 1.0

// Objects for interfacing with sensors

TinyGPS gps;      // Create gps object
SoftwareSerial serial_gps(5, 6); // RX and TX pins in use
MechaQMC5883 magnetometer;

// Stores the next action the robot car should take (just for debugging)
String str_action;

// Control signal pins for the L298N the motor driver
int left_motor_pin1 = 10; // IN1 to the driver
int left_motor_pin2 = 11; // IN2 to the driver
int right_motor_pin1 = 2; // IN3 to the driver
int right_motor_pin2 = 3; // IN4 to the driver

//------- Globals for storing waypoints
// Struct to store waypoints (GPS coordinates):
struct t_waypoint {
  float lat;
  float lon;
};

// Place here your own waypoints, or you'll have your robot car trying tof
// reach some place in my hometown! You can use Google Maps to define them:

//waypoints {lat, lon}
t_waypoint nav_waypoints[] =  {
  { 12.917023, 77.52587},   // Point 1
  { 12.917027, 77.53400},  // Point 2
};

byte waypoint_index = 0;  // Index to the current waypoint in array
// Get the number of waypoints:
int num_waypoints = sizeof(nav_waypoints) / sizeof(nav_waypoints[0]);

// Auxiliary variables for computing the navigation vector:
float waypoint_lat_rad;
float waypoint_lon_rad;

//------- Globals for compass readings
int16_t x, y, z;   // Earth's magnetic field components
float compass_angle, compass_heading;  

// Stores the angle retrieved from the compass

// Magnetic declination in my current area/city. Change it for the value
// defined for your area/city or leave it at zero:
const float mag_declination = -1.23;

int heading_error; // Stores the heading error

//------- Globals for GPS readings
float f_gps_reading_lat, f_gps_reading_lon; // Stores the reading from the GPS receiver
float waypoint_angle; // Stores the azimuth to the current goal
float last_calc_dist = 0; // Las computed distance to goal (just for debugging)

// Auxiliaries for dumping GPS data
long lat, lon;
unsigned long age, date, time, chars;
unsigned short sentences, failed;

//------- Globals for the moving average filter
// Increse NUM_FILTERING_POINTS if you want more filtering of the GPS readings,
// it also adds more delay to the system. Sometimes less could be better.
#define NUM_FILTERING_POINTS  15
float buffer_gps_lat[NUM_FILTERING_POINTS]; // Buffer for latitudes
float buffer_gps_lon[NUM_FILTERING_POINTS]; // Buffer for longitudes
t_waypoint gps_filtered; // For storing a filtered GPS reading
unsigned int buffer_fill_index = 0; // Index for filling filter buffers

/* Setup function */
void setup() {
  // Initialize control pins for the motor driver
  pinMode (left_motor_pin1, OUTPUT);
  pinMode (left_motor_pin2, OUTPUT);
  pinMode (right_motor_pin1, OUTPUT);
  pinMode (right_motor_pin2, OUTPUT);

  Stop(); // Stop the car

  Wire.begin();           // Initialize I2C comm. for the compass
  Serial.begin(9600);     // Initialize serial comm. for debugging
  serial_gps.begin(9600);
  magnetometer.init();
  magnetometer.setMode(Mode_Continuous, ODR_200Hz, RNG_2G, OSR_256);
  Serial.println("Scanning for satellites......");
  Serial.println("Latitude, Longitude, Target distance, Target heading, Current heading, Error, status");
}

/* Main loop */
void loop() {

  // Uncomment the following line and comment the rest in this function if
  // you want to verify the rotation direction of your motors:
  //  Forward(FORWARD_VEL);

//  compass_heading = compass_data();  // Read the digital compass
//  Serial.println(compass_heading);
//  delay(1000);
  compass_data();
  if (Query_Gps()) {     // Query the GPS
    Gps_Dump(gps);        // Read the GPS

    // Store the new GPS reading in filter buffers
    Store_Gps_Reading(f_gps_reading_lat, f_gps_reading_lon);

    // Get filtered GPS latitude and longitude
    gps_filtered = Compute_Filtered_Gps();
    Serial.print(gps_filtered.lat);
    Serial.print(",");
    Serial.print(gps_filtered.lon);

    // Compute distance and heading to the goal
    Compute_Navigation_Vector(gps_filtered.lat, gps_filtered.lon);

    // Print data to the PC just for debugging
    Print_Data();
    Control_Navigation();
  }
}

/* Stores a GPS reading in the moving average filter's buffers */
void Store_Gps_Reading(float lat, float lon)
{
  // Shift all buffer values towards the tail
  for (int i = (NUM_FILTERING_POINTS - 1); i > 0; --i)
  {
    buffer_gps_lat[i] = buffer_gps_lat[i - 1];
    buffer_gps_lon[i] = buffer_gps_lon[i - 1];
  }

  // Insert new values at the head
  buffer_gps_lat[0] = lat;
  buffer_gps_lon[0] = lon;

  // Increment the number of readings stored in buffers
  if (buffer_fill_index < NUM_FILTERING_POINTS) {
    ++buffer_fill_index;
  }
}

/* Computes filtered latitude and longitude using a moving average filter */
t_waypoint Compute_Filtered_Gps()
{
  float lat_sum;
  float lon_sum;

  t_waypoint filtered_waypoint;

  lat_sum = 0;
  lon_sum = 0;

  // Add all values in each buffer
  for (int i = 0; i < buffer_fill_index; ++i) {
    lat_sum = lat_sum + buffer_gps_lat[i];
    lon_sum = lon_sum + buffer_gps_lon[i];
  }

  // Take the average
  filtered_waypoint.lat = lat_sum / float(buffer_fill_index);
  filtered_waypoint.lon = lon_sum / buffer_fill_index;

  return filtered_waypoint; // Return filtered values
  }

/* Queries the GPS receiver */
bool Query_Gps()
{
  while (serial_gps.available())
  {
    if (gps.encode(serial_gps.read())) {
      return true;
    }
  }
  return false;
}

int compass_data()
{
  magnetometer.read(&x, &y, &z, &compass_angle);
  compass_angle = magnetometer.azimuth(&y, &x);
  compass_angle = compass_angle - mag_declination;
  return compass_angle;
}

/* Computes the navigation vector */
void Compute_Navigation_Vector(float gps_lat, float gps_lon) {
  t_waypoint cur_waypoint;

  // Get current goal
  cur_waypoint = Get_Waypoint_With_Index(waypoint_index);

  float gps_f_lat_rad;
  float gps_f_lon_rad;
  float a_haversine = 0;
  float c_haversine = 0;
  float d_haversine = 0;
  float parcial = 0;
  float delta_lat = 0;
  float delta_lon = 0;

  // Compute the distance to the goal with Haversine formula
  // *******************************************************
  delta_lat = radians(cur_waypoint.lat - gps_lat);
  gps_f_lat_rad = radians(gps_lat);
  waypoint_lat_rad = radians(cur_waypoint.lat);
  delta_lon = radians(cur_waypoint.lon - gps_lon);

  a_haversine = sin(delta_lat / 2.0) * sin(delta_lat / 2.0);
  parcial = cos(gps_f_lat_rad) * cos(waypoint_lat_rad);
  parcial = parcial * sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
  a_haversine += parcial;

  c_haversine = 2 * atan2(sqrt(a_haversine), sqrt(1.0 - a_haversine));

  d_haversine = 6371000.0 * c_haversine; // Multiply by Earth's radius in meters

  last_calc_dist = d_haversine;
  Serial.print(",");
  Serial.print(last_calc_dist); delay(100);

  

  // Check if we are inside the goal's tolerance radius
  if (d_haversine < TOLERANCE_RADIUS) 
  {
    Stop();       // Stop the car
    delay(3000);  // Delay just to check visually were exactly the car reached the goal
    waypoint_index++; // Switch to the next waypoint
  }
  
  // Check if we reached all waypoints
  if (waypoint_index == num_waypoints) {
    Stop(); // Stop the car
    while (1); // Stop the program (reset Arduino board to repeat)
  }
  
  // Compute the forward azimuth
  // **************************************************
  gps_f_lon_rad = radians(gps_lon);
  waypoint_lon_rad = radians(cur_waypoint.lon);

  waypoint_angle = atan2(sin(waypoint_lon_rad - gps_f_lon_rad) * cos(waypoint_lat_rad),
                         cos(gps_f_lat_rad) * sin(waypoint_lat_rad) - sin(gps_f_lat_rad) * cos(waypoint_lat_rad) * cos(waypoint_lon_rad - gps_f_lon_rad));

  waypoint_angle = waypoint_angle * 180 / PI; // Convert from radians to degrees
  Serial.print(",");
  Serial.print( waypoint_angle); delay(100);
  compass_data();
  Serial.print(",");
  Serial.print(compass_angle); delay(100);
  

  // Always convert to positive angles
  if (waypoint_angle < 0) {
    waypoint_angle += 360;
  }
}

/* Controls the robot car navigation towards the goal */
void Control_Navigation() {
  heading_error = (waypoint_angle - compass_angle); // Compute the heading error

  // Correct angle for wrap around
  if (heading_error < -180) {
    heading_error = heading_error + 360;
  }
  if (heading_error > 180) {
    heading_error = heading_error - 360;
  }
  Serial.print(",");
  Serial.print( heading_error); delay(100);
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

/*  TinyGPS auxiliary routine, comes with the library */
void Gps_Dump(TinyGPS &gps)
{
  gps.get_position(&lat, &lon, &age);

  //  Query_Gps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&f_gps_reading_lat, &f_gps_reading_lon, &age);

  Query_Gps();

  gps.stats(&chars, &sentences, &failed);
  //  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}


/* Returns a waypoint from the array */
struct t_waypoint Get_Waypoint_With_Index(int index)
{
  return nav_waypoints[index];
}

/* Moves the car forward */
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

/* Prints data to the serial port for debugging */
/* Open the Serial Plotter in the Arduino IDE to see a graph of the unfiltered
   and filtered latitude. Comment/uncomment sections to see other data.
*/
void Print_Data(void)
{
  // Original and filtered latitude (multiplied by a factor to help the
  // Serial Plotter's minimal capabilities!)
  Serial.print("GPSLAT: "); Serial.print(1000000 * f_gps_reading_lat, 1);
  Serial.print(" GPSLATF: "); Serial.print(1000000 * gps_filtered.lat, 1);

  // Original and filtered longitude
  //  Serial.print(" GPSLON: "); Serial.print(1000000*(f_gps_reading_lon), 1);
  //  Serial.print(" GPSLONF: "); Serial.print(1000000*(gps_filtered.lon), 1);


  // Waypoint angle, robot car angle and heading error
  //  Serial.print(" CMPANG: "); Serial.print(compass_angle);
  //  Serial.print(" GPSANG: "); Serial.print(waypoint_angle);
  //  Serial.print(" HDERR: "); Serial.print(heading_error);

  // Put here other data you want to plot
  Serial.println();
}
