//Libraries
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MechaQMC5883.h>

//Method prototypes
void gps_data();
int compass_heading_angle();
int gps_path_angle();
int distance_between_waypoint();
void traverse_to_waypoint();
void validate_left();
void validate_right();
void move_forward();
void turn_right();
void turn_left();
void idle();

//objects and variable declaration
MechaQMC5883 magnetometer; //compass object to get heading
SoftwareSerial serial_connection(5,6);  //tx,rx 
TinyGPSPlus gps; //GPS object to process the NMEA data
static const uint32_t GPS_baud_rate = 9600; //baudrate 
int satellite_count = 0; //number of satellites
int path_angle, heading_angle, path_distance;
int x,y,z,azimuth;
float lat_value[]={};
float long_value[]={};
int lat_value_length = sizeof(lat_value)/sizeof(lat_value[0]);

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT); 
  pinMode(9, OUTPUT); //pwm1
  pinMode(10, OUTPUT); //pwm2
  
  Serial.begin(GPS_baud_rate);
  Wire.begin();
  serial_connection.begin(GPS_baud_rate); 
  magnetometer.init();
  magnetometer.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);

  while(satellite_count < 4)
  {
    gps_data();
    satellite_count = gps.satellites.value();
    Serial.println("Scanning for satellites...");
    Serial.println(satellite_count);
  }
}

void loop()
{ 
  gps_data();
  compass_heading_angle();
  distance_between_waypoint();
  traverse_to_waypoint();
  Serial.print("Moving..");
}

void gps_data() //gets the lattitude and longitude from neo-6m gps module
{
    //while there are incoming characters  from the GPS
    while(serial_connection.available()) 
    {
      //This feeds the serial NMEA data into the library one char at a time
      gps.encode(serial_connection.read()); 
    }
    // uh oh
    if (millis() > 5000 && gps.charsProcessed() < 10) 
    {
      Serial.println("ERROR: not getting any GPS data!");
      // dump the stream to Serial
      Serial.println("GPS stream dump:");
      while (true); // infinite loop
    }
}

int compass_heading_angle()
{
    magnetometer.read(&x, &y, &z, &azimuth);
    //get custom azimuth
    heading_angle = magnetometer.azimuth(&y,&x);
    Serial.print("compass heading: ");
    Serial.println(heading_angle);
    return heading_angle; 
}

int gps_path_angle()
{
    gps_data();
    for (int i=0; i<lat_value_length; i++)
    {
      path_angle = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),lat_value[i],long_value[i]);
      Serial.print("path angle: ");
      Serial.println(path_angle);
      Serial.println(TinyGPSPlus::cardinal(path_angle));
      return path_angle;
    }
}

int distance_between_waypoint()
{
    gps_data();
    for (int i=0; i<lat_value_length; i++)
    {
      path_distance = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),lat_value[i],long_value[i]);
      Serial.print("path angle: ");
      Serial.println(path_distance);
      return path_distance;
    }
}

void traverse_to_waypoint()
{
    int heading = compass_heading_angle();
    int path_direction = gps_path_angle();
    int waypoint_distance = distance_between_waypoint();
    if (waypoint_distance < 2)
      idle();
    if(abs(path_direction - heading) <=5 )
      move_forward();
    else if (abs(path_direction > heading))
    {
      if((abs(path_direction - heading) <= 180))
      {
        validate_right();
      }
      else
      {
        validate_left();
      }
    }
    else if (abs(path_direction < heading))
    {
      if((abs(path_direction - heading) <= 180))
      {
        validate_right();
      }
      else
      {
        validate_left();
      }
    }
}

void validate_right()
{
    while(abs(path_angle - heading_angle) > 5)
    {
      compass_heading_angle();
      gps_path_angle();
      distance_between_waypoint();
      move_right();
      if(path_distance<2)
        idle();
      if(abs(path_angle - heading_angle) <=5)
        break;
    }
    move_forward();
}

void validate_left()
{
     while(abs(path_angle - heading_angle) > 5)
      {
        compass_heading_angle();
        gps_path_angle();
        distance_between_waypoint();
        move_left();
        if(path_distance<2)
          idle();
        if(abs(path_angle - heading_angle) <=5)
          break;
      }
      move_forward();
}

void move_forward()
{
    analogWrite(10, 255);
    analogWrite(9, 255);
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    
}

void move_left()
{
    analogWrite(10, 200);
    analogWrite(9, 200);
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
        
}

void move_right()
{
    analogWrite(10, 200);
    analogWrite(9, 200);
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
        
}

void idle()
{
    analogWrite(10, 0);
    analogWrite(9, 0);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
        
}
