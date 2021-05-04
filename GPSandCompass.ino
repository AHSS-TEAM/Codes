#include<Arduino.h>
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;
SoftwareSerial serial_connection(5,6); //tx,rx 
TinyGPSPlus gps;// GPS object to process the NMEA data

void setup()
{
  Wire.begin();
  Serial.begin(9600);                //This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);     //This opens up communications to the GPS
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
  Serial.println("GPS and Magnetometer Start "); //To show in the serial monitor that the sketch has started
}


void loop()
{
  
  int x, y, z;
  int azimuth;
  int heading;
  
  qmc.read(&x, &y, &z,&azimuth);
  azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
  heading=azimuth;
  
  while(serial_connection.available())              //While there are incoming characters  from the GPS
  {
    gps.encode(serial_connection.read());           //This feeds the serial NMEA data into the library one char at a time
  }
  
  if(gps.location.isUpdated())          //This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    Serial.print("Satellite Count:");
    Serial.println(gps.satellites.value());
    Serial.print("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude:");
    Serial.println(gps.location.lng(), 6);
    Serial.println(heading)
    heading_id(heading);
    Serial.println();
    Serial.println("");
    
    delay(2000);
  }
  
void heading_id(int heading)
{
  if (heading >= 338 || heading < 22)
  {
    Serial.println("NORTH");
  }
  else if (heading >= 22 && heading < 68)
  {
    Serial.println("NORTH-EAST");
  }
  else if (heading >= 68 && heading < 113)
  {
    Serial.println("EAST");
  }
  else if (heading >= 113 && heading < 158)
  {
    Serial.println("SOUTH-EAST");
  }
  else if (heading >= 158 && heading < 203)
  {
    Serial.println("SOUTH");
  }
 else if (heading >= 203 && heading < 248)
  {
    Serial.println("SOTUH-WEST");
  }
  else if (heading >= 248 && heading < 293)
  {
    Serial.println("WEST");
  }
  else if (heading >= 293 && heading < 338)
  {
    Serial.println("NORTH-WEST");
  }
  else
  {
    Serial.println("No reading");
  }
}
