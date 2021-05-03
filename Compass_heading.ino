
#include <Wire.h>
#include <MechaQMC5883.h>
MechaQMC5883 qmc;

void setup() 
{
  Wire.begin();
  Serial.begin(9600);
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
}

void loop()
{
  int x, y, z;
  int azimuth;
  int heading;
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z,&azimuth);
  azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
  heading=azimuth;
  Serial.println(heading);
  heading_id(heading);
  Serial.println();
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
