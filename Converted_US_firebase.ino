/*Imports*/
#include <ESP8266WiFi.h> //Library to work on NodeMCU
#include "FirebaseESP8266.h" //Library to send the data from USensor to google firebase

/*Pre-requisites*/
#define FIREBASE_HOST "ahss-de7a2-default-rtdb.firebaseio.com" //Firebase project URL
#define FIREBASE_AUTH "e975CHCtEW9X46tlVsqcF0mbKYZC8kMF0zyxs778" //Firebase project secret key 
#define WIFI_SSID "Enter your wifi ssid" //Your wifi name
#define WIFI_PASSWORD "Enter your password" //your wifi password
FirebaseData firebaseData; //Firebase object

const int trigPin = D5; //Usensor trigger pin 
const int echoPin = D6; //Usensor Echo pin

const int tank_height = 10; //Height of the fuel tank

void setup()
{
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
  Serial.print ("Connecting to");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print (".");
    delay (100);
    
  }
  Serial.println ();
  Serial.print ("Connected with IP");
  Serial.println (WiFi.localIP ());
  Serial.println ();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
}

void loop() 
{
  sensorUpdate(); //A funtion call to calculate the fuel level
}

void sensorUpdate()
{
 long d, r, x;
  digitalWrite(trigPin, LOW);
  delay(2000);
  digitalWrite(trigPin, HIGH);
  delay(2000);
  digitalWrite(trigPin, LOW);
  d = pulseIn(echoPin, HIGH); //actual distance
  x = (d / 2) / 29.1;
  float dummy = (x-2);
  float m = (tank_height - dummy);
  if(m<=0 || m >10)
  {
    Serial.print("0");
  }
  float  t = m * 65;
  r = (t/650)*100; // percentage conversion
  Serial.print(r);
  Serial.print("%");
  Serial.println();
  
  //Transmitting data to Firebase after calculation
  if (Firebase.setFloat(firebaseData, "liquid_level", r)) {
    Serial.println("PASSED");
    Serial.println("PATH:" + firebaseData.dataPath());
    Serial.println("type:" + firebaseData.dataType());
    Serial.println("PATH:" + firebaseData.ETag());
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON:" + firebaseData.errorReason());
    Serial.println();
  }
}
