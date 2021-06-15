#include <ESP8266WiFi.h>
#include "FirebaseESP8266.h"

#define FIREBASE_HOST "ahss-77137-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "oQViHnmXCkAywfBMKytevDAYugsx4ruZbx1t8dKV"

#define WIFI_SSID ""
#define WIFI_PASSWORD ""
FirebaseData firebaseData;

const int trigPin = 5;
const int echoPin = 6;


const int tank_height = 10;

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
  Firebase.begin (FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
}
void loop() {
  sensorUpdate();
}
void sensorUpdate()
{
 long d, r, x;
  digitalWrite(trigPin, LOW);
  delay(1000);
  digitalWrite(trigPin, HIGH);
  delay(1000);
  digitalWrite(trigPin, LOW);
  d = pulseIn(echoPin, HIGH);
  x = (d / 2) / 29.1;
  float dummy = (x-2);
  float m = (tank_height - dummy);
  if(m<=0 || m >10)
  {
    Serial.print("0");
  }
  float  t = m * 65;
  r = (t/650)*100;
  Serial.print(r);
  Serial.print("%");
  Serial.println();
  delay(1000);
  
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
