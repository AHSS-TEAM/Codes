#include <ESP8266WiFi.h>
#include "FirebaseESP8266.h"

#define FIREBASE_HOST "ahss-de7a2-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "e975CHCtEW9X46tlVsqcF0mbKYZC8kMF0zyxs778"

#define WIFI_SSID "Your wifi name"
#define WIFI_PASSWORD "your wifi password"

FirebaseData firebaseData;

int trigPin = D5;
int echoPin = D6;

const int tank_height = 20;

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
  delay(2000);
  digitalWrite(trigPin, HIGH);
  delay(1000);
  digitalWrite(trigPin, LOW);
  d = pulseIn(echoPin, HIGH);
  x = (tank_height - ((d / 2) / 29.1));
  r = x * 75;
  
  
  Serial.print(r );
  Serial.println("mL");
  if (isnan(r)) {
    Serial.print(F("failed to read sensor"));
    return;
  }
  if (Firebase.setFloat(firebaseData, "liquid_level", r)) {
    Serial.println("PASSED");
    Serial.println("PATH:" + firebaseData.dataPath());
    Serial.println("type:" + firebaseData.dataType());
    Serial.println("PATH:" + firebaseData.ETag());
    //Serial.println(-------------------);
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON:" + firebaseData.errorReason());
    //Serial.println(-------------------);
    Serial.println();
  }
}
