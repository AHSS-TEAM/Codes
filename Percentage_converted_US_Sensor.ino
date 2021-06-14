const int trigPin = 5;
const int echoPin = 6;

const int tank_height = 10;

void setup()
{
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
}
