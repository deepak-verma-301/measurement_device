#define echoPin      6
#define triggerPin   7
void setup() {
  Serial.begin(115200);
  pinMode(echoPin,INPUT);
  pinMode(triggerPin,OUTPUT);


}
void loop() {
digitalWrite(triggerPin,LOW);
digitalWrite(triggerPin,HIGH);
delayMicroseconds(10);
digitalWrite(triggerPin,LOW);

unsigned long highTime = pulseIn(echoPin,HIGH);

float distance = (highTime * 0.0343) / 2;

Serial.print(distance);
Serial.println(" cm");
delay(100);
}
