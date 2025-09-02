#include <Wire.h>
#include<math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSerif9pt7b.h>
#include<Fonts/FreeSerif12pt7b.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64 
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define mpuAdd 0x68                     // default i2c address of mpu6050
#define accSense 16384.0               
#define gyroSense 131.0
#define toDegree  (180.0/PI)
#define alpha     .87

#define echoPin      6
#define triggerPin   7
#define sel          8

int16_t rawAX, rawAY, rawAZ;             
int16_t rawGX, rawGY, rawGZ;

float ax, ay, az;
float gx, gy, gz;

float angleX,angleY;
float refinedX;
float roll=0,pitch=0;
float offsetX, offsetY;

unsigned long start = 0;
unsigned long stop = 0;
float dt = 0;

byte currentMode=1;
bool selPstate = 0;
bool selCstate = 0;
bool buttonState = 0;
unsigned long lastDebounceTime  = 0;
bool lastButtonState = false;


unsigned long lastUltrasonicTime = 0;
float distance = 0;
unsigned long lastAngleCalTime = 0;
float angle = 0;

void setup() {
Serial.begin(115200);
Wire.begin();
Serial.println("Setup started");

Wire.beginTransmission(mpuAdd);
Wire.write(0x6B);                           // power management register of mpu6050
Wire.write(0x00);                           
Wire.endTransmission();
 
offsetCal(offsetX,offsetY,600);           // calculating the offset value of gyro with 600 sample 
stop = millis();

  pinMode(echoPin,INPUT);
  pinMode(triggerPin,OUTPUT);
  pinMode(sel,INPUT_PULLUP);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
   display.clearDisplay();
   display.drawPixel(20, 20, SSD1306_WHITE);

  display.display();


Wire.beginTransmission(mpuAdd);
Wire.write(0x6B);                           // power management register of mpu6050
Wire.write(0x00);                           
Wire.endTransmission(true);
 
selPstate = digitalRead(sel);
currentMode = 1;
}

void loop() {
selCstate = digitalRead(sel);

if (selCstate != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {
    if ( selCstate != buttonState) {
      buttonState = selCstate;

      if (buttonState == HIGH) {
        currentMode++;
        //Serial.println(count);
      }
    }
  }
  if (currentMode >4) {
    currentMode = 1;
  }
 lastButtonState = selCstate;


  if (millis() - lastUltrasonicTime >= 150) {
    lastUltrasonicTime = millis();
    distance = distCal();  
  }

    if (millis() - lastAngleCalTime >= 50) {
    lastAngleCalTime = millis();
    angle = angleCal();
  }







  
switch (currentMode){

case 1:
display.clearDisplay();
  display.setFont(&FreeSerif9pt7b);
  display.setTextColor(SSD1306_WHITE);  
  display.setCursor(SCREEN_WIDTH/4,20);
  display.print("Distance");
  display.setFont(&FreeSerif12pt7b);
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(10, 50);  
  display.print(distance,1);
  display.setCursor(70,50);
  display.print("cm");
  display.drawLine(0,SCREEN_HEIGHT-1,map(distance,0,300,0,SCREEN_WIDTH),SCREEN_HEIGHT-1,SSD1306_WHITE);
display.display();
break;

case 2:
display.clearDisplay();
  display.setFont(&FreeSerif9pt7b);
  display.setTextColor(SSD1306_WHITE);  
  display.setCursor(5,15);
  display.print("Angle");
  display.setCursor(SCREEN_WIDTH/2, 15);
  display.print(angle);
  display.drawCircle(110, 7, 2, SSD1306_WHITE);

  display.drawCircle(SCREEN_WIDTH/2,SCREEN_HEIGHT,45,SSD1306_WHITE);
  if(angle>0){
  display.drawLine(SCREEN_WIDTH/2,SCREEN_HEIGHT,(SCREEN_WIDTH/2 + 45 * cos(angle*(PI/180.0))),(SCREEN_HEIGHT - 45 * sin(angle*(PI/180.0))),SSD1306_WHITE);
  }
  else if(angle<0){
    display.drawLine((SCREEN_HEIGHT - 45 * sin(abs(angle-90)*(PI/180.0))), (SCREEN_WIDTH/2 + 45 * cos(abs(angle-90)*(PI/180.0))), SCREEN_WIDTH/2,SCREEN_HEIGHT,SSD1306_WHITE);
  }

display.display();
break;
case 3:
spiritLevel(roll,pitch);
display.clearDisplay();
  display.setFont(&FreeSerif9pt7b);
  display.setTextColor(SSD1306_WHITE); 

  display.drawRect(0,0,SCREEN_WIDTH,SCREEN_HEIGHT,SSD1306_WHITE);
  display.drawLine(SCREEN_WIDTH/2,0,SCREEN_WIDTH/2,SCREEN_HEIGHT,SSD1306_WHITE);
  display.drawLine(0,SCREEN_HEIGHT/2,SCREEN_WIDTH,SCREEN_HEIGHT/2,SSD1306_WHITE);
  display.drawCircle((int)((SCREEN_WIDTH/2)+roll),(int)((SCREEN_HEIGHT/2)-pitch),7,SSD1306_WHITE);
  display.fillCircle((int)((SCREEN_WIDTH/2)+roll),(int)((SCREEN_HEIGHT/2)-pitch),3,SSD1306_WHITE);

display.display();
break;
case 4:
display.clearDisplay();
  display.setFont(&FreeSerif9pt7b);
  display.setTextColor(SSD1306_WHITE);  
  display.setCursor(10, 13);  
  display.print("Press the button");
display.display();
break;
}


}


float distCal(){
 digitalWrite(triggerPin,LOW);
 digitalWrite(triggerPin,HIGH);
 delayMicroseconds(10);
 digitalWrite(triggerPin,LOW);

 unsigned long highTime = pulseIn(echoPin,HIGH,30000);

 float distance = (highTime * 0.0343) / 2;
 return distance;
}

void offsetCal(float &x, float &y, int sampleSize){
  Serial.println("Calculating the offset values");
   for(int i = 0; i < sampleSize; i++){
  Wire.beginTransmission(mpuAdd);
  Wire.write(0x43);
  Wire.endTransmission();

  if(Wire.requestFrom(mpuAdd,6) == 6){

     int16_t rawX = (int16_t)Wire.read() << 8 | Wire.read();
     int16_t rawY = (int16_t)Wire.read() << 8 | Wire.read();
     float  gx = (float)rawX/131.0;
     float  gy = (float)rawY/131.0;

      x += gx;
      y += gy;
      
      delay(5);                        } 
   }
   x /= sampleSize;
   y /= sampleSize;

}

float angleCal(){
  start = millis();
  dt = (start-stop)/1000.0;
  stop = start;
  Wire.beginTransmission(mpuAdd);
  Wire.write(0x3B);
  Wire.endTransmission();

  if(Wire.requestFrom(mpuAdd,14) == 14){
    rawAX = (int16_t)Wire.read() << 8 | Wire.read();
    rawAY = (int16_t)Wire.read() << 8 | Wire.read();
    rawAZ = (int16_t)Wire.read() << 8 | Wire.read();

    Wire.read(); Wire.read();

    rawGX = (int16_t)Wire.read() << 8 | Wire.read();
    rawGY = (int16_t)Wire.read() << 8 | Wire.read();
    rawGZ = (int16_t)Wire.read() << 8 | Wire.read();

    ax = (float)rawAX/accSense;
    ay = (float)rawAY/accSense;
    az = (float)rawAZ/accSense;

    // angleX = atan2(ax,sqrt(ay*ay + az*az)) * toDegree;
    // angleY = atan2(ay,sqrt(ax*ax+ az*az)) * toDegree;

      angleX = atan2(ay,az) * toDegree;
    //angleY = atan2(-ax,sqrt(ay*ay + az*az)) * toDegree;

    gx = (float)rawGX/gyroSense - offsetX;
    //gy = (float)rawGY/gyroSense - offsetY;

    refinedX = alpha * (refinedX + (gx * dt)) + (1.0 - alpha) * angleX;
    //refinedY = alpha * (refinedY + (gy * dt)) + (1.0 - alpha) * angleY;
    return refinedX;
    }

}

void spiritLevel(float &roll, float &pitch){
  start = millis();
  dt = (start - stop)/1000;
  stop = start;

  Wire.beginTransmission(mpuAdd);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if(Wire.requestFrom(mpuAdd,14) == 14){
    rawAX = (int16_t)Wire.read() << 8 | Wire.read();
    rawAY = (int16_t)Wire.read() << 8 | Wire.read();
    rawAZ = (int16_t)Wire.read() << 8 | Wire.read();


    Wire.read(); Wire.read();

    rawGX = (int16_t)Wire.read() << 8 | Wire.read();
    rawGY = (int16_t)Wire.read() << 8 | Wire.read();
    rawGZ = (int16_t)Wire.read() << 8 | Wire.read();

    ax = (float)rawAX/accSense;
    ay = (float)rawAY/accSense;
    az = (float)rawAZ/accSense; 

    
    angleX = atan2(ax,sqrt(ay*ay + az*az)) * toDegree;
    angleY = atan2(ay,sqrt(ax*ax + az*az)) * toDegree;

    

    gx = (float)rawGX/gyroSense;
    gy = (float)rawGY/gyroSense;  

    gx -= offsetX;
    gy -= offsetY;

    roll = alpha*(roll + (gx*dt)) + (1.0-alpha)*(angleX);
    pitch = alpha*(pitch + (gy*dt)) + (1.0-alpha)*(angleY);

      }
 
}