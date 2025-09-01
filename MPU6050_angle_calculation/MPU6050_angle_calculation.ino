#include<Wire.h>                          // wire library for i2c communication
#include<math.h>

#define mpuAdd 0x68                     // default i2c address of mpu6050
#define accSense 16384.0               
#define gyroSense 131.0
#define toDegree  (180.0/PI)
#define alpha     .96                  

int16_t rawAX, rawAY, rawAZ;             
int16_t rawGX, rawGY, rawGZ;

float ax, ay, az;
float gx, gy, gz;

float angleX,angleY;
float refinedX,refinedY;
float offsetX, offsetY;

unsigned long start = 0;
unsigned long stop = 0;
float dt = 0;

void setup() {
Serial.begin(115200);
Wire.begin();

Wire.beginTransmission(mpuAdd);
Wire.write(0x6B);                           // power reset register of mpu6050
Wire.write(0x00);                           
Wire.endTransmission();
 
offsetCal(offsetX,offsetY,600);           // calculating the offset value of gyro with 600 sample 
stop = millis();
}

void loop() {
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

    Serial.print(100);
    Serial.print(" AngleX ");
    Serial.print((int)refinedX);
    Serial.print(" ");
    Serial.print(-100);
    Serial.println(" ");

  }

delay(10);
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