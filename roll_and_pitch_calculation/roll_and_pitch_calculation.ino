  #include <Arduino.h>
#include<Wire.h>
#include<math.h>

#define add           0x68
#define accSense      16384.0
#define gyroSense     131.0
#define sample        600
#define deg           (180.0/PI) 
#define alpha         .95

int16_t rawax,raway,rawaz,rawgx,rawgy,rawgz;
float ax,ay,az,gx,gy,gz;
float biasGX=0;
float biasGY=0;

float fusionX,fusionY;
float angleX,angleY;

unsigned long start=0;
unsigned long stop=0;
float dt=0;

void setup() {
Serial.begin(115200);
Wire.begin();

Wire.beginTransmission(add);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission(false);

for(int i=0; i<sample; i++){
  Wire.beginTransmission(add);
  Wire.write(0x43);
  Wire.endTransmission(false);
  if(Wire.requestFrom(add,4) == 4){

    rawgx = (int16_t)Wire.read() << 8 | Wire.read();
    rawgy = (int16_t)Wire.read() << 8 | Wire.read();

    gx = (float)rawgx/gyroSense;
    gy = (float)rawgy/gyroSense;

    biasGX += gx;
    biasGY += gy;

    delay(5);
}
}
biasGX /= sample;
biasGY /= sample;

}

void loop() {
  start = millis();
  dt = (start - stop)/1000;
  stop = start;

  Wire.beginTransmission(add);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if(Wire.requestFrom(add,14) == 14){
    rawax = (int16_t)Wire.read() << 8 | Wire.read();
    raway = (int16_t)Wire.read() << 8 | Wire.read();
    rawaz = (int16_t)Wire.read() << 8 | Wire.read();


    Wire.read(); Wire.read();

    rawgx = (int16_t)Wire.read() << 8 | Wire.read();
    rawgy = (int16_t)Wire.read() << 8 | Wire.read();
    rawgz = (int16_t)Wire.read() << 8 | Wire.read();

    ax = (float)rawax/accSense;
    ay = (float)raway/accSense;
    az = (float)rawaz/accSense; 

    
    angleX = atan2(ax,sqrt(ay*ay + az*az)) * deg;
    angleY = atan2(ay,sqrt(ax*ax + az*az)) * deg;

    

    gx = (float)rawgx/gyroSense;
    gy = (float)rawgy/gyroSense;  

    gx -= biasGX;
    gy -= biasGY;

    fusionX = alpha*(fusionX + (gx*dt)) + (1.0-alpha)*(angleX);
    fusionY = alpha*(fusionY + (gy*dt)) + (1.0-alpha)*(angleY);
    
Serial.print(100);
Serial.print(" X ");        
Serial.print(fusionX);
Serial.print(" | Y ");
Serial.print(fusionY);
Serial.print(" ");
Serial.print(-100);
Serial.println("");

      }
  else{
    Serial.println("Data fetch error check request byte and connection");
  }

  delay(8);
}