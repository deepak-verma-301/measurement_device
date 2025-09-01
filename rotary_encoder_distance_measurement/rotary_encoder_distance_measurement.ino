#define clk           2
#define data          3
#define button        4
#define wheel_circum  35 // in mm
bool clkPreState = false;
bool clkCurState = false;

int c = 0,cw = 0;

void setup() {
 Serial.begin(115200);
 pinMode(clk,INPUT);
 pinMode(data,INPUT);
 pinMode(button,INPUT);
  attachInterrupt(digitalPinToInterrupt(clk), readEncoder, CHANGE);
  clkPreState = digitalRead(clk);

}

void loop() {
    if(digitalRead(button)==LOW){
      c = 0;
      cw = 0;
    }
 float distance = c * (wheel_circum / 30);
   Serial.println(distance/10.0);

}

void readEncoder() {

  clkCurState = digitalRead(clk);

  if(clkCurState != clkPreState){
    if(digitalRead(data) != clkCurState){
      c++;
      }
    else{
      cw++;
    }
  } 

  clkPreState = clkCurState; 
}
