#include <Servo.h>

unsigned short int fl,fr,rl,rr;

#define BaseValueFRONT   (short int)1101U
#define BaseValueREAR    (short int)1100U
unsigned char a,b;
Servo FrontLeft,FrontRight,RearLeft,RearRight;

void setup() {
  // put your setup code here, to run once:
  FrontLeft.attach(3);
  FrontRight.attach(6);
  RearLeft.attach(10);
  RearRight.attach(11);

  fl = BaseValueFRONT;
  fr = BaseValueFRONT;
  rl = BaseValueREAR;
  rr = BaseValueREAR;

    Serial.begin(9600);
}

void loop() {

  a = Serial.read();
  
  if (a == 55)  // 7 is pressed
  {
    Serial.write("Front Left: ");
    Serial.println(fl,10);
    fl += 1;
  }
  else if(a == 57)  // 9 is pressed
  {
    Serial.write("Front Right: ");
    Serial.println(fr,10);
    fr +=  1;
  }
  else if(a == 49)  // 1 is pressed
  {
    Serial.write("Rear Left: ");
    Serial.println(rl,10);
    rl += 1;
  }
  else if(a == 52)
  {
    Serial.write("ALL Where Upped: ");
    Serial.println(rr,10);

    fl+=1;
    fr+=1;
    rl+=1;
    rr+=1;
  }
  else if(a == 54)  // 6 is pressed
  {
    Serial.write("ALL Where Downed: ");
    Serial.println(rr,10);

    
    fl-=1;
    fr-=1;
    rl-=1;
    rr-=1;
  }
  else if(a == 51)  // 3 is pressed
  {
    Serial.write("Rear Right: ");
    Serial.println(rr,10);
    rr += 1;
  }
  else if(a == 53)  // 5 is pressed
  {
    Serial.write("STOP\n");
        while(fl > BaseValueFRONT){ 
      fl--;
    }

    
        while(fr > BaseValueFRONT){
      fr--;
    }

    
     while(rl > BaseValueREAR){
      rl--;
    }
  
    
       while(rr > BaseValueREAR){
      rr--;
    }

 
  delay(100);
  }

  FrontLeft.writeMicroseconds(fl);
  FrontRight.writeMicroseconds(fr);
  RearLeft.writeMicroseconds(rl);
  RearRight.writeMicroseconds(rr);

}


