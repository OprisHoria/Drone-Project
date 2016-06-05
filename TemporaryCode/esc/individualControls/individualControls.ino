#include <Servo.h>
unsigned char fl,fr,rl,rr;
unsigned char a,b;
Servo FrontLeft,FrontRight,RearLeft,RearRight;

void setup() {
  // put your setup code here, to run once:
  FrontLeft.attach(3);
  FrontRight.attach(6);
  RearLeft.attach(10);
  RearRight.attach(11);

  fl = 30;
  fr = 30;
  rl = 30;
  rr = 30;

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
    fr += 1;
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
        while(fl > 30){
      fl--;
    }

    
        while(fr > 30){
      fr--;
    }

    
     while(rl > 30){
      rl--;
    }
  
    
       while(rr > 30){
      rr--;
    }

  
  delay(100);
  }

  FrontLeft.write(fl);
  FrontRight.write(fr);
  RearLeft.write(rl);
  RearRight.write(rr);

}


