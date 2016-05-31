#include <Servo.h>
unsigned char x=30;
unsigned char a,b;
Servo m1,m2,m3,m4;
void setup()
{
  m1.attach(3);
  //m1.write(40);

  m2.attach(11);
  //m2.write(40);

  m3.attach(10);
 // m3.write(40);

  m4.attach(6);
 // m4.write(40);
  Serial.begin(9600);
}

void loop(){

  a = Serial.read();
  if (a==48)
  {
    Serial.println(x,10);
    x+=1;
  }
  else if(a==49)
  {
    Serial.println(x,10);
    x-=1;
  }

  m1.write(x);
  m2.write(x);
  m3.write(x);
  m4.write(x);
 }

