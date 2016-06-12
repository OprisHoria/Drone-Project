void setup()
{
 Serial.begin(9600);
 pinMode(13, OUTPUT);
 pinMode(12, OUTPUT);
 pinMode(11, OUTPUT);
 pinMode(10, OUTPUT);
 digitalWrite(11, LOW);
 digitalWrite(13, HIGH);
 }

int led = 1;

void loop()
{
  if(Serial.available())
  {
    Serial.read();
    switch(led)
    {
      case 1:
      led = 2;
      digitalWrite(13, LOW);
      digitalWrite(12, HIGH);
      break;
      case 2:
      led = 3;
      digitalWrite(12, LOW);
      digitalWrite(10, HIGH);
      break;
      case 3:
      led = 1;
      digitalWrite(10, LOW);
      digitalWrite(13, HIGH);
    }
  }
}
