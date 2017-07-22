#include <SoftwareSerial.h>
SoftwareSerial mySerial(-1, 2);
unsigned long lasttime = 0;
int mode = 2;

void setup()
{
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(1, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  mySerial.begin(9600);
  ledblink(mode, 300, 3);
  //OSCCAL = 0x8D;
}

void loop()
{
  if (digitalRead(0) == LOW) {
    int val;
    lasttime = millis();
    int reading = analogRead(A2);
    if (mode == 1) {
      val = map(reading, 0, 1024, 1250, 1750);
    }
    if (mode == 2) {
      val = map(reading, 0, 1024, 1250, 1750);
    }
    if (mode == 3) {
      val = map(reading, 0, 1024, 1000, 2000);
    }
    mySerial.print(val);
    mySerial.println(",1500,1500n");
  }
  else {
    mySerial.println("1500,1500,1500n");
  }
  if (millis() - lasttime > 500 && digitalRead(1) == LOW) {
    if (digitalRead(1) == LOW) {
      if (mode == 3) {
        mode = 0;
      }
      mode++;
      ledblink(mode, 300, 3);
    }
  }
}
void ledblink(int times, int lengthms, int pinnum) {
  int i = 0;
  while (i < times) {
    digitalWrite(pinnum, HIGH);
    delay (lengthms);
    digitalWrite(pinnum, LOW);
    delay(lengthms);
    i++;
  }
}
