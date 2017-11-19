#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
int throttle, steering, button;
int prevthrottle = 1500;
int slowdown = 1500;
int prevvoltage = 0;
unsigned long starttime;
int led = 12;
int interval = 500;
unsigned long blinktime;
int ledState = LOW;

Servo esc;
Servo steer;
Servo toggle;

void setup() {
  //starttime = millis();
  analogReference(EXTERNAL);
  pinMode(A5, INPUT);
  pinMode(A2, INPUT);
  pinMode(3, INPUT);
  pinMode(led, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(led, HIGH);
  blinktime = millis();
  Serial.begin(9600);
  //delay(5000);
  esc.attach(9);
}
void loop() {
  if (Serial.available() > 0) {
    digitalWrite(led, HIGH);
    if (!steer.attached()) {
      steer.attach(10);
    }
    if (!toggle.attached()) {
      toggle.attach(11);
    }
    if (!esc.attached()) {
      esc.attach(9);
    }
    throttle = Serial.parseInt();
    steering = Serial.parseInt();
    button = Serial.parseInt();
    if (Serial.read() == 'n') {
      if (throttle >= 800 && throttle <= 2200 && steering >= 800 && steering <= 2200 && button >= 1000 && button <= 2000) {
        Serial.print("b");
        int voltage = analogRead(A5);
        Serial.print((int)(1.0647957189871569879086401003569*voltage));
        Serial.print("e");
        esc.writeMicroseconds(throttle);
        steer.writeMicroseconds(steering);
        toggle.writeMicroseconds(button);
        prevthrottle = throttle;
        slowdown = throttle;
      }
      starttime = millis();
      blinktime = millis();
    }
  }
  if ((millis() - starttime) > 500) {
    toggle.detach();
    steer.detach();
    if (prevthrottle > 1500 && slowdown != 1500) {
      esc.write(slowdown--);
      delay(1);
    }
    if (prevthrottle < 1500 && slowdown != 1500) {
      esc.write(slowdown++);
      delay(1);
    }
  }
  if (millis() - blinktime > interval) {
    blinktime = millis();
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(led, ledState);
  }
}
