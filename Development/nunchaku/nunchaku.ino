#include <Wire.h>
#include "Nunchuk.h"

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2,3); // Arduino: RX | TX

// Prevents false positives (sometimes 
// joystick gets stuck at small numbers
// around 0)
int16_t j_thresh = 5;

// Motor speed value range
int motor_low = -255;
int motor_high = 255;

// Joystick x range
int jx_low = -116;
int jx_high = 116;

// Joystick y range
int jy_low = -123;
int jy_high = 114;

void setup() 
{
  
  Serial.begin(9600);
    
  /* Nunchuk I2C library setup*/
  Wire.begin();
  // nunchuk_init_power(); // A1 and A2 is power supply
  nunchuk_init();


  /* HC-05 Serial setup */
  // HC-06 default serial speed for communcation mode is 9600
  BTserial.begin(9600);  
}


void loop() 
{

//    if (nunchuk_read()) {
//        // Work with nunchuk_data
//        nunchuk_print();
//    }
//    delay(10);



  if (nunchuk_read()) 
  {
    // Work with nunchuk_data
    int16_t jx = nunchuk_joystickX();
    int16_t jy = nunchuk_joystickY();
    // Button c
    // Button z

    if (abs(jx) > j_thresh  || abs(jy) > j_thresh)
    {
      // Map the joystick values to the motor speed domain
      jx = map(jx, jx_low, jx_high, motor_low, motor_high);
      jy = map(jy, jy_low, jy_high, motor_low, motor_high);
      
      // write out the payload
      Serial.print("jx: ");
      Serial.print(jx, DEC);
      Serial.print(",  jy: ");
      Serial.print(jy, DEC);
      Serial.print("\n");
    }
  }
  
  delay(10);    
  //Serial.print("nothing\n");


//  BTserial.write("Hello from Master Module!\n"); 

} 
