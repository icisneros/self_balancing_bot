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
int jy_low = -114;
int jy_high = 114;


// Digital debouncing
unsigned long lasttime = 0;
unsigned long lockout = 300;  // lockout time in milliseconds


// Comms
const char start_symb = 'S';
const char end_symb = 'N';


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

    // For debugging
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
    uint8_t cbutton = nunchuk_buttonC();
    uint8_t zbutton = nunchuk_buttonZ();

    // only send out a message if the joystick is moved beyond the noise floor (j_thresh), or if a button is 
    // pressed once outside of the lockout time (to prevent duplicates of the same message)
    if (abs(jx) > j_thresh  || abs(jy) > j_thresh || ((cbutton != 0  || zbutton != 0 ) && (millis() - lasttime) > lockout) )
    {
      
      // keep track of the time counter
      lasttime = millis();

      // Map the joystick values to the motor speed domain
      jx = map(jx, jx_low, jx_high, motor_low, motor_high);
      jy = map(jy, jy_low, jy_high, motor_low, motor_high);

      // Convert ints to chars so that it can be sent out via the write function
      char jx_str[5];
      char jy_str[5];
      char cb_str[2];
      char zb_str[2];
      
      // write out the payload
      // payload format: Sjx,jy,cb,zbN
      // there has to be an easier way to just concatenate all of this
      BTserial.write(start_symb);
      BTserial.write(itoa(jx, jx_str, 10));
      BTserial.write(',');
      BTserial.write(itoa(jy, jy_str, 10));
      BTserial.write(',');
      BTserial.write(itoa(cbutton, cb_str, 10));
      BTserial.write(',');
      BTserial.write(itoa(zbutton, zb_str, 10));
      BTserial.write(end_symb);
    }
  }
  
  delay(10);    


} 
