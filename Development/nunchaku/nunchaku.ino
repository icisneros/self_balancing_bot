#include <Wire.h>
#include "Nunchuk.h"

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2,3); // Arduino: RX | TX


void setup() {

    Serial.begin(9600);
    
    /* Nunchuk I2C library setup*/
    Wire.begin();
    // nunchuk_init_power(); // A1 and A2 is power supply
    nunchuk_init();


    /* HC-05 Serial setup */
    // HC-06 default serial speed for communcation mode is 9600
    BTserial.begin(9600);  
}


void loop() {

    if (nunchuk_read()) {
        // Work with nunchuk_data
        nunchuk_print();
    }
    delay(10);
} 
