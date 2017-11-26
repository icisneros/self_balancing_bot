/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//#include <FastSerial.h>

//#define SLAVE_ADDRESS          0x29 //slave address,any number from 0x01 to 0x7F
#define DEVICE_ID           122

//#define MAX_SENT_BYTES        3
//#define MODE_REG        0x09  // remove?
//#define CONFIG_REG        0x0A  //


// where we store the values
static uint8_t  mode_register;
static uint8_t  config_register;


// new data flag
static float speed_1, speed_2;



// Can probably get rid of the status and id
struct reg_map {
  uint8_t status;     // I2C status
  int16_t wheel_1_delta;  // My Data
  int16_t wheel_2_delta;
  int16_t wheel_1_speed;  // My Data
  int16_t wheel_2_speed;
  uint8_t mode;     // register values  WTF is this
  uint8_t config;     // register values  WTF is this
  uint8_t id;       // never changes
};



// buffer
static union {
  reg_map map;
  uint8_t bytes[];
} 
_buffer;


volatile int16_t wheel_1, wheel_2;  // From the ISRs

static uint32_t loopTimer;

// Serial ports
//FastSerialPort0(Serial);        // FTDI / console


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);



void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  myOtherMotor->setSpeed(150);
  myOtherMotor->run(FORWARD);
  // turn on motor
  myOtherMotor->run(RELEASE);

//  Serial.begin(115200);
//  Serial.print("Begin");

  // setup pins to read the encoder
  pin_setup();


  // init our device ID
  _buffer.map.id = DEVICE_ID;

  
}

void loop() {
  uint8_t i;
  
//  Serial.print("tick");

  myMotor->run(FORWARD);
  myOtherMotor->run(FORWARD);
  
  for (i=0; i<255; i++) 
  {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);   
    delay(10);
  }
  
  Serial.print("New datapoint ===============\n");
  requestEvent();
  
  for (i=255; i!=0; i--) 
  {
    myMotor->setSpeed(i); 
    myOtherMotor->setSpeed(i);  
    delay(10);
  }
  
//  Serial.print("tock");

  myMotor->run(BACKWARD);
  myOtherMotor->run(BACKWARD);
  
  for (i=0; i<255; i++) 
  {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }
  
  Serial.print("New datapoint ===============\n");
  requestEvent();
  
  for (i=255; i!=0; i--) 
  {
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

//  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);

//  delay(20);

}





//void setup()
//{
//
//}

//void loop()
//{
//  delay(20);
//  Serial.print("New datapoint ===============\n");
//  requestEvent();
//}


// The main function - where all of the magic happens
void requestEvent()
{
  uint32_t timer = micros();

  // copy over and clear wheel encoder values
  _buffer.map.wheel_1_delta = wheel_1;
  wheel_1 = 0;

  _buffer.map.wheel_2_delta = wheel_2;
  wheel_2 = 0;

  // find elapsed time since last read
  float dt  = (float)(timer - loopTimer) / 1000000.0;

  // prevent a big buildup, read at a least 10hz
  dt = min(dt, .15);

  loopTimer   = timer;

  // Find velocity of rotation
  speed_1 += ((float)_buffer.map.wheel_1_delta / dt);
  speed_2 += ((float)_buffer.map.wheel_2_delta / dt);

  // small averaging filter
  speed_1 *= .5;
  speed_2 *= .5;

  _buffer.map.wheel_1_speed = speed_1;
  _buffer.map.wheel_2_speed = speed_2;



//  Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
//  _buffer.map.status,
//  _buffer.map.wheel_1_delta,
//  _buffer.map.wheel_2_delta,
//  _buffer.map.wheel_1_speed,
//  _buffer.map.wheel_2_speed,
//  _buffer.map.mode,
//  _buffer.map.config,
//  _buffer.map.id);



//  Serial.print("Wheel 1:   %d : %d \t|\t Wheel2:  %d: %d\n",
//  _buffer.map.wheel_1_delta,
//  _buffer.map.wheel_1_speed,
//  _buffer.map.wheel_2_delta,
//  _buffer.map.wheel_2_speed);

  Serial.print("Wheel 1:  ");
  Serial.print(_buffer.map.wheel_1_delta );
  Serial.print(" : ");
  Serial.print(_buffer.map.wheel_1_speed);


  Serial.print("Wheel 2:  ");
  Serial.print(_buffer.map.wheel_2_delta );
  Serial.print(" : ");
  Serial.print(_buffer.map.wheel_2_speed);

  //Set the buffer to send all bytes I2C
  //Wire.write(_buffer.bytes, sizeof(_buffer));
}



// Clean this up so that we are only initializing the Analog INs
void pin_setup()
{

  // ATMEGA
  // PORTD - PCMSK2
  // p0       // PD0 - RX D     - PCINT16
  // p1       // PD1 - TX D     - PCINT17
//  pinMode(2, INPUT);  // PD2 - INT0     - PCINT18  WHEEL1
//  pinMode(3, INPUT);  // PD3 - INT1     - PCINT19  WHEEL1
  pinMode(4, INPUT);  // PD4 - XCK / T0   - PCINT20
  pinMode(5, INPUT);  // PD5 - T0     - PCINT21
//  pinMode(6, INPUT);  // PD6 - T1     - PCINT22
//  pinMode(7, INPUT);  // PD7 - AIN0   - PCINT23

  // PORTB- PCMSK0
  pinMode(8, INPUT);  // PB0 - AIN1   - PCINT0   WHEEL2
  pinMode(9, INPUT);  // PB1 - OC1A   - PCINT1   WHEEL2
//  pinMode(10, INPUT); // PB2 - OC1B   - PCINT2
//  pinMode(11, INPUT);   // PB3 - MOSI / OC2 - PCINT3
//  pinMode(12, INPUT);   // PB4 - MISO   - PCINT4
//  pinMode(13, INPUT);   // PB5 - SCK    - PCINT5


  // Activate pin change interrupt ports
  PCICR = _BV(PCIE2) | _BV(PCIE0);  // Turn on Pin Change interrupts for ports 0 and 2
  
  // Choose which pins to use for interrupts
//  PCMSK2   = _BV(PCINT18); // | _BV(PCINT19);
  PCMSK2  = _BV(PCINT21); // | _BV(PCINT19);
  PCMSK0  = _BV(PCINT0);  // | _BV(PCINT1);
  
  // ISR vectors
  // PCINT0_vect = interrupt vector for external interrupt on pin PCINT  0..7   WHEEL2
  // PCINT1_vect = interrupt vector for external interrupt on pin PCINT  8..14  NOT USED
  // PCINT2_vect = interrupt vector for external interrupt on pin PCINT 16..23  WHEEL1
}



// Wheel1 interrupt service  (pins 2 and 3)
//ISR(PCINT2_vect){
//
//  if((PIND & B00000100) == 0 && (PIND & B00001000) == 0){
//    wheel_1++;
//  }
//  else if((PIND & B00000100) > 1 && (PIND & B00001000) > 1){
//    wheel_1++;
//  }
//  else{
//    wheel_1--;
//  }
//
//  //Serial.printf("1: %d\n",_buffer.map.wheel_1_speed);
//}


// Wheel1 interrupt service (pins 4 and 5)
ISR(PCINT2_vect){

  if((PIND & B00010000) == 0 && (PIND & B00100000) == 0){
    wheel_1++;
  }
  else if((PIND & B00010000) > 1 && (PIND & B00100000) > 1){
    wheel_1++;
  }
  else{
    wheel_1--;
  }

  //Serial.printf("1: %d\n",_buffer.map.wheel_1_speed);
}


// Wheel2 interrupt service  (pins 8 and 9)
ISR(PCINT0_vect){

  if((PINB & B00000001) == 0 && (PINB & B00000010) == 0){
    wheel_2++;
  }
  else if((PINB & B00000001) == 1 && (PINB & B00000010) > 1){
    wheel_2++;
  }
  else{
    wheel_2--;
  }

  //Serial.printf("2: %d\n",_buffer.map.wheel_2_speed);
}
