#include <FastSerial.h>

//#define SLAVE_ADDRESS      		0x29 //slave address,any number from 0x01 to 0x7F
#define DEVICE_ID      			122

//#define MAX_SENT_BYTES     		3
//#define MODE_REG				0x09	// remove?
//#define CONFIG_REG				0x0A	//

//static uint8_t receivedCommands[MAX_SENT_BYTES];

// where we store the values
static uint8_t	mode_register;
static uint8_t	config_register;


// new data flag
static float speed_1, speed_2;

// s    1       2       1s 	   2s	   m   c  id
// 0    1   2   3   4   5  6   7  8    9  10  11
// 8,   8   8   8   8   8  8   8  8    8,  8,  8
// 1    22      33      44     55      66  77  88 // proof
// 1,    0, 0,  6,  0,  0, 0,  90, 0,  66, 77, 122

// Can probably get rid of the status and id
struct reg_map {
  uint8_t status;			// I2C status
  int16_t wheel_1_delta;	// My Data
  int16_t wheel_2_delta;
  int16_t wheel_1_speed;  // My Data
  int16_t wheel_2_speed;
  uint8_t	mode;			// register values  WTF is this
  uint8_t	config;			// register values  WTF is this
  uint8_t	id;				// never changes
};



// buffer
static union {
  reg_map map;
  uint8_t bytes[];
} 
_buffer;


volatile int16_t wheel_1, wheel_2;

static uint32_t loopTimer;

// Serial ports
FastSerialPort0(Serial);        // FTDI / console


void setup()
{
  Serial.begin(115200);
  Serial.println("Begin");
  //Serial.printf("buff: %d\n", sizeof(_buffer));

  // setup pins to read the encoder
  pin_setup();


  // init our device ID
  _buffer.map.id 			= DEVICE_ID;
}

void loop()
{
  delay(20);
  Serial.println("New datapoint ===============\n");
  requestEvent();
}


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

  loopTimer 	= timer;

  // Find velocity of rotation
  speed_1 += ((float)_buffer.map.wheel_1_delta / dt);
  speed_2 += ((float)_buffer.map.wheel_2_delta / dt);

  // small averaging filter
  speed_1 *= .5;
  speed_2 *= .5;

  _buffer.map.wheel_1_speed = speed_1;
  _buffer.map.wheel_2_speed = speed_2;

//  Serial.printf("%1.4f, %d, %d\n", dt, _buffer.map.wheel_1_speed, _buffer.map.wheel_2_speed);

  /*
		debug values
   	_buffer.map.wheel_1_delta = 55;
   	_buffer.map.wheel_2_delta = 66;
   	_buffer.map.wheel_1_speed = 99;
   	_buffer.map.wheel_2_speed = 88;
   	*/

  //Serial.printf("%1.4f, %d, %d\n", //dt, _buffer.map.wheel_1_speed, _buffer.map.wheel_2_speed);
  //11, 0, -2, 0, -59, 66, 77, 122


//  Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d\n",
//  _buffer.map.status,
//  _buffer.map.wheel_1_delta,
//  _buffer.map.wheel_2_delta,
//  _buffer.map.wheel_1_speed,
//  _buffer.map.wheel_2_speed,
//  _buffer.map.mode,
//  _buffer.map.config,
//  _buffer.map.id);



  Serial.printf("Wheel 1:   %d : %d \t|\t Wheel2:  %d: %d\n",
  _buffer.map.wheel_1_delta,
  _buffer.map.wheel_1_speed,
  _buffer.map.wheel_2_delta,
  _buffer.map.wheel_2_speed);

  //Set the buffer to send all bytes I2C
  //Wire.write(_buffer.bytes, sizeof(_buffer));
}



// Clean this up so that we are only initializing the Analog INs
void pin_setup()
{
  // ATMEGA ADC
  // PORTC - PCMSK1
  // PC0 - ADC0 		– PCINT8
  // PC1 - ADC1 		– PCINT9
  // PC2 - ADC2 		– PCINT10
  // PC3 - ADC3 		– PCINT11
  // PC4 - ADC4 		– PCINT12	// SDA
  // PC5 - ADC5 		– PCINT13	// SLC

  // ATMEGA
  // PORTD - PCMSK2
  // p0				// PD0 - RX D  		- PCINT16
  // p1				// PD1 - TX D  		- PCINT17
  pinMode(2, INPUT);	// PD2 - INT0 		- PCINT18  WHEEL1
  pinMode(3, INPUT);	// PD3 - INT1 		- PCINT19  WHEEL1
  pinMode(4, INPUT);	// PD4 - XCK / T0 	- PCINT20
  pinMode(5, INPUT);	// PD5 - T0			- PCINT21
  pinMode(6, INPUT);	// PD6 - T1			- PCINT22
  pinMode(7, INPUT);	// PD7 - AIN0		- PCINT23

  // PORTB- PCMSK0
  pinMode(8, INPUT); 	// PB0 - AIN1		- PCINT0   WHEEL2
  pinMode(9, INPUT);	// PB1 - OC1A		- PCINT1   WHEEL2
  pinMode(10, INPUT);	// PB2 - OC1B		- PCINT2
  pinMode(11, INPUT); 	// PB3 - MOSI / OC2	- PCINT3
  pinMode(12, INPUT); 	// PB4 - MISO		- PCINT4
  pinMode(13, INPUT); 	// PB5 - SCK		- PCINT5
  //					// PB6 -    		- PCINT6
  //					// PB7 -    		- PCINT7


  // PCINT0_vect = interrupt vector for external interrupt on pin PCINT  0..7   WHEEL2
  // PCINT1_vect = interrupt vector for external interrupt on pin PCINT  8..14  NOT USED for wheel Encoders
  // PCINT2_vect = interrupt vector for external interrupt on pin PCINT 16..23  WHEEL1

  PCMSK2 	= _BV(PCINT18);	// | _BV(PCINT19);
  PCMSK0 	= _BV(PCINT0);	// | _BV(PCINT19);

  PCICR	= _BV(PCIE2) | _BV(PCIE0);
}


// Necessary - DO NOT REMOVE
ISR(PCINT2_vect){
  /* debugging
   	
   	if((PINB & B00000001) == 0){
   		Serial.print("3l ");
   	}else{
   		Serial.print("3h ");
   	}
   	if((PINB & B00000010) == 0){
   		Serial.println("4l");
   	}else{
   		Serial.println("4h");
   	}
   	//*/

  if((PIND & B00000100) == 0 && (PIND & B00001000) == 0){
    wheel_1++;
  }
  else if((PIND & B00000100) > 1 && (PIND & B00001000) > 1){
    wheel_1++;
  }
  else{
    wheel_1--;
  }

  //Serial.printf("1: %d\n",_buffer.map.wheel_1_speed);
}


// Necessary - DO NOT REMOVE
ISR(PCINT0_vect){
  /* debugging
   	
   	if((PINB & B00000001) == 0){
   		Serial.print("8l ");
   	}else{
   		Serial.print("8h ");
   	}
   	if((PINB & B00000010) == 0){
   		Serial.println("9l");
   	}else{
   		Serial.println("9h");
   	}
   	//*/

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
