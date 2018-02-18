// Setup taken from: https://diyhacking.com/build-arduino-self-balancing-robot/

// --------------PID-------------------
#include "PID_v1.h"


// --------------MOTOR CONTROL-------------------
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"


// --------------IMU-------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


// --------------DEBUGGING-------------------
// Uncomment to see Telemetry
#define LOG_TELEM
// Uncomment to output serial plot variables
//#define SERIAL_PLOT

long loop_count = 0;
long overflows = 0;


//--------------PID SETPOINTS-------------------
double originalSetpoint = 88.0;  // what angle [DEG] is the IMU when bot is balanced?
double setpoint = originalSetpoint;
//double movingAngleOffset = 0.1;
double input, output;
//int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 36; //50 > Kp > 35  //50 //30
double Kd = 0; // 1 > Kd > 0
double Ki = 0;
//double Kp = 35;
//double Kd = 5;
//double Ki = 60;
uint16_t pid_smpl_time_ms = 15;  // How often, in milliseconds, the PID will be evaluated. (originally set to 15)
double pid_speed_max = 255;
double pid_speed_min = -255;
double min_mtr_speed = 40;
double max_mtr_speed = 255;
double set_speed = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);


// --------------MOTOR CONTROL-------------------
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *left_motor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);


// --------------MPU DATA/STATUS VARS-------------------
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float Yaw, Pitch, Roll; // in degrees

//                       XA      YA     ZA     XG     YG      ZG
int MPUOffsets[6] = {  -2423,  -806,   888,    136,   160,   -110}; //MPU6050 on balanceing bot




// +-+++-------+++++----------+++++++_-0000-------------
// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering


uint8_t directionality;
bool motor_toggle = true; // true=left;  false = right
// +-+++-------+++++----------+++++++_-0000-------------





// --------------PIN DEFINITIONS-------------------
#define INTERRUPT_PIN 2  // pin 2 as suggested in the forums





volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


void setup()
{

    // initialize serial communication
    Serial.begin(115200);
    while(!Serial);


    Serial.println(F("Initializing the IO pins..."));
    init_IO();
    

    Serial.println(F("Joining I2C bus..."));
    i2cSetup();


    Serial.println(F("Initializing MPU6050..."));
    MPU6050Connect();


//    Serial.println(F("Initializing PID..."));
    pid_setup();


    Serial.println(F("Initializing Motorboard..."));
    motor_shield_setup();



    Serial.println(F("Finished setup..."));
    
}


void loop() {
  if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
    GetDMP(); // Gets the MPU Data and canculates angles

    if (Pitch > 160  || Pitch < 10){
      Serial.print(F("ERROR. Out of Bounds\n"));
      set_speed = 0;
    }
    else
    {
      pid_control();
    }

    if (motor_toggle)
    {
      control_left();
      motor_toggle = false;
    }
    else{
      control_right();
      motor_toggle = true;
    }
  }

  static long QTimer = millis();
  if ((long)( millis() - QTimer ) >= 20) {
    QTimer = millis();

    #ifdef LOG_TELEM
        Serial.print(F("\t Yaw ")); Serial.print(Yaw);
        Serial.print(F("\t Pitch ")); Serial.print(Pitch);
        Serial.print(F("\t Roll ")); Serial.print(Roll);
        Serial.print(F("\t error: ")); Serial.print(Pitch - setpoint);
        Serial.print(F("\t output: ")); Serial.print(output);
        Serial.print(F("\t set_speed: ")); Serial.print(set_speed);
        Serial.println();
    #endif


    #ifdef SERIAL_PLOT
        Serial.print(input); 
        Serial.print(",");
        Serial.println(setpoint);
    #endif

    
  }
}




void init_IO()
{
  pinMode(INTERRUPT_PIN, INPUT);
}

// ================================================================
// ===                      init the IMU                        ===
// ================================================================
void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }

  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection

  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  attachInterrupt(0, dmpDataReady, FALLING); //pin 2 on the Uno

  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt

}

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        Wire.setClock(400000);
        boolean error = true;
        while (error) 
        {
          Wire.beginTransmission(0x68);
          error = Wire.endTransmission(); // if error = 0, we are properly connected
          if (error) 
          { // if we aren't properly connected, try connecting again and loop
            Serial.println("  ");
            Serial.println("Not properly connected to I2C, trying again");
            Serial.println(" ");
            Wire.begin();
            TWBR = 24; // 400kHz I2C clock
          }
        }
        Serial.println("Properly connected to I2C");
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}


// ================================================================
// ===                   motor Shield Setup                     ===
// ================================================================
void motor_shield_setup() {
    AFMS.begin();  // create with the default frequency 1.6KHz
    // Set the speed at start to 0(off) so it doesn't go running off. Max speed is 255
    left_motor->setSpeed(0);
    left_motor->run(RELEASE);
    right_motor->setSpeed(0);
    right_motor->run(RELEASE);
}



void pid_setup() {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(pid_smpl_time_ms);
    pid.SetOutputLimits(pid_speed_min, pid_speed_max);  // Motors' top speeds
}

// ================================================================
// ===        read angle values from the DMP on the IMU         ===
// ================================================================
void GetDMP() {
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  /*
  fifoCount is a 16-bit unsigned value. Indicates the number of bytes stored in the FIFO buffer.
  This number is in turn the number of bytes that can be read from the FIFO buffer and it is
  directly proportional to the number of samples available given the set of sensor data bound
  to be stored in the FIFO
  */

  // PacketSize = 42; refference in MPU6050_6Axis_MotionApps20.h Line 527
  // FIFO Buffer Size = 1024;
  uint16_t MaxPackets = 20;// 20*42=840 leaving us with  2 Packets (out of a total of 24 packets) left before we overflow.
  // If we overflow the entire FIFO buffer will be corrupt and we must discard it!

  // At this point in the code FIFO Packets should be at 1 99% of the time if not we need to look to see where we are skipping samples.
  if ((fifoCount % packetSize) || (fifoCount > (packetSize * MaxPackets)) || (fifoCount < packetSize)) { // we have failed Reset and wait till next time!
//    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    Serial.println(F("Reset FIFO"));
    if (fifoCount % packetSize) Serial.print(F("\t Packet corruption")); // fifoCount / packetSize returns a remainder... Not good! This should never happen if all is well.
    Serial.print(F("\tfifoCount ")); Serial.print(fifoCount);
    Serial.print(F("\tpacketSize ")); Serial.print(packetSize);

    mpuIntStatus = mpu.getIntStatus(); // reads MPU6050_RA_INT_STATUS       0x3A
    Serial.print(F("\tMPU Int Status ")); Serial.print(mpuIntStatus , BIN);
    // MPU6050_RA_INT_STATUS       0x3A
    //
    // Bit7, Bit6, Bit5, Bit4          , Bit3       , Bit2, Bit1, Bit0
    // ----, ----, ----, FIFO_OFLOW_INT, I2C_MST_INT, ----, ----, DATA_RDY_INT

    /*
    Bit4 FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated.
    Bit3 I2C_MST_INT: This bit automatically sets to 1 when an I2C Master interrupt has been generated. For a list of I2C Master interrupts, please refer to Register 54.
    Bit1 DATA_RDY_INT This bit automatically sets to 1 when a Data Ready interrupt is generated.
    */
    if (mpuIntStatus & B10000) { //FIFO_OFLOW_INT
      Serial.print(F("\tFIFO buffer overflow interrupt "));
    }
    if (mpuIntStatus & B1000) { //I2C_MST_INT
      Serial.print(F("\tSlave I2c Device Status Int "));
    }
    if (mpuIntStatus & B1) { //DATA_RDY_INT
      Serial.print(F("\tData Ready interrupt "));
    }
    Serial.println();
    //I2C_MST_STATUS
    //PASS_THROUGH, I2C_SLV4_DONE,I2C_LOST_ARB,I2C_SLV4_NACK,I2C_SLV3_NACK,I2C_SLV2_NACK,I2C_SLV1_NACK,I2C_SLV0_NACK,
    mpu.resetFIFO();// clear the buffer and start over
    mpu.getIntStatus(); // make sure status is cleared we will read it again.
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      if (fifoCount < packetSize) break; // Something is left over and we don't want it!!!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
    if (fifoCount > 0) mpu.resetFIFO();
  }
}

// ================================================================
// ===                   PID Implementation                     ===
// ================================================================
void pid_control() {
  input = Pitch;  // we just want to react on the pitch (item index 1 in the ypr list)


  //performing PID calculations and output to motors
  pid.Compute();

  double abs_output = abs(output);
  if (isnan(output)) 
  {
      abs_output = 0.0;
  }
  // Completely arbitrary assignment. depends on which side of the robot you consider "forward".
  if (output < 0.0)
  {
    directionality = BACKWARD;
    // additionally, output needs to be converted to positive using abs(output) 
  }
  else
  {
    directionality = FORWARD;
  }

//  set_speed = max(min_mtr_speed, abs_output);
  set_speed = constrain(abs_output, min_mtr_speed, max_mtr_speed);


}

void control_left()
{
  
  left_motor->setSpeed(set_speed);
//  left_motor->setSpeed(65); // 50
  left_motor->run(directionality);
}


void control_right()
{
  right_motor->setSpeed(set_speed);
//  right_motor->setSpeed(65); // 50
  right_motor->run(directionality);
}



void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.resetFIFO(); // trying to prevent the halt
  Yaw = (ypr[0] * 180 / M_PI);
  Pitch = (ypr[1] * 180 / M_PI);
  Roll = (ypr[2] * 180 / M_PI);
}

