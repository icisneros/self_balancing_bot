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
// Uncomment to see MPU values
#define LOG_INPUT
// Uncomment to see Telemetry
//#define LOG_TELEM
// Uncomment to output serial plot variables
//#define SERIAL_PLOT

long loop_count = 0;
long overflows = 0;


//--------------PID SETPOINTS-------------------
double originalSetpoint = 87.7;  // what angle [DEG] is the IMU when bot is balanced?
double setpoint = originalSetpoint;
//double movingAngleOffset = 0.1;
double input, output;
//int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 50; //50 > Kp > 35  //50 //34
double Kd = 0; // 1 > Kd > 0
double Ki = 0;
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



// --------------PIN DEFINITIONS-------------------
#define INTERRUPT_PIN 2


void init_IO()
{
  pinMode(INTERRUPT_PIN, INPUT);
}


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

    // pinmode init
    init_IO();
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Serial.println(F("Joining I2C bus..."));
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



    // initialize devices
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();


    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(2);
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-2423);
    mpu.setYAccelOffset(-806);
    mpu.setZAccelOffset(888);
    mpu.setXGyroOffset(136);
    mpu.setYGyroOffset(160);
    mpu.setZGyroOffset(-110);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        
        Serial.println(F("Initializing PID..."));
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(15);
        pid.SetOutputLimits(-255, 255);  


        Serial.println(F("Initializing Motorboard..."));
        AFMS.begin();  // create with the default frequency 1.6KHz
        // Set the speed at start to 0(off) so it doesn't go running off. Max speed is 255
        left_motor->setSpeed(0);
        left_motor->run(RELEASE);
        right_motor->setSpeed(0);
        right_motor->run(RELEASE);
    

        Serial.println(F("Finished setup..."));




    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        Serial.println(F("Initiating halt..."));
        while(1);
    }




}


void loop()
{
    
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
      Serial.print(F("An error occured")); 
      return;
    }
    
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // nothing here
    }


    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();


//    unsigned long lastPrint = millis();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        overflows += 1;
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
          fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGyro(gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


        // Update the input variable (and convert from RAD to DEG)
        input = ypr[1] * 180/M_PI;  // we just want to react on the pitch (item index 1 in the ypr list)



        // Default direction. Use when output > 0. Completely arbitrary.
        uint8_t directionality = FORWARD;


        //no mpu data - performing PID calculations and output to motors
        pid.Compute();


        double abs_output = output;
//        if (isnan(output)) 
//        {
//            abs_output = 0.0;
//        }
        if (output < 0)
        {
          directionality = BACKWARD;
          abs_output = abs(output);
        }


        left_motor->run(directionality);
        right_motor->run(directionality);
        left_motor->setSpeed(abs_output);
        right_motor->setSpeed(abs_output);


//        if (millis() - lastPrint > 200UL)  // print status every 200ms
//        {
//          lastPrint = millis();
//          Serial.print(F("mpuInterrupt: ")); Serial.print(mpuInterrupt);
//          Serial.print(F("    fifoCount: ")); Serial.print(fifoCount);
//          Serial.print(F("    packetSize: ")); Serial.println(packetSize);
//        }

        #ifdef LOG_INPUT
            Serial.print("\nloop_count: ");
            Serial.println(loop_count);
            Serial.print("overflow count: ");
            Serial.println(overflows);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);  // yaw
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);  // pitch
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI); // roll
        #endif


        #ifdef LOG_TELEM
          Serial.print("\nloop_count: ");
          Serial.println(loop_count); 
          Serial.print("PID Output: ");
          Serial.println(output); 
          Serial.print("PID Input: ");
          Serial.println(input); 
          Serial.print("Direction: ");
          Serial.println(directionality);
          Serial.print("Speed: ");
          Serial.println(abs_output); 
        #endif



        #ifdef SERIAL_PLOT
          Serial.print(input); 
          Serial.print(",");
          Serial.println(setpoint);
        #endif


        mpu.resetFIFO();  // To prevent overflows

   }

   loop_count += 1;
}






