// ################################################################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ################################################################

// if give an error about BUFFER_LENGTH on compilation
// on library file MPU6050.cpp, insert the following line
//#define BUFFER_LENGTH 32
// at the top or just before the following line (near 2751)
//int8_t MPU6050::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof

// ################################################################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ################################################################

#define M_PI 3.14159265358979323846

#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// ==================================================

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

static int i2cCore = 1;

#define INTERRUPT_PIN_MPU 13
#define LED_PIN 33 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

volatile bool mpuDataReady = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

volatile int mpuDataCounter = 0;
int mpuDataCounterPrev = 0;

bool blinkState = false;

void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

void sensorTask( void * pvParameters ) {

  // ================== SETUP ==================

  delay(100);

  String taskMessage = "sensorTask running on core ";
  taskMessage = taskMessage + xPortGetCoreID();

  Serial.println(taskMessage);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(14, 15, 4000000);

  delay(1000);

  // initialize device
  Serial.println(F("First MPU6050 initialization ..."));
  mpu.initialize();

  delay(100);

  mpu.reset(); //help startup reliably - doesn't always work though.
  // maybe can also reset i2c on esp32?
  Serial.println(F("MPU6050 reset..."));

  delay(100);

  mpu.resetI2CMaster();
  Serial.println(F("MPU6050 resetI2CMaster..."));

  delay(100);

  // initialize device
  Serial.println(F("Final MPU6050 initialization..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN_MPU, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for someone else's test chip
  //  mpu.setZAccelOffset(0); // 1688 factory default for someone else's test chip


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (mcu external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_MPU));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100);

  // ================== LOOP ==================

  while (true) {
    //if (connected) {

      // if programming failed, don't try to do anything
      if (!dmpReady) {
        Serial.println("dmpNotReady");
        delay(100);
      }

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        //
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // full -180 to 180 deg Pitch, but other od behavior
        // mpu.dmpGetYawPitchRollBeng27(ypr, &q, &gravity);
        
        // for different board orientation. But Yaw doesn't work properly with this.
        // mpu.dmpGetYawPitchRollVertical(ypr, &q, &gravity);

        mpuDataCounter++;

        // blink LED to indicate activity - need to move this elsewhere
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
      }
  } // end of loop
} // end sensorTask


void setup() {
  // initialize serial communication
  Serial.begin(115200);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  delay(100);

  Serial.print("Creating i2c task on core ");
  Serial.println(i2cCore);
  xTaskCreatePinnedToCore(
    sensorTask,   /* Function to implement the task */
    "coreTask", /* Name of the task */
    10000,      /* Stack size in words */
    NULL,       /* Task input parameter */
    20,          /* Priority of the task */
    NULL,       /* Task handle. */
    i2cCore);  /* Core where the task should run */

  Serial.println("i2c task created.");
}

void loop() {
    if (mpuDataCounter != mpuDataCounterPrev) {

      Serial.print(" mpu ypr ");
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      mpuDataCounterPrev = mpuDataCounter;
    }
}
