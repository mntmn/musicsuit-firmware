// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif

#include <SPI.h>
#include "RF24.h"

bool radioNumber = 1;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpuA(0x68);
MPU6050 mpuB(0x69); // <-- use for AD0 high
#define NUM_MPUS 2

MPU6050* mpus[NUM_MPUS] = {&mpuA,&mpuB};

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   * ========================================================================= */

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCountA;
uint16_t fifoCountB;

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

#define NOT_AN_INTERRUPT -1

RF24 radio(7,8);
byte addresses[][6] = {"1Node","2Node"};

void setup() {
  Serial.begin(115200);
    
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  
  radio.write("I2C 400khz\r\n",13);
  Fastwire::setup(400, true);
  
  //pinMode(INTERRUPT_PIN, INPUT);
  // supply your own gyro offsets here, scaled for min sensitivity
  /*mpuA.setXGyroOffset(220);
  mpuA.setYGyroOffset(76);
  mpuA.setZGyroOffset(-85);
  mpuA.setZAccelOffset(1788); // 1688 factory default for my test chip
  */
  // make sure it worked (returns 0 if so)
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  //mpuIntStatus = mpu.getIntStatus();
  // get expected DMP packet size for later comparison


  pinMode(LED_PIN, OUTPUT);
  
  for (int i=0; i<NUM_MPUS; i++) {
    MPU6050* mpu = mpus[i];
    mpu->initialize();
    devStatus = mpu->dmpInitialize();
    
    mpu->setXGyroOffset(220);
    mpu->setYGyroOffset(76);
    mpu->setZGyroOffset(-85);
    mpu->setZAccelOffset(1788);
    
    mpu->setDMPEnabled(true);
    packetSize = mpu->dmpGetFIFOPacketSize();
  }
}

char outbuf[32];
int tick=0;

void loop() {
  for (int i=0; i<NUM_MPUS; i++) {
    MPU6050* mpu = mpus[i];
    int fifoCount = mpu->getFIFOCount();
    if (fifoCount >= 1024) {
      mpu->resetFIFO();
    } else {
      while (fifoCount < packetSize) {
        fifoCount = mpu->getFIFOCount();
      }

      mpu->getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu->dmpGetQuaternion(&q, fifoBuffer);
      mpu->dmpGetGravity(&gravity, &q);
      mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
      memset(outbuf,0,32);
      sprintf(outbuf,"%d\t%d\t%d\t%d\r\n",
              i,
              int(100 * ypr[0] * 180/M_PI),
              int(100 * ypr[1] * 180/M_PI),
              int(100 * ypr[2] * 180/M_PI));
      radio.write(outbuf, 32);
    }
  }
}
