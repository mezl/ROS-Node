/*
    Kalman Filter Example for MPU6050. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/


#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <KalmanFilter.h>
#include <Adafruit_FeatherOLED.h>
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 0 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup() 
{
  Serial.begin(115200);

  // Initialize oled
  oled.init();
  oled.setBatteryVisible(true);
  // Initialize MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-1169);
    mpu.setYAccelOffset(744);
    mpu.setZAccelOffset(1620);
    mpu.setXGyroOffset(48);
    mpu.setYGyroOffset(47);
    mpu.setZGyroOffset(-8);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
}

void loop()
{

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        oled.clearDisplay();
        oled.setCursor(0,0);
        oled.print("G:");
        oled.println();

      
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
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

        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        // gyro values
        teapotPacket[10] = fifoBuffer[16];
        teapotPacket[11] = fifoBuffer[17];
        teapotPacket[12] = fifoBuffer[20];
        teapotPacket[13] = fifoBuffer[21];
        teapotPacket[14] = fifoBuffer[24];
        teapotPacket[15] = fifoBuffer[25];
        // accelerometer values
        teapotPacket[16] = fifoBuffer[28];
        teapotPacket[17] = fifoBuffer[29];
        teapotPacket[18] = fifoBuffer[32];
        teapotPacket[19] = fifoBuffer[33];
        teapotPacket[20] = fifoBuffer[36];
        teapotPacket[21] = fifoBuffer[37];
        //temperature
        int16_t temperature = mpu.getTemperature();
        teapotPacket[22] = temperature >> 8;
        teapotPacket[23] = temperature & 0xFF;
        Serial.write(teapotPacket, 28);
        teapotPacket[25]++; // packetCount, loops at 0xFF on purpose

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

/*

  // clear the current count

  oled.print(int(accPitch));
  oled.print(":");
  oled.print(int(accRoll));
  oled.print(":");
  oled.print(int(kalPitch));
  oled.print(":");
  oled.println(int(kalRoll));
  
  oled.print("A:");
  oled.print(int(acc.XAxis));
  oled.print(":");
  oled.print(int(acc.YAxis));
  oled.print(":");
  oled.println(int(acc.ZAxis));
  
  oled.print("G:");
  oled.print(int(gyr.XAxis));
  oled.print(":");
  oled.print(int(gyr.YAxis));
  oled.print(":");
  oled.println(int(gyr.ZAxis));
  
  oled.display();



  Serial.print(accPitch);
  Serial.print(":");
  Serial.print(accRoll);
  Serial.print(":");
  Serial.print(kalPitch);
  Serial.print(":");
  Serial.print(kalRoll);
  Serial.print(":");
  Serial.print(acc.XAxis);
  Serial.print(":");
  Serial.print(acc.YAxis);
  Serial.print(":");
  Serial.print(acc.ZAxis);
  Serial.print(":");
  Serial.print(gyr.XAxis);
  Serial.print(":");
  Serial.print(gyr.YAxis);
  Serial.print(":");
  Serial.print(gyr.ZAxis);

  Serial.println();
*/
/*

*/
  // update the display with the new count
  //oled.display();
  // delay 1 second (1 second == 1000 milliseconds)
  //delay(1000);  
}


