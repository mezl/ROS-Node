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

MPU6050 mpu;


#ifdef ESP8266
#define INTERRUPT_PIN 15  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 0 // (Arduino is 13, Teensy is 11, Teensy++ is 6,Pro Micro RXLED 17)
#define FEATHER
#else
#define INTERRUPT_PIN 0  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6,Pro Micro RXLED 17)
#endif

#ifdef FEATHER
  #include <Adafruit_FeatherOLED.h>
#endif

//#define OUTPUT_READABLE_QUATERNION
//#define SERIAL_DEBUG //instead output dmp data, print readable from serial

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
VectorInt16 gyro;       // [x, y, z]            world-frame accel sensor measurements
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
    //Serial.println(F("DMP Ready..."));
}


#ifdef FEATHER
Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();
#endif
float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void printLcd(const __FlashStringHelper * str, bool clear=false)
{
  #ifdef FEATHER
  // Initialize oled
  if(clear){
    oled.clearDisplay();
    oled.setCursor(0,0);
  }
  oled.println(str);      
  oled.display();
  ESP.wdtFeed();//reset watch dog
  #endif  
  
}
void setup() 
{
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, true);
  
  Serial.begin(115200);
  
  #ifdef __AVR_Atmega32U4__ // Yun 16Mhz, Micro, Leonardo, Esplora
    //while (!Serial) ;
  #endif
  /*
  Since the Micro, like the Leonardo, uses software USB instead of a hardware USB-to-Serial adapter you need to add some code:

  http://arduino.cc/en/Guide/ArduinoLeonardo

  "This change means that if you're using any Serial print(), println() or write() statments in your setup, 
  they won't show up when you open the serial monitor. 
  To work around this, you can check to see if the serial port is open after calling Serial.begin():
  */
  

  #ifdef FEATHER
  // Initialize oled
  oled.init();
  oled.setBatteryVisible(false);
  oled.clearDisplay();
  oled.setCursor(0,0);
  oled.println("Initializing I2C...");      
  oled.display();
  
  #endif
  // Initialize MPU6050
    Serial.println(F("HELLO I2C devices..."));
    Serial.println(F("Initializing I2C..."));
    
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));

    if(mpu.testConnection()){
      Serial.println(F("MPU6050 connection successful")); 
      printLcd(F("MPU6050 successful"));
      }
    else{
      while(1){
        Serial.println(F("MPU6050 connection failed"));
        printLcd(F("MPU6050 failed"));        
        delay(1000);
       }
      }







    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    printLcd(F("Initializing DMP..."));
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
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);//reenable Interrupt for now
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


}
void loop()
{
   // blink LED to indicate activity
   blinkState = !blinkState;
   digitalWrite(LED_PIN, blinkState);
   
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    int waitCnt = 1;
    while (!mpuInterrupt && fifoCount < packetSize) {
      //detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));//disable Interrupt for now
      if(waitCnt % 30000 == 0){
        printLcd(F("Waiting DMP."),true);

        
      }else if(waitCnt % 60000 == 0){
        printLcd(F("Waiting DMP...."),true);

      }
      waitCnt++;
      if(waitCnt >60000) waitCnt = 0;
      //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);//reenable Interrupt for now
      
      
 
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

        #ifdef OUTPUT_READABLE_QUATERNION
        
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGyro(&gyro, fifoBuffer); //x,y,z
            float ctemp = (temperature/340.0) + 36.53;
            #ifdef FEATHER
            oled.clearDisplay();            
            oled.setCursor(0,0);
            oled.print("q");
            oled.print(q.w);
            oled.print("\t");
            oled.print(q.x);
            oled.print("\t");
            oled.print(q.y);
            oled.print("\t");
            oled.println(q.z);
            oled.print("G:");         
            oled.print(gyro.x);
            oled.print("\t");
            oled.print(gyro.y);
            oled.print("\t");
            oled.println(gyro.z);
            oled.print("Temp:");
            oled.print(ctemp);
            oled.println(" C");            
            oled.display();
            #endif
            #ifdef SERIAL_DEBUG
            Serial.print("q");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
            Serial.print("G:");         
            Serial.print(gyro.x);
            Serial.print("\t");
            Serial.print(gyro.y);
            Serial.print("\t");
            Serial.println(gyro.z);
            Serial.print("Temp:");
            Serial.print(ctemp);
            Serial.println(" C");            
            
            #endif

            
        #endif


    }

 }


