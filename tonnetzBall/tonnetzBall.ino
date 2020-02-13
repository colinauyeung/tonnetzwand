#include <I2Cdev.h>

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#include <FastLED.h>
#define NUM_LEDS 4
#define DATA_PIN 3

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */





#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer



// orientation/motion vars
Quaternion quat;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int count = 0;
bool stable = false;
float lastw = 0;


// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7,8);  // CE,CSN

// Radio message components
const byte address[6] = "00010";

// Structure of message sent to reciever
/*
 * ID == 2 : Ball
 *  stateOne is gesture 0-4, NOTE - 5 is unstable
 *  stateTwo is button(s) pressed (1:b1, 2:b2, 3:b1&b2)
*/
typedef struct{
  int ID = 2;
  int stateOne = 0;
  int stateTwo = 0;
 } actionMessage;

// Message sent to reciever
 actionMessage message;

int button1 = 4;
int button2 = 5;

int gesture = 0;
bool b1Engaged = false;
bool b2Engaged = false;

CRGB leds[NUM_LEDS];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                       Methods                            ===
// ================================================================


/*
 * Sets the messages structures values to indicate
 * gesture and button press
 */
void setRadioMessage()
{
  if(!stable)
  {
    message.stateOne = 5;
  }
  else
  {
    message.stateOne = gesture;
    //if(b1Engaged && b2Engaged)
    //{
      //message.stateTwo = 3;
    //}
    if(b1Engaged)
    {
      message.stateTwo = 1;
    }
    //else if(b2Engaged)
    //{
      //message.stateTwo = 2;
    //}
    else
    {
      message.stateTwo = 0;
    }
  }
}

void sendRadioMessage()
{
  if(radio.available())
  {
    radio.write(&message,sizeof(message));
  }
}


void setcolors(CRGB color){
  for(int i = 0; i<NUM_LEDS; i++){
    leds[i] = color;
  }
  FastLED.show();
}

void setcolor1(CRGB color){
  leds[0] = color;
  FastLED.show();
}

bool readbutton(int button){
  int buttonValue = digitalRead(button1);
  if (buttonValue == LOW){
    return true;
  }
  else{
    return false;
  }
}


//checks if the w value has not changed too much
bool checkstable(float quatw){
  float set = lastw-quatw;
  if(set > -0.01 & set < 0.01){
    
   return true;
  }
  else{
    lastw = quatw;
    return false;
  }
}

int checkgesture(float quatx, float quaty){

  //tilt down
  if(quatx > 0.20 & quaty > 0.20) return 1;

  //tilt up
  if(quatx < -0.20 & quaty < -0.20) return 2;

  //tilt right
  if(quatx > 0.20 & quaty < -0.20) return 3;

  //tilt left
  if(quatx < -0.20 & quaty > 0.20) return 4;

  //Neutral position
  return 0;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready


    // load and configure the DMP
    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(button1, INPUT_PULLUP);
    pinMode(button2, INPUT_PULLUP);

    // radio configuration
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

          // display quaternion values in easy matrix form: w x y z
          mpu.dmpGetQuaternion(&quat, fifoBuffer);
          //Serial.print("quat\t");
         // Serial.print(quat.w);
          //Serial.print("\t");
          //Serial.print(quat.x);
          //Serial.print("\t");
          //Serial.print(quat.y);
          //Serial.print("\t");
          //Serial.println(quat.z);
          if(count > 500 & not stable){
            stable = checkstable(quat.x);
            count = 0;
          }
          if(stable and count > 100 ){
            gesture = checkgesture(quat.x, quat.y);
            if(gesture == 1){
              setcolors(CRGB::Red);
            }
            if(gesture == 2){
              setcolors(CRGB::Blue);
            }
            if(gesture == 3){
              setcolors(CRGB::Cyan);
            }
            if(gesture == 4){
              setcolors(CRGB::Green);
            }
            if(gesture == 0){
              setcolors(CRGB::White);
            }
            Serial.print(readbutton(button1));
            Serial.print(", ");
            Serial.println(readbutton(button2));
            if(readbutton(button1)){
              Serial.print(readbutton(button1));
              b1Engaged = true;
              setcolor1(CRGB::Green);
            }
            else
            {
              b1Engaged = false;
            }
            if(readbutton(button2)){
              b2Engaged = true;
              setcolor1(CRGB::Red);
            }
            else
            {
              b2Engaged = false;
            }
          }
        // blink LED to indicate activity
        if(not stable){
          setcolor1(CRGB::Green);
        }
        count = count + 1;
    }

    setRadioMessage();
    sendRadioMessage();
}
