
/*
 * CPSC 599 - Physical and Tangible HCI
 * Sarah Walker and Colin Au Yeung
 * 
 * tonnetz ball
 * 
 * This device maps gestures from a gyroscope and an external device (recieved over radio)
 * to a musical chord. It then sends that chord as a MIDI message.
 * 
 */

#include <I2Cdev.h>
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

#include <MIDI.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#include <FastLED.h>
#define NUM_LEDS 4
#define DATA_PIN 3
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

// Radio requirements
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7,8);  // CE,CSN
const byte address[6] = "00001";

// Structure of message sent from other device
/*
 * ID == 1 : Wand
 *  stateOne is pitch action
 *  stateTwo is roll action
*/
typedef struct{
  int ID = 1;
  int stateOne = 0;
  int stateTwo = 0;
 } actionMessage;

// Message recieved from transmitter
 actionMessage message;

int button1 = 4;
int button2 = 5;

int gesture = 0;
bool b1Engaged = 0;
bool b2Engaged = 0;

CRGB leds[NUM_LEDS];

int major[3] = {0, 4, 7};
int minor[3] = {0, 3, 7};
int aug[3] = {0, 4, 8};
int major7 = 11;
int minor7 = 10;

int majorleft[3] = {0, 4, 9};
int majorright[3] = {4, 7, 11};

int minorleft[3] = {0, 3, 8}; 
int minorright[3] = {3, 7, 10};

int augflipped[3] = {0, 3, 9};

// previous note tracking - used to turn off last sent chord
int prevNote[4] = {0,0,0,0};

int f = 5; // offset for f note
int g = 7; // offset for g note

// MIDI message requirements
int velocity = 100;//velocity of MIDI notes, must be between 0 and 127
int noteON = 144; //144 = 10010000 in binary, note on command

MIDI_CREATE_DEFAULT_INSTANCE();

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
  int buttonValue = digitalRead(button);
  if (buttonValue == LOW){
    return true;
  }
  else{
    return false;
  }
}


// checks if the w value has not changed too much
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

/*
 * Maps up, down, left, and right gestures to a state
 */
int checkgesture(float quatx, float quaty){

  //tilt down
  if(quatx > 0.40) return 3;

  //tilt up
  if(quatx < -0.40) return 4;

  //tilt right
  if(quaty < -0.40) return 2;

  //tilt left
  if(quaty > 0.40) return 1;

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
    MIDI.begin(MIDI_CHANNEL_OMNI);
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);

    // initialize device
    mpu.initialize();

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
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(button1, INPUT_PULLUP);
    pinMode(button2, INPUT_PULLUP);

    // radio configuration
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    if (radio.available()) {
      radio.read(&message, sizeof(message));
  //    Serial.print(message.stateOne); Serial.print(" ");
  //    Serial.println(message.stateTwo);
    }
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
          if(count > 500 & not stable){
            stable = checkstable(quat.x);
            count = 0;
          }
          if(stable){
            gesture = checkgesture(quat.x, quat.y);
            
//            bool statecheck = state[0] != message.stateOne or state[1] != message.stateTwo or state[2] != b1Engaged or state[3] != b2Engaged  or state[4] != gesture;
//            if(statecheck){
//              for(int i = 48; i<72; i++){
//                MIDImessage(noteON, i, 0);
//              }
//              state[0] = message.stateOne;
//              state[1] = message.stateTwo;
//              state[2] = b1Engaged;
//              state[3] = b2Engaged;
//              state[4] = gesture;
//            }
            
            
            if(gesture == 1){
              setcolors(CRGB::Blue);
            }
            if(gesture == 2){
              setcolors(CRGB::Red);
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

            int notes[4];
              
            for(int i=0; i<3; i++){
              if(message.stateOne == 0){
                if(gesture == 3){
                  notes[i] = majorright[i];
                }
                else{
                  if(gesture == 4){
                    notes[i] = majorleft[i];
                  }
                  else{
                    notes[i] = major[i];
                  }
                }           
              }
              else{
                if(message.stateOne == 1){
                  if(gesture == 3){
                    notes[i] = minorright[i];
                  }
                  else{
                    if(gesture == 4){
                      notes[i] = minorleft[i];
                    }
                    else{
                      notes[i] = minor[i];
                    }
                  } 
                }
                else{
                  if(gesture == 3 or gesture == 4){
                    notes[i] = augflipped[i];
                  }
                  else{
                    notes[i] = aug[i];
                  }
                }
              }
            }
            if(gesture == 2){
              notes[3] = major7;
            }
            else{
              if(gesture == 1){
                notes[3] = minor7;
              }
              else{
                notes[3] = 0;
              }
            }

    
          if(readbutton(button1)){
            if(b1Engaged == 0){
              if(message.stateTwo == 1){
                playchord(notes, f, 60);
              }
              else{
                if(message.stateTwo == 2){
                  playchord(notes, g, 60);
                }
                else{
                  playchord(notes, 0, 60);
                }
              }
            }
            b1Engaged = 1;
            setcolor1(CRGB::Green);
            
          }
          else
          {
            b1Engaged = 0;
          }

            
          if(readbutton(button2)){
            if(b2Engaged == 0){
               if(message.stateTwo == 1){
              playchord(notes, f, 48);
            }
            else{
              if(message.stateTwo == 2){
                playchord(notes, g, 48);
              }
              else{
                playchord(notes, 0, 48);
              }
            }
            }
            b2Engaged = 1;
            setcolor1(CRGB::Red);
          }
          else
          {
            b2Engaged = 0;
          }

        // blink LED to indicate activity
        
      }
      if(not stable){
        setcolor1(CRGB::Green);
      }
      count = count + 1;
  }
}

//send MIDI message
void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
  Serial.write(command);//send note on or note off command 
  Serial.write(MIDInote);//send pitch data
  Serial.write(MIDIvelocity);//send velocity data
}

void playchord(int chord[], int offset, int base){
  for(int i = 0; i<4; i++){
    MIDI.sendNoteOff(prevNote[i], velocity, 1);
    MIDI.sendNoteOn(base+offset+chord[i], velocity, 1);
    prevNote[i] = base+offset+chord[i];
  }
}
