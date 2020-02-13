
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

//Set button pins
int button1 = 4;
int button2 = 5;

//storage values for gestures and buttons
int gesture = 0;
bool b1Engaged = 0;
bool b2Engaged = 0;

//Set up array for leds
CRGB leds[NUM_LEDS];

//Define the values for the chords
int major[3] = {0, 4, 7};
int minor[3] = {0, 3, 7};
int aug[3] = {0, 4, 8};

//Define the values for adding a 4th note to make a seventh chord
int major7 = 11;
int minor7 = 10;

//Chords when major chord is reflected along a tonnetz grid line
int majorleft[3] = {0, 4, 9};
int majorright[3] = {4, 7, 11};

//Chords when minor chord is reflected along a tonnetz grid line
int minorleft[3] = {0, 3, 8}; 
int minorright[3] = {3, 7, 10};

//Chords when the augmented chord is flipped
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
// ===                       Function                           ===
// ================================================================



//Function for writing a color to all leds
void setcolors(CRGB color){
  for(int i = 0; i<NUM_LEDS; i++){
    leds[i] = color;
  }
  FastLED.show();
}

//Function for writing a color to the first led (front facing)
void setcolor1(CRGB color){
  leds[0] = color;
  FastLED.show();
}

//Read a value from a button
bool readbutton(int button){
  int buttonValue = digitalRead(button);

  //Truth values flipped because button should be pulled high
  if (buttonValue == LOW){
    return true;
  }
  else{
    return false;
  }
}


// checks if the w value has not changed too much
bool checkstable(float quatw){

  //Get the difference
  float set = lastw-quatw;

  //Check if it's within acceptable margins
  if(set > -0.01 & set < 0.01){
   return true;
  }

  //otherwise update past values
  else{
    lastw = quatw;
    return false;
  }
}


/*
 * Maps up, down, left, and right gestures to a state
 */
int checkgesture(float quatx, float quaty){

  //tilt right
  if(quatx > 0.40) return 3;

  //tilt left
  if(quatx < -0.40) return 4;

  //tilt up
  if(quaty < -0.40) return 2;

  //tilt down
  if(quaty > 0.40) return 1;

  //Neutral position
  return 0;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    /*
     * I2C set up Code from https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
     */
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Set up midi channel
    MIDI.begin(MIDI_CHANNEL_OMNI);

    //Begin Serial
    Serial.begin(115200);

    //Something from https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
    //Not sure what it does, but looks like it might break something if removed
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    //Set up LEDS
    FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);


    /*
     * MPU set up code from https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
     */
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

    // configure Buttons for input
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
    //from https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
    if (!dmpReady) return;

    //Check if there is a message on the radio
    if (radio.available()) {
      radio.read(&message, sizeof(message));
    }

    /*
     * More MPU set up code from https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
     */
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



          //If the accelerometer valueshas stabilized
          if(stable){

            //Check which gesture the ball is in
            gesture = checkgesture(quat.x, quat.y);
            
            
            //Set colors accord to the ball gesture
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

            //Set up chord array
            int notes[4];

            //For the first three notes in the chord
            for(int i=0; i<3; i++){

              //If the Wand has a pitch of upright
              if(message.stateOne == 0){

                //And the ball is in gesture 3
                if(gesture == 3){

                  //Write major reflected right to the chord array
                  notes[i] = majorright[i];
                }
                else{

                  //Else if ball in gesture 4
                  if(gesture == 4){

                    //Write major reflected left to the chord array
                    notes[i] = majorleft[i];
                  }

                  //Otherwise right the major 
                  else{
                    notes[i] = major[i];
                  }
                }           
              }

              //If the Wand is at 45 degree angle
              else{
                if(message.stateOne == 1){

                  //And the ball is in gesture 3
                  if(gesture == 3){
                    
                    //Write minor reflected right to the chord array
                    notes[i] = minorright[i];
                  }
                  else{

                    //Else if ball in gesture 4
                    if(gesture == 4){

                      //Write minor reflected left to the chord array
                      notes[i] = minorleft[i];
                    }
                    else{

                      //Otherwise right the minor 
                      notes[i] = minor[i];
                    }
                  } 
                }

                //If the Wand is at horizontal angle
                else{

                  //If there is a left or right gesture on the ball
                  if(gesture == 3 or gesture == 4){

                    //Write the augmented chord flipped to the chord array
                    notes[i] = augflipped[i];
                  }
                  else{

                    //Otherwise write the augmented chord to the chord array
                    notes[i] = aug[i];
                  }
                }
              }
            }

            //If the ball is in gesture 2
            if(gesture == 2){

              //Add the major 7th to the chord array
              notes[3] = major7;
            }

            //if the ball is in gesture 1
            else{
              if(gesture == 1){

                //Add the minor 7th to the chord array
                notes[3] = minor7;
              }
              else{

                //Otherwise add the first note again (effectively null note)
                notes[3] = 0;
              }
            }

          //if the first button is pressed
          if(readbutton(button1)){

            //If it isn't held
            if(b1Engaged == 0){

              //Play the store array 
              //If the roll is to the left on the wand, shift to f chords
              if(message.stateTwo == 1){
                playchord(notes, f, 60);
              }
              else{
                //If the roll is to the right on the wand, shift to g chords
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

          //Same as for button one, just transposed to the bass clef
          if(readbutton(button2)){

            //If it isn't held
            if(b2Engaged == 0){

              //Play the store array 
              //If the roll is to the left on the wand, shift to f chords
              if(message.stateTwo == 1){
                playchord(notes, f, 48);
              }
              else{
                //If the roll is to the right on the wand, shift to g chords
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

        
      }

      //Show that the accelerometer is not stable
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

//Play a chord
//Chord a 4 note array to be played
//Offset depends on if you want to shift the from C to exmaple F
//base is the base offset to get you to your base note (for example C4)
void playchord(int chord[], int offset, int base){
  for(int i = 0; i<4; i++){
    MIDI.sendNoteOff(prevNote[i], velocity, 1);
    MIDI.sendNoteOn(base+offset+chord[i], velocity, 1);
    prevNote[i] = base+offset+chord[i];
  }
}
