/*
 * CPSC 599 - Physical and Tangible HCI
 * Sarah Walker and Colin Au Yeung
 * 
 * tonnetz wand
 * 
 * This device sends a message over radio to indicate whether it is
 * in an upward, mid, or downward position, and whether it has been twisted
 * 
 * LEDs are used to indicate the position states:
 * Pitch:
 *  Upwards: Green
 *  Mid: blue
 *  Down: pink
 *  
 * Roll: 
 *  Red if twisted to either left or right side
 *  
 *  
 * Radio reciever (external device) is called tonnetzBall and maps
 * this devices pitch and roll states to a musical chord
 */

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7,8);  // CE,CSN

#include <SoftwareSerial.h>
#include <Nunchuk.h>
#include <Wire.h>
#include "nunchuk.h"

#include <FastLED.h>
#define NUM_LEDS 4
#define DATA_PIN 3

// Radio message components
const byte address[6] = "00001";

// Structure of message sent to reciever
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

// Message sent to reciever
 actionMessage message;


// Read values from sensor
double pitchVal;
double rollVal;

// Actions determined by value thresholds
int pitchAction = 0;
int rollAction = 0;

int action = 0;

CRGB leds[NUM_LEDS];



/*
 * Sets the leds to show pitch and roll action
 * 
 * Pitch:
 *  Upwards: Green
 *  Mid: blue
 *  Down: pink
 *  
 * Roll: 
 *  Red if twisted to either left or right side
 */
void setcolors(int pitch, int roll){
  if(roll == 0){
    leds[3] = CRGB::White; 
  }
  else{
    leds[3] = CRGB::Green; 
  }
  if(pitch == 0){
    for(int i = 0; i<NUM_LEDS-1; i++){
      leds[i] = CRGB::Red;
    }
  }
  else{
   if(pitch == 1){
      for(int i = 0; i<NUM_LEDS-1; i++){
        leds[i] = CRGB::Blue;
      }
    }
    else{
      for(int i = 0; i<NUM_LEDS-1; i++){
        leds[i] = CRGB::Cyan;
      }
    }
  }
  FastLED.show();
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    nunchuk_init();

    FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);    
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

void loop() {
  if (nunchuk_read()) {
      // Work with nunchuk_data
      pitchVal = nunchuk_pitch();
      rollVal = nunchuk_roll();
      
      setAction();

      // Set message
      message.stateOne = pitchAction;
      message.stateTwo = rollAction;

      Serial.println(message.ID);
      Serial.println(message.stateOne);
      Serial.println(message.stateTwo);
      Serial.println("Writing");

      //Send data over radio
      radio.write(&message,sizeof(message));
      setcolors(pitchAction, rollAction);
  }
}

// Set actions for roll and pitch
void setAction()
{  
  determinePitchAction();
  determineRollAction();  
}

/*
 * Determines the pitch of the wand and assigns relative action
 * 
 * Upwards position = 0
 * Mid position (wand ~45 degress) = 1
 * Low position (wand ~0 degrees) = 2
 */

void determinePitchAction()
{
  double upDivider = -0.65;
  double downDivider = 0.4;


  // Up position
  if(pitchVal < upDivider)
  {
    if(pitchAction != 0 && (rollVal > - 0.7) && (rollVal < 0.5))
    {
      pitchAction = 0;
    }
    
  }
  // Middle position
  else if(pitchVal > upDivider && pitchVal < downDivider)
  {
    if(pitchAction != 1)
    {
      pitchAction = 1;
    }
  }

  // Down position
  else if(pitchVal > downDivider)
  {
    if(pitchAction != 2)
    {
      pitchAction = 2;
    }
  }
}


/*
 * Determines if wand has being rolled left or right
 * and assigns an action accordingly
 * 
 * Left roll = 1
 * Right roll = 2
 * No roll = 0
 */
void determineRollAction()
{
  if(pitchAction == 0)
  {
      if( rollVal < -0.4)
      {
        rollAction = 1;
      }
      else if(rollVal > -0.10)
      {
        rollAction = 2;
      }
      else
      {
        rollAction = 0;
      }
  }
  else if(pitchAction == 1)
  {
      if(rollVal < -0.5)
      {
        rollAction = 1;
      }
      else if(rollVal > 0.7)
      {
        rollAction = 2;
      }
      else
      {
        rollAction = 0;
      }
  }
  else if(pitchAction == 2)
  {
      if(rollVal < -1.3)
      {
        rollAction = 1;
      }
      else if(rollVal > 0.9)
      {
        rollAction = 2;
      }
      else
      {
        rollAction = 0;
      }
  }

}
