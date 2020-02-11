/*
 * CPSC 599 - Physical and Tangible HCI
 * Sarah Walker and Colin Au Yeung
 * 
 * tonnetz wand
 * 
 * This unit is used to control musical extension beyond a basic chord
 * 
 * See README.txt for states and mapping with tonnetz
 */

// Radio stuff
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7,8);  // CE,CSN

#include <SoftwareSerial.h>
#include <Nunchuk.h>
#include <Wire.h>
#include "nunchuk.h"

// Radio message components
const byte address[6] = "00001";
char message[3];


// Read values from sensor
double pitchVal;
double rollVal;

// Actions determined by value thresholds
int pitchAction = 0;
int rollAction = 0;

int action = 0;


void setup() {
    Serial.begin(9600);
    Wire.begin();
    nunchuk_init();
    
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}

void loop() {
 /* 
  if (nunchuk_read()) {
      // Work with nunchuk_data
      pitchVal = nunchuk_pitch();
      rollVal = nunchuk_roll();
      
      determineAction();
      delay(100);
      playChord();
  }
  
  else
  {
  }*/

  pitchAction = 1;
  rollAction = 2;
  
  itoa(action, message, 10);
  radio.write(&message, sizeof(message));
}


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

// Trigger action based on change in roll and pitch
void determineAction()
{  
  determinePitchAction();
  determineRollAction();  
}


void setID()
{
  // Wand identifier
  action = 100;
  action += 10 * pitchAction;
  action += rollAction;
}
