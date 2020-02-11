/*
 * * Arduino Wireless Communication Tutorial
 * * Example 1 - Transmitter Code
 * *
 * * by Dejan Nedelovski, www.howtomechatronics.com
 * * Library TMRh20/RF24, github.com/tmrh/RF24
 * */


#include <SoftwareSerial.h>
#include <Nunchuk.h>
#include <Wire.h>
#include "nunchuk.h"


 
 #include <SPI.h>
 #include <nRF24L01.h>
 #include <RF24.h>
 RF24 radio(7,8);  // CE,CSN
 const byte address[6] = "00001";
 char idstr[5];
 char message[10];
 char info[10];
 int Year =2020;

// 
 double x;
 int id = 0;
 
 /*
  * 1 - shift note upward by 1
  * 2 - shit note downward by 1
  */
 int action = 0;

//History
 double history[4] = {0,0,0,0};

 int count = 0;




 // Timer
 unsigned long timerStart;
 unsigned long timerPeriod = 1000;
  
 void setup(){
 Serial.begin(9600);
 
 radio.begin();
 radio.openWritingPipe(address);
 radio.setPALevel(RF24_PA_MIN);
 radio.stopListening();
 
 Wire.begin();
 nunchuk_init(); 

 timerStart = millis();
 }

 
 void loop() {
  
  double x;
  if(nunchuk_read()) {
    x = nunchuk_pitch();
    //Serial.println(x);

    count++;


   // set historical values
    history[0] = history[1];
    history[1] = history[2];
    history[2] = history[3];
    history[3] = x;
    //Serial.println(x);

    triggerAction(history[0],history[1],history[2],history[3]);
    
  }
  delay(200);
  //const char text[] = "Hello World";
  //radio.write(&text, sizeof(text));
  //delay(1000);

  //itoa(Year,cstr,10);
  //Serial.println(Year);
  //Serial.println(cstr);
  //radio.write (&cstr, sizeof(cstr));

  itoa(id,idstr,10);        // convert id to cstring
  itoa(action,message,10);  // convert action to cstring
  
  strcat(message,idstr);
  //action, id
  Serial.println(message);

  //Serial.println(action);
  //radio.write(&cstr, sizeof(cstr));
  
  //delay(500);
}



// Trigger action based on relation between current and three historial values
void triggerAction(double one, double two, double three, double current)
{
    bool changedAction = false;
    
    // tilt forward action for x
    if((two > 0 || three > 0) && (one < 0 || two < 0 || three < 0) && current < 0)
    {
      if(action != 1)
      {
        action = 1;
        id++;
        changedAction = true;
      }
    }

    // tilt backward action for x
    else if((two < -2 || three < -2) && (one > -2 || two > -2 || three > -2) && current > -2)
    {
      if(action != 2)
      {
        action = 2;
        id++;
        changedAction = true;
      }
    }

    if(!changeAction && (millis() - timerStart >= timerPeriod))
    {
      id++;
    }
    else if(changedAction)
    {
      timerStart = millis();
    }
    
    if(id > 9)
    {
      id = 1;
    }
}
 
