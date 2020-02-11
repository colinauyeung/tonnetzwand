/*
 * * Arduino Wireless Communication Tutorial
 * * Example 1 - Transmitter Code
 * *
 * * by Dejan Nedelovski, www.howtomechatronics.com
 * * Library TMRh20/RF24, github.com/tmrh/RF24
 * */

 // Radio stuff
 #include <SPI.h>
 #include <nRF24L01.h>
 #include <RF24.h>
 RF24 radio(7,8);  // CE,CSN
 const byte address[6] = "00001";



// Music Stuff
int major[] = {0,4,7,0};
int dominantSeventh[] = {0,4,7,10};
int majorSixth[] = {0,-3,4,7};
int majorSeventh[] = {0,4,7,11};
int velocity = 100;//velocity of MIDI notes, must be between 0 and 127
//(higher velocity usually makes MIDI instruments louder)
int noteON = 144;//144 = 10010000 in binary, note on command

int one;
int two;
int three;
int root = 60;

// receive data as double
double val = 0;

 void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
 }
void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    //Serial.println(text);

    val = atof(text);

    Serial.println(val);
    if(val == -1)
    {
      root--;
    }
  }

    one = root+major[1];
    two = root+major[2];
    three = root+major[3];
    //playChord(root, one, two, three);
}

void playChord(int root, int one, int two, int three)
{
  MIDImessage(noteON, root, velocity);
  delay(100);
  MIDImessage(noteON, root, 0);

  MIDImessage(noteON, one, velocity);
  delay(100);
  MIDImessage(noteON, one, 0);

  MIDImessage(noteON, two, velocity);
  delay(100);
  MIDImessage(noteON, two, 0);

  if(three != 0)
  {
    MIDImessage(noteON, three, velocity);
    delay(100);
    MIDImessage(noteON, three, 0);
  }
}

//send MIDI message
void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
  Serial.write(command);//send note on or note off command 
  Serial.write(MIDInote);//send pitch data
  Serial.write(MIDIvelocity);//send velocity data
}
