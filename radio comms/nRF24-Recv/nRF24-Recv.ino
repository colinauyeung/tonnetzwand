/*
 * CPSC 599 - Phhysical and Tangible HCI
 * Sarah Walker and Colin Au Yeung
 * 
 * Tonnetz Wand Master Reciever
 * 
 * This unit recieves action id's from wand and ball
 * and converts it to a midi message
 * 
 * See README.txt for more information
 */

 // Radio stuff
 #include <SPI.h>
 #include <nRF24L01.h>
 #include <RF24.h>
 RF24 radio(7,8);  // CE,CSN
 const byte address[6] = "00001";





// receive data as double
double val = 0;



// MIDI message requirements
int velocity = 100;//velocity of MIDI notes, must be between 0 and 127
int noteON = 144; //144 = 10010000 in binary, note on command

// Different chords

// Position 1 Inversions (C major is PONE)
// acion id = 1 - 7
int PONE[3] = {0,7,3};      //id = 1
int PONE_LL[3] = {0,7,11};  // id = 2
int PONE_LM[3] = {0,7,14};  // id = 3
int PONE_LR[3] = {0,7,10};  // id = 4
int PONE_RL[3] = {0,3,10};  // id = 5
int PONE_RM[3] = {0,3,6};   // id = 6
int PONE_RR[3] = {0,3,-1};  // id = 7

// Position Two Inversions
// action id -> 8-14
int PTWO[3] = {0,3,-4};     // id = 8
int PTWO_LL[3] = {0,3,10};  // id = 9
int PTWO_LM[3] = {0,3,6};   //id = 10
int PTWO_LR[3] = {0,3,-1};  // id = 11
int PTWO_RL[3] = {0,-4,-1}; // id = 12
int PTWO_RM[3] = {0,-4,-8}; // id = 13
int PTWO_RR[3] = {0,-4,-11};// id = 14

// Position Three Inversions
// action id -> 15-21
int PTHREE[3] = {0,-4,-7};    // id = 15
int PTHREE_LL[3] = {0,-4,-1}; // id = 16
int PTHREE_LM[3] = {0,-4,-8}; // id = 17
int PTHREE_LR[3] = {0,-4,-11};// id = 18
int PTHREE_RL[3] = {0,-7,-11};// id = 19
int PTHREE_RM[3] = {0,-7,-14};// id = 20
int PTHREE_RR[3] = {0,-7,-10};// id = 21

// Position Four Inversions
// action id -> 23-28
int PFOUR[3] = {0,-7,-3};     // id = 22
int PFOUR_LL[3] = {0,-7,-11}; // id = 23
int PFOUR_LM[3] = {0,-7,-14}; // id = 24
int PFOUR_LR[3] = {0,-7,-10}; // id = 25
int PFOUR_RL[3] = {0,-3,-10}; // id = 26
int PFOUR_RM[3] = {0,-3,-6};  // id = 27
int PFOUR_RR[3] = {0,-3,1};   // id = 28

// Position Five Inversions
// action id -> 20-35
int PFIVE[3] = {0,-3,4};      // id = 29
int PFIVE_LL[3] = {0,-3,-10}; // id = 30
int PFIVE_LM[3] = {0,-3,-6};  // id = 31
int PFIVE_LR[3] = {0,-3,1};   // id = 32
int PFIVE_RL[3] = {0,4,1};    // id = 33
int PFIVE_RM[3] = {0,4,8};    // id = 34
int PFIVE_RR[3] = {0,4,11};   // id = 35

// Position Six Inversions
// action id -> 36-43
int PSIX[3] = {0,4,7};      // id = 36
int PSIX_LL[3] = {0,4,1};   // id = 37
int PSIX_LM[3] = {0,4,8};   // id = 38
int PSIX_LR[3] = {0,4,11};  // id = 39
int PSIX_RL[3] = {0,7,11};  // id = 40
int PSIX_RM[3] = {0,7,14};  // id = 41
int PSIX_RR[3] = {0,7,10};  // id = 42











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

    val = atof(text);

  }
}


//send MIDI message
void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
  Serial.write(command);//send note on or note off command 
  Serial.write(MIDInote);//send pitch data
  Serial.write(MIDIvelocity);//send velocity data
}
