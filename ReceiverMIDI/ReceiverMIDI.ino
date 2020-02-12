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

 // Radio declarations
 #include <SPI.h>
 #include <nRF24L01.h>
 #include <RF24.h>
 RF24 radio(7,8);  // CE,CSN
 const byte addressWand[6] = "00001";
 const byte addressBall[6] = "00010";

/*
 * ID == 1 : Wand
 *  stateOne is pitch action
 *  stateTwo is roll action
 *  
 * ID == 2 : Ball
 *  stateOne is
 *  stateTwo is
 */
 typedef struct{
  int ID;
  int stateOne;
  int stateTwo;
 } actionMessage;

 actionMessage messageWand;
 actionMessage messageBall;


int lastMainState = 0; // only states that are basic major/minor chords
int action = 1;
int pastAction = 0;
bool change = false;

int noteOne;
int noteTwo;
int noteThree;


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

// Actions determined by wand
int pitchAction = 0;
int rollAction = 0;

// Actions determined by ball


 void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, addressWand);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  //radio.stopListening();
  
 }
void loop() {
  if (radio.available()) {
    


    
    radio.read(&messageWand, sizeof(messageWand));

    Serial.print(messageWand.ID); Serial.print(" ");
    Serial.print(messageWand.stateOne); Serial.print(" ");
    Serial.print(messageWand.stateTwo); Serial.print(" ");
    Serial.println();

    // Set wand actions
    if(messageWand.ID == 1)
    {
      pitchAction = messageWand.stateOne;
      rollAction = messageWand.stateTwo;
    }
    //else if(message.ID == 2)
    //{
      // Ball actions go ehre!
    //}

  }

  setChordAction();
  if(change)
  {
    //turnChordOff();
    //setChord();
    //turnChordOn();
  }
  
  
}



void setChordAction()
{

  if(action != pastAction)
  {
    change = true;
    
    pastAction = action;
    if(action == 1 || action == 8 || action == 15 || action == 22 || action == 29 || action == 36)
    {
      lastMainState = action;
    }
  }
  else
  {
    change = false;
  }

  // If down direction
  if(pitchAction == 2)
  {
    //if right twist
    if(rollAction == 2)
    {
       switch(lastMainState)
      {
        case (1):
          action = 7;
          break;
        case (8):
          action = 14;
          break;
        case (15):
          action = 21;
          break;
        case (22):
          action = 28;
          break;
        case (29):
          action = 35;
          break;
        case (36):
          action = 42;
          break;
        default:
          break;
      }
    }
    // left twist
    else if(rollAction == 1)
    {
      switch (lastMainState)
      {
        case (1):
          action = 5;
          break;
        case (8):
          action = 12;
          break;
        case (15):
          action = 19;
          break;
        case (22):
          action = 26;
          break;
        case (29):
          action = 33;
          break;
        case (36):
          action = 40;
          break;
        default:
          break;
      }
    }
    // no twist
    else if (rollAction == 0)
    {
      switch (lastMainState)
      {
        case (1):
          action = 6;
          break;
        case (8):
          action = 13;
          break;
        case (15):
          action = 20;
          break;
        case (22):
          action = 27;
          break;
        case (29):
          action = 34;
          break;
        case (36):
          action = 42;
          break;
        default:
          break;
      } 
    }  
  }
  // Mid direction
  else if(pitchAction == 1)
  {
    //if right twist
    if(rollAction == 2)
    {
      switch(lastMainState)
      {
        case (1):
          action = 4;
          break;
        case (8):
          action = 11;
          break;
        case (15):
          action = 18;
          break;
        case (22):
          action = 25;
          break;
        case (29):
          action = 32;
          break;
        case (36):
          action = 39;
          break;
        default:
          break;
      }
    }
    //if no twist
    else if(rollAction == 0)
    {
      switch(lastMainState)
      {
        case (1):
          action = 3;
          break;
        case (8):
          action = 10;
          break;
        case (15):
          action = 17;
          break;
        case (22):
          action = 24;
          break;
        case (29):
          action = 31;
          break;
        case (36):
          action = 38;
          break;
        default:
          break;
      }
    } 
    //if left twist
    if(rollAction == 1)
    {
      switch(lastMainState)
      {
        case (1):
          action = 2;
          break;
        case (8):
          action = 9;
          break;
        case (15):
          action = 17;
          break;
        case (22):
          action = 24;
          break;
        case (29):
          action = 31;
          break;
        case (36):
          action = 38;
          break;
        default:
          break;
      }
    }  
  }
  else if (pitchAction == 0)
  {
    action = lastMainState;
  }
}


void setChord()
{
  switch(action)
  {
    case (1):
      noteOne = PONE[0];
      noteTwo = PONE[1];
      noteThree = PONE[2];
      break;
    case (2):
      noteOne = PONE_LL[0];
      noteTwo = PONE_LL[1];
      noteThree = PONE_LL[2];
      break;
    case (3):
      noteOne = PONE_LM[0];
      noteTwo = PONE_LM[1];
      noteThree = PONE_LM[2];
      break;
    case (4):
      noteOne = PONE_LR[0];
      noteTwo = PONE_LR[1];
      noteThree = PONE_LR[2];
      break;
    case (5):
      noteOne = PONE_RL[0];
      noteTwo = PONE_RL[1];
      noteThree = PONE_RL[2];
      break;
    case (6):
      noteOne = PONE_RM[0];
      noteTwo = PONE_RM[1];
      noteThree = PONE_RM[2];
      break;
    case (7):
      noteOne = PONE_RR[0];
      noteTwo = PONE_RR[1];
      noteThree = PONE_RR[2];
      break;
    case (8):
      noteOne = PTWO[0];
      noteTwo = PTWO[1];
      noteThree = PTWO[2];
      break;
    case (9):
      noteOne = PTWO_LL[0];
      noteTwo = PTWO_LL[1];
      noteThree = PTWO_LL[2];
      break;
    case (10):
      noteOne = PTWO_LM[0];
      noteTwo = PTWO_LM[1];
      noteThree = PTWO_LM[2];
      break;
    case (11):
      noteOne = PTWO_LR[0];
      noteTwo = PTWO_LR[1];
      noteThree = PTWO_LR[2];
      break;
    case (12):
      noteOne = PTWO_RL[0];
      noteTwo = PTWO_RL[1];
      noteThree = PTWO_RL[2];
      break;
    case (13):
      noteOne = PTWO_RM[0];
      noteTwo = PTWO_RM[1];
      noteThree = PTWO_RM[2];
      break;
    case (14):
      noteOne = PTWO_RR[0];
      noteTwo = PTWO_RR[1];
      noteThree = PTWO_RR[2];
      break;
    case (15):
      noteOne = PTHREE[0];
      noteTwo = PTHREE[1];
      noteThree = PTHREE[2];
      break;
    case (16):
      noteOne = PTHREE_LL[0];
      noteTwo = PTHREE_LL[1];
      noteThree = PTHREE_LL[2];
      break;
    case (17):
      noteOne = PTHREE_LM[0];
      noteTwo = PTHREE_LM[1];
      noteThree = PTHREE_LM[2];
      break;
    case (18):
      noteOne = PTHREE_LR[0];
      noteTwo = PTHREE_LR[1];
      noteThree = PTHREE_LR[2];
      break;
    case (19):
      noteOne = PTHREE_RL[0];
      noteTwo = PTHREE_RL[1];
      noteThree = PTHREE_RL[2];
      break;
    case (20):
      noteOne = PTHREE_RM[0];
      noteTwo = PTHREE_RM[1];
      noteThree = PTHREE_RM[2];
      break;
    case (21):
      noteOne = PTHREE_RR[0];
      noteTwo = PTHREE_RR[1];
      noteThree = PTHREE_RR[2];
      break;
    case (22):
      noteOne = PFOUR[0];
      noteTwo = PFOUR[1];
      noteThree = PFOUR[2];
      break;
    case (23):
      noteOne = PFOUR_LL[0];
      noteTwo = PFOUR_LL[1];
      noteThree = PFOUR_LL[2];
      break;
    case (24):
      noteOne = PFOUR_LM[0];
      noteTwo = PFOUR_LM[1];
      noteThree = PFOUR_LM[2];
      break;
    case (25):
      noteOne = PFOUR_LR[0];
      noteTwo = PFOUR_LR[1];
      noteThree = PFOUR_LR[2];
      break;
    case (26):
      noteOne = PFOUR_RL[0];
      noteTwo = PFOUR_RL[1];
      noteThree = PFOUR_RL[2];
      break;
    case (27):
      noteOne = PFOUR_RM[0];
      noteTwo = PFOUR_RM[1];
      noteThree = PFOUR_RM[2];
      break;
    case (28):
      noteOne = PFOUR_RR[0];
      noteTwo = PFOUR_RR[1];
      noteThree = PFOUR_RR[2];
      break;
    case (29):
      noteOne = PFOUR[0];
      noteTwo = PFOUR[1];
      noteThree = PFOUR[2];
      break;
    case (30):
      noteOne = PFOUR_LL[0];
      noteTwo = PFOUR_LL[1];
      noteThree = PFOUR_LL[2];
      break;
    case (31):
      noteOne = PFOUR_LM[0];
      noteTwo = PFOUR_LM[1];
      noteThree = PFOUR_LM[2];
      break;
    case (32):
      noteOne = PFOUR_LR[0];
      noteTwo = PFOUR_LR[1];
      noteThree = PFOUR_LR[2];
      break;
    case (33):
      noteOne = PFOUR_RL[0];
      noteTwo = PFOUR_RL[1];
      noteThree = PFOUR_RL[2];
      break;
    case (34):
      noteOne = PFOUR_RM[0];
      noteTwo = PFOUR_RM[1];
      noteThree = PFOUR_RM[2];
      break;
    case (35):
      noteOne = PFOUR_RR[0];
      noteTwo = PFOUR_RR[1];
      noteThree = PFOUR_RR[2];
      break;
    case (36):
      noteOne = PSIX[0];
      noteTwo = PSIX[1];
      noteThree = PSIX[2];
      break;
    case (37):
      noteOne = PSIX_LL[0];
      noteTwo = PSIX_LL[1];
      noteThree = PSIX_LL[2];
      break;
    case (38):
      noteOne = PSIX_LM[0];
      noteTwo = PSIX_LM[1];
      noteThree = PSIX_LM[2];
      break;
    case (39):
      noteOne = PSIX_LR[0];
      noteTwo = PSIX_LR[1];
      noteThree = PSIX_LR[2];
      break;
    case (40):
      noteOne = PSIX_RL[0];
      noteTwo = PSIX_RL[1];
      noteThree = PSIX_RL[2];
      break;
    case (41):
      noteOne = PSIX_RM[0];
      noteTwo = PSIX_RM[1];
      noteThree = PSIX_RM[2];
      break;
    case (42):
      noteOne = PSIX_RR[0];
      noteTwo = PSIX_RR[1];
      noteThree = PSIX_RR[2];
      break;
  }
}

// Starts chord
void turnChordOn()
{
  MIDImessage(noteON, 60+noteOne-36, velocity);
  MIDImessage(noteON, 60+noteOne, velocity);
  MIDImessage(noteON, 60+noteTwo, velocity);
  MIDImessage(noteON, 60+noteThree, velocity);  
}

// Turns chord off
void turnChordOff()
{
  MIDImessage(noteON, 60+noteOne-36, 0);
  MIDImessage(noteON, 60+noteOne, 0);
  MIDImessage(noteON, 60+noteTwo, 0);
  MIDImessage(noteON, 60+noteThree, 0);
}

//send MIDI message
void MIDImessage(int command, int MIDInote, int MIDIvelocity) {
  Serial.write(command);//send note on or note off command 
  Serial.write(MIDInote);//send pitch data
  Serial.write(MIDIvelocity);//send velocity data
}
