//////////////////////////////////////////
//  PRJ-0010-InputOutputTests-20220216  //
//////////////////////////////////////////
//
// This sketch contains project specific input output tests
//


// Switch 1 Variables
int state_s1 = 0;

int state_s1_last = 0;

const int AtRetractPin     =  4;     // ESP32 input pin for digital input Retract Stop microswitch (NC)
const int AtExtendPin      = 16;     // ESP32 input pin for digital input Extend  Stop microswitch (NC)
const int CurrPositionPin  = 34;     // ESP32 input pin for analog input Position Potentiometer

// Variables related to the retract move
int AtRetract              = 0;      // declare and set initial value
int LastAtRetract          = 0;

// Variables related to the extend move
int AtExtend               = 0;          // declare and set initial value
int LastAtExtend           = 0;

// Variables related to the current position potentiometer input
int RawPosition            = 0;      // declare and set initial position as potentiometer ohms value
int CurrentPosition        = 50;     // declare and set initial raw position ohms value mapped to 0-100 range
int LastPosition           = 50;     // 

// Variables related to the capacitive touch buttons
bool RetractTouchState          = false;  // Retract capacitive button touch made true/false
bool StopNowTouchState          = false;  // Stop now capacitive button touch made true/false
bool ExtendTouchState           = false;  // Extend capacitive button touch made true/false
bool LastRetractTouchState      = false;  // Last Retract capacitive button touch made true/false
bool LastStopNowTouchState      = false;  // Last Stop now capacitive button touch made true/false
bool LastExtendTouchState       = false;  // Last Extend capacitive button touch made true/false
const int ExtendMaxPosCapValue  = 45;     // Extend  touch(7) < this impirical test value for TRUE button pushed
const int StopNowMaxPosCapValue = 45;     // StopNow touch(6) < this impirical test value for TRUE button pushed 
//const int RetractMaxPosCapValue = 25;     // Retract touch(5) < this impirical test value for TRUE button pushed
const int RetractMaxPosCapValue = 0;        // why this change?
int ReadRetractTouchValue       = 0;      // Last Retract capacitive button touch made true/false
int ReadStopNowTouchValue       = 0;      // Last Stop now capacitive button touch made true/false
int ReadExtendTouchValue        = 0;      // Last Extend capacitive button touch made true/false
int LastRetractTouchValue       = 0;      // Last Retract capacitive button touch made true/false
int LastStopNowTouchValue       = 0;      // Last Stop now capacitive button touch made true/false
int LastExtendTouchValue        = 0;      // Last Extend capacitive button touch made true/false


// Variables related to the serial command input
String SerialCommandInput  = "0";    // Valid choices are [0, 1, 2, 3, 4, 5, 6]
int    TestChoice          = 0;
long CommandPosition       = 20;     // valid values are in the ramge from 0 to 100
int CommandDistance        = 30;     // valid values are 1, 2, or three (0 to 100)
bool CommandInputError     = true;   // CommandInputError test status

// Other Variables
bool DebugOn               = true;   // if true, print helpful debug status information
bool DeepDebugOn           = true;   // if true, print second-level debug status information
bool TestDone              = false;  // use in each test case to stay or leave
unsigned long myTime;                // used for test timeout
unsigned long TimeOut;               // used for test timeout
int DebounceCheck;                   // used for test switch debounce
/////////////
//  setup  //
/////////////

void setup(){
  // define pinModes
  pinMode(AtRetractPin,INPUT_PULLUP);       // Retract Stop from NC Microswitch
  pinMode(AtExtendPin,INPUT_PULLUP);        // Extend  Stop from NC Microswitch
  pinMode(CurrPositionPin, INPUT);          // Current Position from Potentiometer (optional definition)

  // Serial monitor setup
  Serial.begin(115200);

  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Welcome - Launching Input Output Test");
  state_s1_last = -1;
  state_s1 = 0;
 }//end of setup

////////////
//  loop  //
////////////
 
void loop() {
  SM_s1();                        // State Machine SM_s1
} //end of loop

//////////////////////
//  Function SM_s1  //
//////////////////////
void SM_s1() {
  // state machine for test choice selections
  switch (state_s1) {
    case 0:  // RESET FOR NEXT TEST
      SM_case_0();                      // function call
      break;
    case 1:  // TEST 1 = 'AtRetract' microswitch input
      SM_case_1();                      // function call
      break;
    case 2:  // TEST 2 = 'AtExtend' microswitch input
      SM_case_2();                      // function call
      break;
    case 3:  // TEST 3 = potentiometer slide values
      SM_case_3();                      // function call
      break;
    case 4:  // TEST 4 = EXTEND capacitive button touch
      SM_case_4();                      // function call
      break;
    case 5:  // TEST 5 = STOP capacitive button touch
      SM_case_5();                      // function call
      break;
    case 6:  // TEST 6 = RETRACT capacitive button touch
      SM_case_6();                      // function call
      break;
  }// END switch
}  //end of function SM_s1



//////////////////////////
//  Function SM_case_0  //
//////////////////////////
void SM_case_0(){                   // RESET FOR NEXT TEST
  if(state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_0 - Reset for next test");
    Serial.println("            Instructions - Use serial monitor data input to select a test");
    Serial.println("            '1' to test 'AtRetract' microswitch input");
    Serial.println("            '2' to test 'AtExtend' microswitch input");
    Serial.println("            '3' to test potentiometer slide values");
    Serial.println("            '4' to test EXTEND capacitive button touch");
    Serial.println("            '5' to test STOP capacitive button touch");
    Serial.println("            '6' to test RETRACT capacitive button touch");
    Serial.println("            What is your test choice?");
    Serial.println(" ");
  }
  // case actions
  TestDone = false;
  // wait for operator to enter the serial data test choice
  if(Serial.available()) {
    SerialCommandInput = Serial.readStringUntil('\n');
    if(DebugOn == true) Serial.println("            Received Serial Command");
    TestChoice = SerialCommandInput.toInt();
    // test for valid input
    if(TestChoice >= 0 && TestChoice <= 6) {
      CommandInputError = false;
      if(DebugOn == true) Serial.println("            Serial Command input is valid, in-range");
      state_s1 = TestChoice;
      Serial.print ("            Test Choice = ");
      Serial.println(TestChoice);
      TestDone = true;
      }
  }
  // test to stay or transition out
  if(TestDone == false) {          // stay in test
    state_s1_last = 0;
    state_s1 = 0;
  } else if(TestDone == true) {   // transition out3
    state_s1_last = 0;
    state_s1 = TestChoice;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_0


//////////////////////////
//  Function SM_case_1  //
//////////////////////////
void SM_case_1(){                   // TEST 1 = 'AtRetract' microswitch input
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_1 - Test 'AtRetract' microswitch input");
    Serial.println("            Mechanically operate RETRACT switch to see state changes");
    Serial.println("            Test ends in 10 seconds if no or after last state change");
    Serial.print  ("            AtRetract = ");
    Serial.println(AtRetract);
    myTime = millis();
   }
  // case actions
  TestDone = false;
  LastAtRetract = AtRetract;
  AtRetract = digitalRead(AtRetractPin);
  if (AtRetract != LastAtRetract) {
    // state change indicated, do debounce to eliminate noise and confirm change
    delay(50);
    DebounceCheck = digitalRead(AtRetractPin);
    if(DebounceCheck == AtRetract) {
      Serial.print  ("            State Change Detected. AtRetract = ");
      Serial.println(AtRetract);
      myTime = millis();
    }
  }
  // Look for test timeout
  TimeOut = millis() - myTime;
  if(TimeOut >= 10000) {
    TestDone = true;
    Serial.println("            Test time out");
  }else{
    TestDone = false;
  }
  
  if(TestDone == false) {          // stay in test
    state_s1_last = 1;
    state_s1 = 1;
  } else if(TestDone == true) {   // transition out
    state_s1_last = 1;
    state_s1 = 0;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_1



//////////////////////////
//  Function SM_case_2  //
//////////////////////////
void SM_case_2(){                   // TEST 2 = 'AtExtend' microswitch input
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_2 - Test 'AtExtend' microswitch input");
    Serial.println("            Mechanically operate EXTEND switch to see state changes");
    Serial.println("            Test ends in 10 seconds if no or after last state change");
    Serial.print  ("            AtExtend = ");
    Serial.println(AtExtend);
    myTime = millis();
   }
  // case actions
  TestDone = false;
  LastAtExtend = AtExtend;
  AtExtend = digitalRead(AtExtendPin);
  if (LastAtExtend != AtExtend) {
    // state change indicated, do debounce to eliminate noise and confirm change
    delay(50);
    DebounceCheck = digitalRead(AtExtendPin);
    if(DebounceCheck == AtExtend) {
      Serial.print  ("            State Change Detected. AtExtend = ");
      Serial.println(AtExtend);
      myTime = millis();
    }
  }
  // Look for test timeout
  TimeOut = millis() - myTime;
  if(TimeOut >= 10000) {
    TestDone = true;
    Serial.println("            Test time out");
  }else{
    TestDone = false;
  }
  if(TestDone == false) {          // stay in test
    state_s1_last = 2;
    state_s1 = 2;
  } else if(TestDone == true) {   // transition out
    state_s1_last = 2;
    state_s1 = 0;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_2



//////////////////////////
//  Function SM_case_3  //
//////////////////////////
void SM_case_3(){                   // TEST 3 = potentiometer slide values
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_3 - potentiometer slide values");
    Serial.println("            Mechanically operate Potentiometer to see value changes");
    Serial.println("            Test ends in 10 seconds if no or after last state change");
    myTime = millis();
  }
  
  // case actions
  TestDone = false;
  LastPosition = CurrentPosition;
  GetCurrentPosition();
  if(CurrentPosition != LastPosition) {
    Serial.print("            Current Position = ");
    Serial.println(CurrentPosition);
    myTime = millis();
    LastPosition = CurrentPosition;
  }
  delay(1000);

  // Look for test timeout
  TimeOut = millis() - myTime;
  if(TimeOut >= 10000) {
    TestDone = true;
    Serial.println("            Test time out");
  }else{
    TestDone = false;
  }

  if(TestDone == false) {          // stay in test
    state_s1_last = 3;
    state_s1 = 3;
  } else if(TestDone == true) {   // transition out
    state_s1_last = 3;
    state_s1 = 0;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_3


//////////////////////////
//  Function SM_case_4  //
//////////////////////////
void SM_case_4(){                   // TEST 4 = EXTEND capacitive button touch
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_4 - EXTEND capacitive button touch");
    Serial.println("            Touch the EXTEND button to see state changes");
    Serial.println("            Test ends in 10 seconds if no or after last state change");
    Serial.print  ("            ExtendMaxPosCapValue = ");
    Serial.println(ExtendMaxPosCapValue);
    LastRetractTouchValue = touchRead(T7);
    myTime = millis();
  }  
   
  // case actions
  TestDone = false;
  ExtendTouchState = false;
  ReadExtendTouchValue = touchRead(T7);                            // read EXTEND touch button
  LastExtendTouchValue = ReadExtendTouchValue;
  if(ReadExtendTouchValue < ExtendMaxPosCapValue) {                // touch value has changed is it a command touch or or not?
    // Touch indicated, do a debounce second read to confirm
    delay(50);
    if(touchRead(T7) < ExtendMaxPosCapValue) {
      ExtendTouchState = true;                                     // EXTEND button value change is a command
      Serial.println("            EXTEND touch made");
      LastExtendTouchValue = ReadExtendTouchValue;
      myTime = millis();
    }else{
      ExtendTouchState = false;                                    // EXTEND button value change is noise
    }
  }
  Serial.print("            LastExtendTouchValue = ");
  Serial.print(LastExtendTouchValue);
  Serial.print(", ReadExtendTouchValue = ");
  Serial.print(ReadExtendTouchValue);
  Serial.print(" , ExtendTouchState = ");
  Serial.println(ExtendTouchState);
  delay(1000);
  // Look for test timeout
  TimeOut = millis() - myTime;
  if(TimeOut >= 10000) {
    TestDone = true;
    Serial.println("            Test time out");
  }else{
    TestDone = false;
  }
  
  if(TestDone == false) {          // stay in test
    state_s1_last = 4;
    state_s1 = 4;
  } else if(TestDone == true) {   // transition out
    state_s1_last = 4;
    state_s1 = 0;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_4


//////////////////////////
//  Function SM_case_5  //
//////////////////////////
void SM_case_5(){                   // TEST 5 = STOP capacitive button touch
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_5 - EXTEND capacitive button touch");
    Serial.println("            Touch the STOP button to see state changes");
    Serial.println("            Test ends in 10 seconds if no or after last state change");
    Serial.print  ("            StopNowMaxPosCapValue = ");
    Serial.println(StopNowMaxPosCapValue);
    LastStopNowTouchValue = touchRead(T6);
    myTime = millis();
  }
  
  // case actions
  TestDone = false;
  StopNowTouchState = false;
  ReadStopNowTouchValue = touchRead(T6);                               // read STOP touch button
  LastStopNowTouchValue = ReadStopNowTouchValue;
  if(ReadStopNowTouchValue < StopNowMaxPosCapValue) {                  // touch value has changed is it a command touch or or not?
    // Touch indicated, do a debounce second read to confirm
    delay(50);
    if(touchRead(T6) < StopNowMaxPosCapValue) {
      StopNowTouchState = true;                                        // RETRACT button value change is a command
      Serial.println("            STOP touch made");
      LastStopNowTouchValue = ReadStopNowTouchValue;
      myTime = millis();
    }else{
      StopNowTouchState = false;                                       // STOP button value change is noise
    }
  }
  Serial.print("            LastStopNowTouchValue = ");
  Serial.print(LastStopNowTouchValue);
  Serial.print(", ReadStopNowTouchValue = ");
  Serial.print(ReadStopNowTouchValue);
  Serial.print(" , StopNowTouchState = ");
  Serial.println(StopNowTouchState);
  delay(1000);
  // Look for test timeout
  TimeOut = millis() - myTime;
  if(TimeOut >= 10000) {
    TestDone = true;
    Serial.println("            Test time out");
  }else{
    TestDone = false;
  }

  if(TestDone == false) {          // stay in test
    state_s1_last = 5;
    state_s1 = 5;
  } else if(TestDone == true) {   // transition out
    state_s1_last = 5;
    state_s1 = 0;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_5


//////////////////////////
//  Function SM_case_6  //
//////////////////////////
void SM_case_6(){                   // TEST 6 = RETRACT capacitive button touch
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("============");
    Serial.println("SM_case_6 - RETRACT capacitive button touch");
    Serial.println("            Touch the RETRACT button to see state changes");
    Serial.println("            Test ends in 10 seconds if no or after last state change");
    Serial.print  ("            RetractMaxPosCapValue = ");
    Serial.println(RetractMaxPosCapValue);
    LastRetractTouchValue = touchRead(T5);
    myTime = millis();
  }
   
  // case actions
  TestDone = false;
  RetractTouchState = false;
  ReadRetractTouchValue = touchRead(T5);                                 // read RETRACT touch button

  // Serial.print("************** touchRead(5) = ");                     // Touch(5) is wacky different
  // Serial.print(ReadRetractTouchValue);
  // Serial.println (" **************");
  
  LastRetractTouchValue = ReadRetractTouchValue;
//  if(ReadRetractTouchValue < RetractMaxPosCapValue) {                  // touch value has changed is it a command touch or or not?
    if(ReadRetractTouchValue > RetractMaxPosCapValue) {                  // why this change?
    // Touch indicated, do a debounce second read to confirm
//    delay(50);
//    if(touchRead(T5) < RetractMaxPosCapValue) {
      if(touchRead(T5) > RetractMaxPosCapValue) {                          // why this change?
      RetractTouchState = true;  // RETRACT button value change is a command
      Serial.println("            RETRACT touch made");
      LastRetractTouchValue = ReadRetractTouchValue;
      myTime = millis();
    }else{
      RetractTouchState = false;                                         // RETRACT button value change is noise
    }
  }
  Serial.print("            LastRetractTouchValue = ");
  Serial.print(LastRetractTouchValue);
  Serial.print(", ReadRetractTouchValue = ");
  Serial.print(ReadRetractTouchValue);
  Serial.print(" , RetractTouchState = ");
  Serial.println(RetractTouchState);
  delay(1000);
  // Look for test timeout
  TimeOut = millis() - myTime;
  if(TimeOut >= 10000) {
    TestDone = true;
    Serial.println("            Test time out");
  }else{
    TestDone = false;
  }
  
  
    
    if(TestDone == false) {          // stay in test
    state_s1_last = 6;
    state_s1 = 6;
  } else if(TestDone == true) {   // transition out
    state_s1_last = 6;
    state_s1 = 0;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }
}//end of function SM_case_6


///////////////////////////////////
//  Function GetCurrentPosition  //
///////////////////////////////////
void GetCurrentPosition(){
  RawPosition = analogRead(CurrPositionPin);
  CurrentPosition = map(RawPosition,100,4000,1,99);
  CurrentPosition = -1 * (CurrentPosition - 100);     // reverses direction
  if(CurrentPosition > 100) CurrentPosition = 100;
  if(CurrentPosition <   0) CurrentPosition =   0;
  if(DebugOn == false){
    Serial.print("call Function GetCurrentPosition => ");
    Serial.print("Current Position = ");
    Serial.println(CurrentPosition);
  }
}//end of function GetCurrentPosition
