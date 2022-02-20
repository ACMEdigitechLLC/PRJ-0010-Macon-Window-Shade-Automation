////////////////////////////////////////////////
//  PRJ-0010-MaconWindowShade-Rev10-20220217  //
////////////////////////////////////////////////
//
// This is built on near-fully working Rev.9 to add raw code for the working web interface
//
// Credits:
// Based upon example from Rui Santos
// Random Nerd Tutorials
// https://randomnerdtutorials.com/esp32-servo-motor-web-server-arduino-ide/
  
// https://dronebotworkshop.com/esp32cam-robot-car/
// https://dronebotworkshop.com/tb6612fng-h-bridge/

// Official Espressif ESP32 touch sensing docs:
// * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/touch_pad.html
// * https://github.com/espressif/esp-iot-solution/blob/master/documents/touch_pad_solution/touch_sensor_design_en.md
// ** https://www.arduino.cc/en/Tutorial/BuiltInExamples/Debounce
// *** https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
// **** https://dronebotworkshop.com/esp32-servo/

// Notes:
// (1) If variable "DebugOn+ == true, output debug info text to the serial monitor (not present in normal operation
// (2) Once running, can enter position command (00, 01, 02, ... 99) from the serial monitor (if connected this way) 
// (3)
// (4)


// State Machine "SM_s1"
//   state_s1 = ?
//   case 0:  //INITIALIZE_RESET
//            reset initial values
//            then transition to case 1
//   case 1:  //WAIT_FOR_COMMAND
//
//            transitions to case 2
//   case 2:  //PROCESS_COMMAND_INPUT
//            transitions to either case 3 or 6      
//   case 3:  //SETUP_RETRACT_MOVE
//            transitions to case 4
//   case 4:  //START_RETRACT_MOVE
//            transitions to case 5
//   case 5:  //MONITOR_TO_STOP_RETRACT
//            transitions to case 1
//   case 6:  //SETUP_EXTEND_MOVE
//            transitions to case 7
//   case 7:  //START_EXTEND_MOVE
//            transitions to case 8
//   case 8:  //MONITOR_TO_STOP_EXTEND
//            transitions to case 1

// EXP32 pin assignments (total 9)
// StartExtend button  is Touch7 which is logical assignment GIOP27 on physical pin 11
// StopNow button      is Touch6 which is logical assignment GIOP14 on physical pin 12
// StartRetract button is Touch5 which is logical assignment GIOP12 on physical pin 13

// **** web code
#include <WiFi.h>
//#include <ESP32Servo.h>
// **** web code

const int AtRetractPin     =  4;     // ESP32 input pin for digital input Retract Stop microswitch (NC)
const int AtExtendPin      = 16;     // ESP32 input pin for digital input Extend  Stop microswitch (NC)
const int CurrPositionPin  = 34;     // ESP32 input pin for analog input Position Potentiometer
const int MotorDriverAIN1pin = 15;    // ES{P2 output pin to the Motor Driver AIN1 pin 
const int MotorDriverAIN2pin = 2;   // ESP32 output pin to the Motor Driver AIN2 pin
const int enable1Pin = 33;           // ESP32 output pin to the Motor Driver PWMA pin

// *** Setting for PWM Motor Driver properties
const int freq = 30000;              // Motor Driver PWMA pin PWM frequence (based on *** source)
const int pwmChannel = 0;            // Motor Driver channel (based on *** source)
const int resolution = 8;            // Motor Driver resolution (based on *** source)
int dutyCycle = 200;                 // Motor Driver dutyCycle (based on *** source 0-255 range)

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
const int RetractMaxPosCapValue = 3;        // why this change?
int ReadRetractTouchValue       = 0;      // Last Retract capacitive button touch made true/false
int ReadStopNowTouchValue       = 0;      // Last Stop now capacitive button touch made true/false
int ReadExtendTouchValue        = 0;      // Last Extend capacitive button touch made true/false
int LastRetractTouchValue       = 0;      // Last Retract capacitive button touch made true/false
int LastStopNowTouchValue       = 0;      // Last Stop now capacitive button touch made true/false
int LastExtendTouchValue        = 0;      // Last Extend capacitive button touch made true/false

// Variables related to the retract move
int AtRetract       = 0;                     // declare and set initial value

// Variables related to the extend move
int AtExtend         = LOW;          // declare and set initial value

// Variables related to the current position potentiometer input
int RawPosition            = 0;      // declare and set initial position as potentiometer ohms value
int CurrentPosition        = 0;      // declare and set initial raw position ohms value mapped to 0-100 range

// Switch 1 Variables
int state_s1 = 0;
int state_s1_last = 0;
int pin_s1 = AtRetractPin;
int val_s1 = 0;
//unsigned long t_s1 = 0;
//unsigned long t_0_s1 = 0;
//unsigned long bounce_delay_s1 = 5;

// Variables related to the serial command input
String SerialCommandInput  = "0";    // 0, 1, 2, 3, ... 98, 99, 100
long CommandPosition       = 0;      // valid values are in the ramge from 0 to 100
int CommandDistance        = 0;      // valid values are 1, 2, or three (0 to 100)
bool CommandInputError     = true;   // CommandInputError test status

// Variables related to the web command input
bool NewWebInput           = true;   // 

// Other Variables
bool DebugOn = true;                 // if true, print helpful debug status information
bool DeepDebugOn = false;            // if true, print second-level debug status information
int CommandSource = 0;               // 1=RetractButton, 2=ExtendButton , 3=Serial, 4=web
bool StopNow;                        // used in case 5 and 8 to stop if commanded
unsigned long myTime;                // used for test timeout


// **** web code
// Servo GPIO pin
static const int servoPin = 18;

// Network credentials
const char* ssid     = "NETGEAR22";
const char* password = "silkycanoe780";

// Web server on port 80 (http)
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Decode HTTP GET value
String valueString = String(5);
String valueStringLast = String(5);
int pos1 = 0;
int pos2 = 0;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
// **** web code

/////////////
//  setup  //
/////////////

void setup(){
  // define pinModes
  pinMode(AtRetractPin,INPUT_PULLUP);       // Retract Stop from NC Microswitch
  pinMode(AtExtendPin,INPUT_PULLUP);        // Extend  Stop from NC Microswitch
  pinMode(CurrPositionPin, INPUT);          // Current Position from Potentiometer (optional definition)
  pinMode(MotorDriverAIN1pin,OUTPUT);       // Retract Stop from NC Microswitch
  pinMode(MotorDriverAIN2pin,OUTPUT);       // Extend  Stop from NC Microswitch
  pinMode(enable1Pin,OUTPUT);               // copy source code as-is from *** example for Motor Driver pin PWMA

  // *** configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);  // copy source code as-is from *** example
  
  // *** attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);    // copy source code as-is from *** example
  // Serial monitor setup
  Serial.begin(115200);

// **** web code
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
// **** web code

  // Start state machine SM_s1 at case 0
  state_s1 = 0;
  state_s1_last = -1;
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("Welcome - Launching app at state_s1 = 0 ");
  
 }//end of setup

////////////
//  loop  //
////////////
 
void loop() {
  //if(state_s1_last == -1) Serial.println("Welcome - Launching app at state_s1 = 0 ");
  SM_s1();                        // State Machine SM_s1
}//end of loop



//////////////////////
//  Function SM_s1  //
//////////////////////

void SM_s1() {

  switch (state_s1) {
    case 0:  //INITIALIZE_RESET
      SM_case_0();                      // function call
      break;
    case 1:  //WAIT_FOR_COMMAND
      SM_case_1();                      // function call
      break;
    case 2:  //PROCESS_COMMAND_INPUT
      SM_case_2();                      // function call
      break;
    case 3:  //SETUP_RETRACT_MOVE
      SM_case_3();                      // function call
      break;
    case 4:  //RETRACT_MOVE
      SM_case_4();                      // function call
      break;
    case 5:  //RETRACT_STOP
      SM_case_5();                      // function call
      break;
    case 6:  //SETUP_EXTEND_MOVE
      SM_case_6();                      // function call
      break;
    case 7:  //EXTEND_MOVE
      SM_case_7();                      // function call
      break;
    case 8:  //EXTEND_STOP
      SM_case_8();                      // function call
      break;
  }// END switch
}  //end of function SM_s1

//////////////////////////
//  Function SM_case_0  //
//////////////////////////
void SM_case_0(){  //INITIALIZE_RESET
  // case actions
  GetCurrentPosition();
  ExtendTouchState = false;
  StopNowTouchState = false;
  RetractTouchState = false;
  if(DebugOn == true){
    Serial.println("SM_case_0 - INITIALIZE_RESET");
    Serial.print  ("            Current position = "); 
    Serial.println(CurrentPosition);
    // read capactive touch push button inputs
    Serial.print("            Extend touchRead(7) = ");
    Serial.print(touchRead(T7));
    Serial.print(", StopNow touchRead(6) = ");
    Serial.print(touchRead(T6));
    Serial.print(", Retract touchRead(5) = ");
    Serial.println(touchRead(T5));
  }
  // transition out
  state_s1_last = 0;
  state_s1 = 1;
  Serial.println("            transition to state_S1 = 1");
  Serial.println("============ What Next? ============");

}//end of function SM_case_0

//////////////////////////
//  Function SM_case_1  //
//////////////////////////
void SM_case_1(){  //WAIT_FOR_COMMAND
  // case actions command can come from four sources
  // In factory desktop mode, command can come from serial monitor text input (00, 01, 02, ...99)
  // In manual operation mode, command can come from Extend/Retract/Stop capactive button inputs
  // In automatic operaton mode, command can come from web text input (00, 01, 02, ...99)
  
  // read capactive touch push button inputs
  // Serial.print("Retract input = ");
  // Serial.print(touchRead(T7));
  // Serial.print(", StopNow input = ");
  // Serial.print(touchRead(T6));
  // Serial.print(", Extend input = ");
  // Serial.println(touchRead(T5));

  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("SM_case_1 - Wait for command");
    state_s1_last = 1;
  }

  // ***** Test for RETRACT command input and transition out if valid *****
  //  
  RetractTouchState = false;
  ReadRetractTouchValue = touchRead(T5);                          // read RETRACT touch button

  // Serial.print("************** touchRead(5) = ");                     // Touch(5) is wacky different
  // Serial.print(ReadRetractTouchValue);
  // Serial.println (" **************");
  
  LastRetractTouchValue = ReadRetractTouchValue;
//  if(ReadRetractTouchValue < RetractMaxPosCapValue) {                  // touch value has changed is it a command touch or or not?
    if(ReadRetractTouchValue < RetractMaxPosCapValue) {                  // why this change?
    // Touch indicated, do a debounce second read to confirm
//    delay(50);
//    if(touchRead(T5) < RetractMaxPosCapValue) {
      if(touchRead(T5) >= RetractMaxPosCapValue) {                          // why this change?
      RetractTouchState = true;                                   // RETRACT button value change is a command
      Serial.println("            RETRACT touch made");
      LastRetractTouchValue = ReadRetractTouchValue;
      myTime = millis();
      // transition out
      state_s1_last = 1;
      state_s1 = 2;                                               // temp ***********************************
      Serial.print("            LastRetractTouchValue = ");
      Serial.print(LastRetractTouchValue);
      Serial.print(", ReadRetractTouchValue = ");
      Serial.print(ReadRetractTouchValue);
      Serial.print(" , RetractTouchState = ");
      Serial.println(RetractTouchState);
      Serial.println("            transition to state_S1 = 2");
    }else{
      RetractTouchState = false;                                  // RETRACT button value change is noise
    }
  }

  // ***** Test for EXTEND command input and transition out if valid *****
  //  
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
      Serial.print("            LastExtendTouchValue = ");
      Serial.print(LastExtendTouchValue);
      Serial.print(", ReadExtendTouchValue = ");
      Serial.print(ReadExtendTouchValue);
      Serial.print(" , ExtendTouchState = ");
      Serial.println(ExtendTouchState);
      // transition out
      state_s1_last = 1;
      state_s1 = 2;
      Serial.println("            transition to state_S1 = 2");   
    }else{
      ExtendTouchState = false;                                    // EXTEND button value change is noise
    }
  }

  // ***** Test for SERIAL MONITOR command input and transition out if valid *****
  //  
  if(Serial.available()){
    SerialCommandInput = Serial.readStringUntil('\n');
    CommandSource = 3;
  if(DebugOn == true) Serial.println("            Received Serial Command");
    state_s1_last = 1;
    state_s1 = 2;

    if(DebugOn == true) {
      Serial.print("            transition to state_S1 = ");
      Serial.println(state_s1);
    }
  }

  // ***** Test for INTERNET WEB command input and transition out if valid *****
  //  
  GetWebInput();
  if (NewWebInput == true) {
    if(DebugOn == true) {
      Serial.print("            Received Web Command, valueString = ");
      Serial.println(valueString);
    }
    NewWebInput = false;
    CommandSource = 4;
    state_s1_last = 1;
    state_s1 = 2;
  }
// ****
}//end of function SM_case_1

//////////////////////////
//  Function SM_case_2  //
//////////////////////////
void SM_case_2(){  //PROCESS_COMMAND_INPUT
  // case actions
  if(DebugOn == true) {
    Serial.println("SM_case_2 - Process Command Input");
    Serial.print  ("            CommandSource = ");
    Serial.print(CommandSource);
  }
  
  GetCurrentPosition();                             // valid values in range (0, 1, 2, ..., 100)
  if(CommandSource == 1) {
    if(DebugOn == true) Serial.println(", RETRACT button pressed");
    CommandInputError = false;
    CommandPosition = 0;
    CommandDistance = CommandPosition - CurrentPosition;
    CommandInputError = false;
    state_s1_last = 2;
    state_s1 = 3;                              // transition out to RETRACT_MOVE
  }else if(CommandSource == 2) {
    if(DebugOn == true) Serial.println(", EXTEND button pressed");
    CommandInputError = false;
    CommandPosition = 100;
    CommandDistance = CommandPosition - CurrentPosition;
    state_s1_last = 2;
    state_s1 = 6;                              // transition out to EXTEND_MOVE
  }else if(CommandSource == 3) { 
    if(DebugOn == true) Serial.println(", Received Serial Command");
    CommandPosition = SerialCommandInput.toInt();
    // test for valid input
    if(CommandPosition >= 0 && CommandPosition <= 100) {
      CommandInputError = false;
      if(DebugOn == true) Serial.println("            Serial Command input is valid, in-range");
    }else{
      CommandInputError = true;
      if(DebugOn == true) Serial.println("            Serial Command input is invalid, out-of-range");
    }
    CommandDistance = CommandPosition - CurrentPosition;  // ???????????????????????????????????
    // set Serial Command transition out destination
    if(CommandInputError == false && CommandDistance > 0){
      state_s1_last = 2;
      state_s1 = 6;  
      //if(DebugOn == true) Serial.println("            Make Serial Command EXTEND Move");
    }else if(CommandInputError == false && CommandDistance < 0){
      state_s1_last = 2;
      state_s1 = 3;
      //if(DebugOn == true) Serial.println("            Make Serial Command RETRACT Move");
    }else if(CommandInputError == false && CommandDistance == 0){
      state_s1_last = 2;
      state_s1 = 1;            // no change, 
      //if(DebugOn == true) Serial.println("            Ignore Zero-Distance Serial Command Move");
    }else{
      CommandInputError = true;
      //if(DebugOn == true) Serial.println("            Ignore Invalid Serial Commanded Move");
    }
  }else if(CommandSource == 4) {
    if(DebugOn == true) Serial.println(", Received Web Interface Command");
    CommandInputError = false;
    CommandPosition = valueString.toInt();
    CommandDistance = CommandPosition - CurrentPosition;
    if(CommandInputError == false && CommandDistance > 0){
      state_s1_last = 2;
      state_s1 = 6; 
      //if(DebugOn == true) Serial.println("            Make Web Command EXTEND Move");
    }else if(CommandInputError == false && CommandDistance < 0){
      state_s1_last = 2;
      state_s1 = 3;
      //if(DebugOn == true) Serial.println("            Make Web Command RETRACT Move");
    }else if(CommandInputError == false && CommandDistance == 0){
      state_s1_last = 2;
      state_s1 = 1;            // no change
      //if(DebugOn == true) Serial.println("            Ignore Zero-Distance Serial Command Move");
    }else{
      CommandInputError = true;
      //if(DebugOn == true) Serial.println("            Ignore Invalid Serial Commanded Move");
    }
  }else {
    if(DebugOn == true) Serial.println("Ignore INVALID Command");
    state_s1_last = 2;
    state_s1 = 1;
  }
    
  // transition out
  
  if(DebugOn == true) {
    Serial.print("            Current Position = ");
    Serial.print(CurrentPosition);
    Serial.print(", Command Position = ");
    Serial.print(CommandPosition);
    Serial.print(", Command Distance = ");
    Serial.println(CommandDistance);
  }else{
    Serial.print(", Command Input Error = ");
    Serial.println(SerialCommandInput);
  }
  if(DebugOn == true) {
    Serial.print  ("            transition to state_S1 = ");
    Serial.println(state_s1);
  }
}//end of function SM_case_2

//////////////////////////
//  Function SM_case_3  //
//////////////////////////
void SM_case_3(){  //SETUP_RETRACT_MOVE
  if(DebugOn == true) {
    //DisplayShade(CurrentPosition);
    Serial.println("SM_case_3 - Setup RETRACT move");
  }
  // case actions

  // transition out
  state_s1_last = 3;
  state_s1 = 4;
  if(DebugOn == true) {
    Serial.print  ("            transition to state_S1 = ");
    Serial.println(state_s1);
  }
}//end of function SM_case_3

//////////////////////////
//  Function SM_case_4  //
//////////////////////////
void SM_case_4(){  //START_RETRACT_MOVE
  if(DebugOn == true) {
    Serial.println("SM_case_4 - Start RETRACT Move");
    Serial.print  ("            Retracting now from ");
    Serial.print("CurrentPosition = ");
    Serial.print(CurrentPosition);
    Serial.print(" to CommandPosition = ");
    Serial.println(CommandPosition);
  }
  // case actions
  // First check to see AtRetract Microswitch is armed to trigger the stop
  AtRetract = digitalRead(AtRetractPin);
  //test
  if(AtRetract == LOW) {
    // AtRetract microswitch is armed to stop the move so turn on motor CW
    digitalWrite(MotorDriverAIN1pin, HIGH);
    digitalWrite(MotorDriverAIN2pin, LOW);
    ledcWrite(pwmChannel,dutyCycle);         // ***
    // display the move on the serial monitor
    if(DeepDebugOn == true) {
      for(int i = CurrentPosition; i >= CommandPosition; i--) {
      DisplayShade(i);
      delay(100);
      }
    }
    state_s1_last = 4;
    state_s1 = 5;
  } else {
    // Possible problem with AtRetract microswitch so don't make the move
    Serial.println("            Problem with AtRetract microswitch - don't start EXTEND move");
    // transition out
    state_s1_last = 4;
    state_s1 = 1;
  }
  // transition out
   if(DebugOn == true) {
    Serial.print  ("            transition to state_S1 = ");
    Serial.println(state_s1);
  }
}//end of function SM_case_4

//////////////////////////
//  Function SM_case_5  //
//////////////////////////
void SM_case_5() {  //MONITOR TO STOP RETRACT
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.println("SM_case_5 - Monitor to Stop RETRACT");
    Serial.print("            CurrentPosition = ");
    Serial.print(CurrentPosition);
    Serial.print(", CommandPosition = ");
    Serial.print(CommandPosition);
    Serial.print(", AtRetract = ");
    Serial.println(AtRetract);
  }
  if(DeepDebugOn == true) {
    Serial.println("            CYCLE SWITCH IN NEXT 3 SECONDS");
    delay(3000);                // allow time to manually operate RETRACT stop switch
  }

  // (1) test the AtRetract state input (HIGH is normally-closed, LOW is operated OPEN)
  AtRetract = digitalRead(AtRetractPin);
  if(AtRetract == HIGH) {
    StopNow = true;
    Serial.println("            AtRetract Microswitch stops the RETRACT");
  }
  
  // (2) test the potentiometer input (0, 1, 2, 3, ... 100) current physical position
  GetCurrentPosition();
  if(CurrentPosition <= CommandPosition) {
    StopNow = true;
    Serial.println("            CurrentPosition stops the RETRACT");
  }
  
  // (3) detect a touch on the STOP capacitive touch button 
  if(touchRead(T6) < StopNowMaxPosCapValue) {
    StopNow = true;
    if(DebugOn == true) {
      Serial.print("            STOP button ends RETRACT, touchRead(6) = ");
      Serial.println(touchRead(6));
    }
  }
  // do the stop
  if(StopNow == true) {
    digitalWrite(MotorDriverAIN1pin, LOW);
    digitalWrite(MotorDriverAIN2pin, LOW);
    ledcWrite(pwmChannel,dutyCycle);         // ***
    Serial.print("            Current Position (at end of RETRACT) = ");
    Serial.println(CurrentPosition);
    // transition out
    StopNow = false;
    state_s1_last = 5;
    state_s1 = 1;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
      Serial.println("============ What Next? ============");
    } else {
       Serial.println("            Stop condition is not yet met");
    }
  } else {
    state_s1_last = 5;
    state_s1 = 5;
  }
}//end of function SM_case_5

//////////////////////////
//  Function SM_case_6  //
//////////////////////////
void SM_case_6(){  //SETUP_EXTEND_MOVE
  if(DebugOn == true) {
    Serial.println("SM_case_6 - Setup EXTEND move");
  }
  // case actions

  // transition out
  state_s1_last = 6;
  state_s1 = 7;
  if(DebugOn == true) {
    Serial.print  ("            transition to state_S1 = ");
    Serial.println(state_s1);
  }
}//end of function SM_case_6

//////////////////////////
//  Function SM_case_7  //
//////////////////////////
void SM_case_7(){  //START_EXTEND_MOVE
  if(DebugOn == true) {
    Serial.println("SM_case_7 - Start EXTEND Move");
    Serial.print  ("            Extending now from ");
    Serial.print("CurrentPosition = ");
    Serial.print(CurrentPosition);
    Serial.print(" to CommandPosition = ");
    Serial.println(CommandPosition);
  }
  // case actions
  // First check to see AtExtend Microswitch is armed to trigger the stop
  AtExtend = digitalRead(AtExtendPin);
  //test
  if(AtExtend == LOW) {
    // AtExtend microswitch is armed to stop the move so turn on motor CCW
    digitalWrite(MotorDriverAIN1pin, LOW);
    digitalWrite(MotorDriverAIN2pin, HIGH);
    ledcWrite(pwmChannel,dutyCycle);         // ***
    // display the move on the serial monitor
    if(DeepDebugOn == true) {
      for(int i = CurrentPosition; i <= CommandPosition; i--) {
      DisplayShade(i);
      delay(100);
      }
    }
    state_s1_last = 7;
    state_s1 = 8;
  } else {
    // Possible problem with AtExtend microswitch so don't make the move
    Serial.println("            Problem with AtExtend microswitch - don't start EXTEND move");
    // transition out
    state_s1_last = 7;
    state_s1 = 1;
  }
  // transition out
   if(DebugOn == true) {
    Serial.print  ("            transition to state_S1 = ");
    Serial.println(state_s1);
  }
}//end of function SM_case_7
  
//////////////////////////
//  Function SM_case_8  //
//////////////////////////
void SM_case_8(){  //EXTEND_STOP
  if(DebugOn == true && state_s1 != state_s1_last) {
    Serial.print  ("SM_case_8 - Monitor to Stop EXTEND");
    Serial.print("            CurrentPosition = ");
    Serial.print(CurrentPosition);
    Serial.print(", CommandPosition = ");
    Serial.print(CommandPosition);
    Serial.print(", AtExtend = ");
    Serial.println(AtExtend);
  }
  if(DeepDebugOn == true) {
    Serial.println("            CYCLE SWITCH IN NEXT 3 SECONDS");
    delay(3000);                // allow time to manually operate EXTEND stop switch
  }

  // (1) test the AtExtend state input (HIGH is normally-closed, LOW is operated OPEN)
  AtExtend = digitalRead(AtExtendPin);
  if(AtExtend == HIGH) {
    StopNow = true;
    Serial.println("            AtExtend Microswitch stops the EXTEND");
  }
  
  // (2) test the potentiometer input (0, 1, 2, 3, ... 100) current physical position
  GetCurrentPosition();
  if(CurrentPosition >= CommandPosition) {
    StopNow = true;
    Serial.println("            CurrentPosition stops the EXTEND");
  }
  
  // (3) detect a touch on the STOP capacitive touch button 
  if(touchRead(T6) < StopNowMaxPosCapValue) {
    StopNow = true;
    if(DebugOn == true) {
      Serial.print("            STOP button ends EXTEND, touchRead(6) = ");
      Serial.println(touchRead(6));
    }
  }
  // do the stop
  if(StopNow == true) {
    digitalWrite(MotorDriverAIN1pin, LOW);
    digitalWrite(MotorDriverAIN2pin, LOW);
    ledcWrite(pwmChannel,dutyCycle);         // ***
    if(CurrentPosition <= CommandPosition) Serial.println("            CurrentPosition stops the EXTEND");
    if(AtExtend == HIGH) Serial.println("            AtExtend Microswitch stops the EXTEND");
    Serial.print("            Current Position (at end of EXTEND) = ");
    Serial.println(CurrentPosition);
    // transition out
    StopNow = false;
    state_s1_last = 8;
    state_s1 = 1;
    if(DebugOn == true) {
      Serial.print  ("            transition to state_S1 = ");
      Serial.println(state_s1);
      Serial.println("============ What Next? ============");
    } else {
       Serial.println("            Stop condition is not yet met");
    }
  } else {
    state_s1_last = 8;
    state_s1 = 8;
  }
}//end of function SM_case_5

////////////////////////////////////////////
//  Function ReceiveSerialInput
///////////////////////////////////////////

void ReceiveSerialInput () {
  if(Serial.available()){
    SerialCommandInput = Serial.readStringUntil('\n');
    ParseSerialInput();
  }
}//end of function ReceiveSerialInput

////////////////////////////////////////////
//  Function ParseSerialInput
///////////////////////////////////////////
void ParseSerialInput () {
//  SerialCommandLength = SerialCommandInput.length();
//  LongCommandPosition = SerialCommandInput.toInt();
//  if(LongCommandPosition >= 0 && LongCommandPosition <= 100) {
//    Serial.print("Valid Serial Command Input = ");
//    Serial.println(LongCommandPosition);
//    DisplayCommandInput();
//  }else{
//    Serial.print("SerialCommandInput parsing error = ");
//    Serial.println(SerialCommandInput);
//  }
}//end of function ReceiveSerialInput

////////////////////////////////////////////
//  Function DisplayShade
///////////////////////////////////////////
void DisplayShade(int Position){
  Serial.print("            *** 100 ");
  for(int i = 1; i <= 100-Position; i++) {
    Serial.print(" ");
  }
  for(int i = 1; i <= Position; i++) {
    Serial.print("|");
  }
  Serial.print(" 0 *** ");
  if(Position >=10 && Position <=99) Serial.print(" ");
  if(Position >=0 && Position <=9)   Serial.print("  "); 
  Serial.println(Position);
}//end of function DisplayShade


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


////////////////////////////
//  Function GetWebInput  //
////////////////////////////
void GetWebInput(){
// ****
  // Listen for incoming clients
  WiFiClient client = server.available();   
  
  // Client Connected
  if (client) {                             
    // Set timer references
    currentTime = millis();
    previousTime = currentTime;
    
    // Print to serial port
    Serial.println("New Client."); 
    
    // String to hold data from client
    String currentLine = ""; 
    
    // Do while client is cponnected
    while (client.connected() && currentTime - previousTime <= timeoutTime) { 
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
        
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK) and a content-type
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Display the HTML web page
            
            // HTML Header
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            
            // CSS - Modify as desired
            client.println("<style>body { text-align: center; font-family: \"Trebuchet MS\", Arial; margin-left:auto; margin-right:auto; }");
            client.println(".slider { -webkit-appearance: none; width: 300px; height: 25px; border-radius: 10px; background: #ffffff; outline: none;  opacity: 0.7;-webkit-transition: .2s;  transition: opacity .2s;}");
            client.println(".slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; border-radius: 50%; background: #ff3410; cursor: pointer; }</style>");
            
            // Get JQuery
            client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
                     
            // Page title
            client.println("</head><body style=\"background-color:#70cfff;\"><h1 style=\"color:#000000;\">ACME Window Shade</h1>");
            
            // Position display
            client.println("<h2 style=\"color:#000000;\">Position: <span id=\"servoPos\"></span>&#176;</h2>"); 
                     
            // Slider control
            client.println("<input type=\"range\" min=\"0\" max=\"100\" class=\"slider\" id=\"servoSlider\" onchange=\"servo(this.value)\" value=\""+valueString+"\"/>");
            
            // Javascript
            client.println("<script>var slider = document.getElementById(\"servoSlider\");");
            client.println("var servoP = document.getElementById(\"servoPos\"); servoP.innerHTML = slider.value;");
            client.println("slider.oninput = function() { slider.value = this.value; servoP.innerHTML = this.value; }");
            client.println("$.ajaxSetup({timeout:1000}); function servo(pos) { ");
            client.println("$.get(\"/?value=\" + pos + \"&\"); {Connection: close};}</script>");
            
            // End page
            client.println("</body></html>");     
            
            // GET data
            if(header.indexOf("GET /?value=")>=0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');

// !!!!! my code start
              valueStringLast = valueString;
// !!!!! my code end

              // String with web input position
              valueString = header.substring(pos1+1, pos2);
              
              // Move servo into position
// my change  myservo.write(valueString.toInt());

// !!!!! my code start              
              if(valueString != valueStringLast) { 
                // new CommandPosition received from web input
                NewWebInput = true;
                if(DeepDebugOn == true) {
                  Serial.print("            ***** Web input in origin code valueString = ");
                  Serial.println(valueString); 
                }
              } else {
                NewWebInput = false;
              }
// !!!!! my code end

            }         
            // The HTTP response ends with another blank line
            client.println();
            
            // Break out of the while loop
            break;
          
          } else { 
            // New lline is received, clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }


// ****  
}
