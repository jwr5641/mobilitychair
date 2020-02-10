/*  Features Implemented:
 *    PID control and encoder feedback of two motors, forward only
 *    Inputs are still sketched as physical buttons
 *    
 *  Features Not Yet Implemented: 
 *    - Gyroscope / Tilt Detection
 *    - IR distance sensing
 *    - Battery Voltage sensing / low battery indicator
 *    - Audible Feedback for motion
 *    - Full directional control
 *    - Proximity of Enclosure
 *    - Seat presence / weight detection
 *    - Therapist override
 *    - Mode selection
 */
//PINOUT DEFINITIONS/////////////////////////////////////////////////////////////////////////////////////
  //Control Inputs
const byte forwardButtonPin = 5;                    //Set the pin used for the input button
const byte leftButtonPin = 6;
const byte rightButtonPin = 7; 
const byte reverseButtonPin = 8;
/* The direction inputs will all eventually be replaced by the I2C communication of the capacitive touch*/
const byte mode1Pin = 32;                           //Used for selecting mode 1 on rotary switch
const byte mode2Pin = 33;                           //Used for selecting mode 2 on rotary switch
const byte mode3Pin = 34;                           //Used for selecting mode 3 on rotary switch
const byte enableButtonPin = 2;                     //Used for button that enables motion control (controlled by supervisor)
  //Status Indicators (Outputs)
const byte systemEnabledIndPin = 30;                //Used for LED lights that indicate motion control is activated
const byte lowBatIndPin = 35;                       //Used for LED lights that indicate a low battery level
const byte leftIndPin = 26;                         //Used for LED lights that indicate left has been pressed
const byte rightIndPin = 27;                        //Used for LED lights that indicate right has been pressed 
const byte forwardIndPin = 28;                      //Used for LED lights that indicate forward has been pressed
const byte reverseIndPin = 29;                      //Used for LED lights that indicate reverse has been pressed
const byte audioIndPin = 31;                        //Reserved for audio feedback
  //Motor Outputs
const byte leftMotorPWMPin = 4;                     //Used for left motor PWM constrol signal
const byte leftMtrDirPin = 25;                      //Pin used to signal rotation of left motor
const byte rightMotorPWMPin = 3;                    //Used for right motor PWM constrol signal
const byte rightMtrDirPin = 24;                    //Pin used to signal rotation of left motor
  //Encoder Inputs
const byte encoderLeftAPin = 14;                    //Encoder channel A for Left Motor - Note: This will be accessed by the interrupt only
const byte encoderLeftBPin = 15;                    //Encoder channel B for Left Motor
const byte encoderRightAPin = 16;                   //Encoder channel A for Right Motor
const byte encoderRightBPin = 17;                   //Encoder channel B for Right Motor
  //Sensor Inputs
const byte sclPin = 21;                             //Used for the gyroscope and the capacitive touch
const byte sdaPin = 20;                             //Used for the gyroscope and the capacitive touch
const byte distIR1Pin = A0;                         //Used for the IR sensor
const byte distIR2Pin = A1;                         //Used for the second IR sensor
const byte proxEncPin = A3;                         //Used for the prox sensor that monitors the enclosure
const byte seatSensePin = A4;                       //Used for the sensor that detects weight of seat/passenger
const byte lowBatSensePin = A5;                     //Used for the voltage sensor that detects low battery level
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//HARDWARE RELATED CONSTANTS/////////////////////////////////////////////////////////////////////////////
const int encoderResolution = 360;                  //Number of pulses for a single resolution (used to calculate RPM for troubleshooting/reference)
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//STATUS VARIABLES///////////////////////////////////////////////////////////////////////////////////////
  //Control Input Variables
boolean fwdbtnState_crnt = 0;                       // current state of the forward button
boolean fwdbtnState_prev = 0;                       // previous state of the forward button
boolean leftBtnState_crnt = 0;
boolean leftBtnState_prev = 0;

  //Motor and Encoder Status Variables
boolean stateEncoderL_B = 0;                        // State of the quatrature channel for Left motor encoder (used to determin direction of rotation)
boolean stateEncoderR_B = 0;                        // State of the quatrature channel for Left motor encoder (used to determin direction of rotation)
volatile unsigned int encoderLeftValue_counter = 0;   //The current count of the left encoder (updated instantly by interrupt)
volatile unsigned int encoderRightValue_counter = 0;   //The current count of the right encoder (updated instantly by interrupt)
int encoderLeftValue_crnt = 0;                      //Count of left encoder after 50ms (updated on time basis)
int encoderLeftValue_prev = 0;                      //Last count of the left encoder
String encoderLeftDir_txt = "";
int encoderRightValue_crnt = 0;                     //Count of left encoder after 50ms (updated on time basis)
int encoderRightValue_prev = 0;                     //Last count of the left encoder
String encoderRightDir_txt = "";
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//TIMING VARIABLES///////////////////////////////////////////////////////////////////////////////////////
//time variables must use long type because they will overflow if other datatypes are used
unsigned long fwdPressedTime = 0;     // the moment when fwrd button was pressed
unsigned long fwdReleasedTime = 0;    // the moment when fwd button was released
unsigned long leftPressedTime = 0;
unsigned long leftReleasedTime = 0;
unsigned long timeHeld = 0;           // the duration the button has currently been held (while active)
unsigned long timeReleased = 0;       // the duration of time since the button was released (while inactive)
unsigned long timeCurrent = 0;        // the length of time the button has been held
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//TIMING CONSTANTS///////////////////////////////////////////////////////////////////////////////////////
const int shortInterval = 50;         // Refresh rate for items checked often, in ms (note if changed from 50 array values need to be recalculated)
const int longInterval = 100;         // Refresh rate for items checked less often 
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//MOTOR SPEED and DIRECTION VARIABLES///////////////////////////////////////////////////////////////////
int targetSpeed_crnt = 0;                  // This value is dynamically adjusted based on the time the control inputs have been held/released
int targetSpeed_prev = 0;
int targetSpeedMapped = 0;            //Note eventually this will need to be converted to a value from 0-255 for PWM

  //LeftMotor
String leftMotorDirection = "";       //Used for troubleshooting detected rotation
String rightMotorDirection = "";      //Used for troubleshooting detected rotation
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//MOTOR SPEED RELATED CONSTANTS/////////////////////////////////////////////////////////////////////////
const int maxSpeed = 243;             //Maximum target speed in pulses per 50ms (based on expected pulses from encoder)
const int accelTime = 4000;           //Target time to reach maximum speed in ms (controls rate of acceleration - note, if this changes array must be recreated)
const int decelTime = 900;            //Target time to reach a stop in ms (rate of deceleration of the motor)
const int AccelSpeedValues[] = {1,1,1,1,1,2,2,2,3,4,4,6,7,9,10,12,14,17,20,22,26,29,33,36,41,45,49,54,59,64,69,75,80,86,92,98,103,109,116,122,128,134,140,146,151,157,163,168,174,179,184,189,194,198,203,207,210,214,217,221,224,226,229,231,233,235,236,237,239,240,240,241,242,242,242,242,243,243,243,243};
const int DecelSpeedValues[18] = {238,232,221,207,190,169,146,122,98,75,54,37,23,12,6,2,1,0};
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//PID CONTROL CONSTANTS//////////////////////////////////////////////////////////////////////////////////
float kp = 3.5; //These values will need some tuning
float ki = 0.2;
float kd = 5.0;
float PID_Lp, PID_Li, PID_Ld;
int PID_Ltotal = 0;
float PID_Rp, PID_Ri, PID_Rd;
int PID_Rtotal = 0;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//PID CONTROL VARIABLES//////////////////////////////////////////////////////////////////////////////////
int errorLeft_crnt;
int errorLeft_prev;
int errorRight_crnt;
int errorRight_prev;
int errorBetween_crnt;
int errorBetween_prev;
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


//*************BEGIN PROGRAM EXECUTION*******************************************************************
//*******************************************************************************************************
//THIS SETS UP THE INPUT AND OUTPUTS AND SERIAL MONITOR COMMUNICATION
void setup() {
  //Initialize the control input buttons
  pinMode(forwardButtonPin, INPUT_PULLUP); // initialize the button pin as a input
  pinMode(leftButtonPin, INPUT_PULLUP); // initialize the button pin as a input
  
  //Initialize the outputs for the motor controller
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMtrDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMtrDirPin, OUTPUT);
  
  //Initialize the inputs for the motor encoders
  pinMode(encoderLeftAPin, INPUT_PULLUP);
  pinMode(encoderLeftBPin, INPUT_PULLUP);
  pinMode(encoderRightAPin, INPUT_PULLUP);
  pinMode(encoderRightBPin, INPUT_PULLUP);

  //SETUP INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(encoderLeftAPin), readLeftMtrSpd, RISING);      //Whenever the Left encoder has a rising pulse call the routine
  attachInterrupt(digitalPinToInterrupt(encoderRightAPin), readRightMtrSpd, RISING);    //Whenever the Right encoder has a rising pulse call the routine
  
  //Start the serial monitor for troubleshooting after everything else has been setup
  Serial.begin(9600);        // initialize serial communication
  //Serial.end(9600);        // Uncomment to turn off serial comm
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*--MAIN PROGRAM LOOP---MAIN PROGRAM LOOP---MAIN PROGRAM LOOP---MAIN PROGRAM LOOP---MAIN PROGRAM LOOP---MAIN PROGRAM LOOP------------------*/
//THIS IS THE MAIN PROGRAM LOOP (it calls all the subroutines)
void loop() {
  //Update the control inputs as soon as they occur
  controlLoop();

  //Update these functions only every 50ms
  if (millis() > timeCurrent + shortInterval) {
    timeCurrent = millis(); //Update the system timer every 50ms
    updateTimerValue();     //Duration controls pressed or released
    updateTargetSpeed();    //Desired target speed value based on encoder
    calcMtrSpd();           //Read the actual encoder speed value
    pidLoop();              //Motor PID control
  }
  //Update these functions every 100ms
  if (millis() > timeCurrent + longInterval) {
    //Do nothing yet
  }
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void controlLoop() {
  //Not timer dependent

  //Read the control input pins
  fwdbtnState_crnt = digitalRead(forwardButtonPin);     // read the forward button input
  leftBtnState_crnt = digitalRead(leftButtonPin);       // read the left button input

  //If a change in the control has occurred (first time pressed or released), keep track of the time
  if (fwdbtnState_crnt != fwdbtnState_prev || leftBtnState_crnt != leftBtnState_prev) {           // A change is detected from last iteration (either pressed or released)
     updateStartStopTimes(); 
  }
  
  fwdbtnState_prev = fwdbtnState_crnt;                  // save fwdbtn state for next loop so that a change can be detected each iteration
  leftBtnState_prev = leftBtnState_crnt;

  //Write the motor controller values

  //Left Motor Values
  byte leftMotorPWM_value = abs(PID_Ltotal);            //PWM value can only be positive (but direction is controlled by sign) 
  if (PID_Ltotal < 0) {                                 //Set the motor direction based on sign of PWM value
    digitalWrite(leftMtrDirPin, HIGH);                  //Set the motor direction
  } else {
    digitalWrite(leftMtrDirPin, LOW);
  }
  analogWrite(leftMotorPWMPin, leftMotorPWM_value);      //Set the motor speed based on the target calculated (first pass is zero)
  
  //Right Motor Values
  byte rightMotorPWM_value = abs(PID_Rtotal); 
  if (PID_Rtotal < 0) {                                 //Set the motor direction based on sign of PWM value
    digitalWrite(rightMtrDirPin, HIGH);                  //Set the motor direction
  } else {
    digitalWrite(rightMtrDirPin, LOW);
  }
  analogWrite(rightMotorPWMPin, rightMotorPWM_value);      //Set the motor speed based on the target calculated (first pass is zero)
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
//THIS UPDATES THE START AND STOP TIMES STORED AS VARIABLES IN THE GLOBAL SCOPE
void updateStartStopTimes() {
  // Not timer dependent, is called by controlLoop()
  //Runs the instant the control is pressed or released, PULLUP so LOW means pressed
  
  //Forward control 
  if (fwdbtnState_crnt == LOW) {
      fwdPressedTime = millis();              //Stores the instant forward button was pressed
      } else {                                // the button was just released as the other state is HIGH
        fwdReleasedTime = millis();           //Stores the instant the forward button was released
  }

  //Left Control
  if (leftBtnState_crnt == LOW) {
      leftPressedTime = millis();              //Stores the instant forward button was pressed
      } else {                                // the button was just released as the other state is HIGH
        leftReleasedTime = millis();           //Stores the instant the forward button was released
  }  
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void updateTimerValue() {
  //Timer Dependent
  // This function runs ALL THE TIME but only calculates a time while the forward button is actively being pressed and for one cycle after its release
  //if (millis() > timeCurrent + shortInterval) {  // Only update the timers every 50ms
    if (fwdbtnState_crnt == fwdbtnState_prev && fwdbtnState_crnt == 0) {  // FWD Button is active and loop has passed at least once
      //timeCurrent = millis();
      timeHeld = timeCurrent - fwdPressedTime;
      timeReleased = 0;
      //Serial.print("The button has been held for ");
      //Serial.print(timeHeld);
      //Serial.println(" ms.");
      }
    if (fwdbtnState_crnt == fwdbtnState_prev && fwdbtnState_crnt == 1) { //FWD Button is inactive and loop has passed at least once
      timeCurrent = millis();
      timeReleased = timeCurrent - fwdReleasedTime;
      //Serial.print("The button has been released for ");
      //Serial.print(timeReleased);
      //Serial.println(" ms.");  
    }
    //}  
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void updateTargetSpeed() {
  //Timer dependent, called by main Void loop every 50ms

  //Set the target speed to zero if the button has not been pressed
  if (timeHeld == 0) {
    targetSpeed_crnt = 0;
    targetSpeedMapped = 0;
  }

  //Set the target speed based on the acceleration profile if the button has been held for a duration within time to accelerate
  if (timeHeld < accelTime && timeHeld > 0 && fwdbtnState_crnt == 0) {
    targetSpeed_crnt = AccelSpeedValues[timeHeld / 50 - 1];     //The speed is selected based on the index of the array
    targetSpeedMapped = map(targetSpeed_crnt, 1, maxSpeed, 1, 255); 
  }
  //Set the target speed to the maximum value if system should have reached full speed (accel time over)
  if (timeHeld > accelTime && fwdbtnState_crnt == 0) {
    targetSpeed_crnt = maxSpeed;
    targetSpeedMapped = 255;   
  }

  //Decelerate gently by following the deceleration profile after the button is released
  if (timeReleased < decelTime && timeReleased > 0 && fwdbtnState_crnt == 1) {
    for (int i = 0; i<18; i++) {  //i is the number of elemens in the decelSpeedValue array
        if (targetSpeed_crnt > DecelSpeedValues[i]) {
          targetSpeed_crnt = DecelSpeedValues[i];
          break;
        }
    }
    //This is currently used to ramp the motor controller to max speed instead of using the PID value
    targetSpeedMapped = map(targetSpeed_crnt, 1, maxSpeed, 1, 255); 
  }
  if (timeReleased > decelTime && fwdbtnState_crnt == 1) {
    targetSpeed_crnt = 0;
    targetSpeedMapped = 0;   
  }
  if (targetSpeed_prev != targetSpeed_crnt) {               //Only update the serial monitor if the value changed
    Serial.print("The target velocity is ");
    Serial.print('\t');
    Serial.println(targetSpeed_crnt);
    //Serial.print(" pulses per 50ms.  Mapped: ");
    //Serial.println(targetSpeedMapped);  
  }
 targetSpeed_prev = targetSpeed_crnt;
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void readLeftMtrSpd() {
  // Increment value for each pulse from encoder, this is triggered by an interrupt
  encoderLeftValue_counter++;
  stateEncoderL_B = digitalRead(encoderLeftBPin);
}

void readRightMtrSpd() {
  // Increment value for each pulse from encoder, this is triggered by an interrupt
  encoderRightValue_counter++;
  stateEncoderR_B = digitalRead(encoderRightBPin);
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void calcMtrSpd() {
  //Left Encoder
  encoderLeftValue_prev = encoderLeftValue_crnt;              //Update the previous value with the current value
  encoderLeftValue_crnt = encoderLeftValue_counter;           //Store how many pulses occurred for calculation 
  encoderLeftValue_counter = 0;                               //Reset the encoder value each loop (after 50ms)
  //Right Encoder
  encoderRightValue_prev = encoderRightValue_crnt;            //Update the previous value with the current value
  encoderRightValue_crnt = encoderRightValue_counter;         //Store how many pulses occurred for calculation 
  encoderRightValue_counter = 0;                              //Reset the encoder value each loop (after 50ms)
  
  if(stateEncoderL_B == LOW) {                                //Determine the direction the LEFT motor is spining based on quadrature signal
      encoderLeftDir_txt = "CW ";
  } else {
      encoderLeftValue_crnt = encoderLeftValue_crnt * -1;     //Set the encoder value to negative if CCW rotation
      encoderLeftDir_txt = "CCW";
  }
  
  if(stateEncoderR_B == LOW) {                                //Determine the direction the RIGHT motor is spining based on quadrature signal
      encoderRightDir_txt = "CW ";
  } else {
      encoderRightValue_crnt = encoderRightValue_crnt * -1;     //Set the encoder value to negative if CCW rotation
      encoderRightDir_txt = "CCW";
  }
  
  if (encoderLeftValue_prev != encoderLeftValue_crnt || encoderRightValue_prev != encoderRightValue_crnt) {       //Only update the serial monitor if there was a change
    Serial.print("PULSES per 50ms[L/R]: ");
    Serial.print('\t');
    Serial.print(encoderLeftValue_crnt);
    Serial.print('\t');
    Serial.print(encoderLeftDir_txt);
    Serial.print(" / ");
    Serial.print(encoderRightValue_crnt);
    Serial.print('\t');
    Serial.println(encoderRightDir_txt);      
  }
}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void pidLoop() {
  //Get the error for each motor setpoint
  errorLeft_crnt = targetSpeed_crnt - encoderLeftValue_crnt;
  errorRight_crnt = targetSpeed_crnt - encoderRightValue_crnt;

  //Calculate Proportional Term
  PID_Lp = kp*(float)errorLeft_crnt;
  PID_Rp = kp*(float)errorRight_crnt;

  //Calculate Derivative Term
  int errorLeft_diff = errorLeft_crnt - errorLeft_prev;
  PID_Ld = kd*(((float)errorLeft_crnt - (float)errorLeft_prev)/(float)shortInterval);

  int errorRight_diff = errorRight_crnt - errorRight_prev;
  PID_Rd = kd*(((float)errorRight_crnt - (float)errorRight_prev)/(float)shortInterval);

  //Calculate Left Integral Term
  if (-3 < errorLeft_crnt && errorLeft_crnt < 3) {
      PID_Li = PID_Li + (ki * (float)errorLeft_crnt);
  
  } else {
    
    PID_Li = 0;  //When both motors are added, this is probably the place to compare their speed
    
    }
  //Calculate Right Integral Term
  if (-3 < errorRight_crnt && errorRight_crnt < 3) {
      PID_Ri = PID_Ri + (ki * (float)errorRight_crnt);
  
  } else {
    
    PID_Ri = 0;  //When both motors are added, this is probably the place to compare their speed
    
    }
  
  //Sum for total PID control signal
  PID_Ltotal = PID_Lp + PID_Li + PID_Ld;
  PID_Rtotal = PID_Rp + PID_Ri + PID_Rd;
  
  //The motor PWM value cannot exceed 255 (if it does it is maxed out)
  if (PID_Ltotal > 255) {
    PID_Ltotal = 255;
  }
  
  if (PID_Rtotal > 255) {
    PID_Rtotal = 255;
  }

  
  Serial.print("PID Values [L/R]: ");
  Serial.print('\t');
  Serial.print(PID_Ltotal);
  Serial.print(" / ");
  Serial.println(PID_Rtotal);
}
