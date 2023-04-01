/*
 * FINAL ROBOT PROJECT - Vineet Ranade, Peter Sorenson, Jacob Rast
 * This program allows a robot to be controlled with an infrared remote and PS2 controller. The robot can also
 * autonomously follow a line and deliver an object to a rescue zone using the gripper.
 * The relevant hardware includes the robot itself with an MSP432p401r mounted on it and various components wired on a breadboard.
 * These components include a bluetooth device for Serial Monitor output, an infrared sensor for the infrared remote, the PS2
 * bluetooth dongle for PS2 control, and three-pin infrared sensors for distance sensing. Note that we did not use the distance sensors
 * in our final version of the code; we only used line following in the autonomous mode.
 * The primary functions include autoDelivery(), which autonomously delivers an object to the rescue zone with the help of the 
 * lineFollow() function, which adjusts the movement of the robot based on its location relative to a line. The PS2Mode() function
 * handles PS2 controller movement, and the PS2Move() function handles the joystick movement on the controller for smooth control of 
 * the robot. The IRMode() function handles the robot's movement based on which button on the infrared remote is pressed. Variables and
 * macros were used to account for the physical characteristics of the robot, for state machines, and for certain speeds or cutoffs used
 * when determining how the robot should move.
 */

/****************/
/*****SETUP******/
/****************/

/* TIME MACROS */
#define MS   1000                           // we use delayMicroseconds to avoid sleeping the MSP432 so we define a constant for readable conversion from milli to micro
#define ZERO  0                             // create these macros to avoid magic numbers in code
#define TWO   2
#define THREE 3
#define FOUR  4

/* PS2 PIN MACROS */
#define PS2_DAT 14                          // P1.7 <-> brown wire
#define PS2_CMD 15                          // P1.6 <-> orange wire
#define PS2_SEL 34                          // P2.3 <-> yellow wire (also called attention)
#define PS2_CLK 35                          // P6.7 <-> blue wire

/* IR BUTTON MACROS */
#define IR_UP      0x46                     // forward (treats gripper as the front...defining our infrared remote buttons in an intuitive way)
#define IR_DOWN    0x15                     // backward
#define IR_LEFT    0x44                     // spin left
#define IR_RIGHT   0x43                     // spin right
#define IR_OK      0x40                     // stop
#define IR_ONE     0x16                     // turn left
#define IR_THREE   0xD                      // turn right
#define IR_EIGHT   0x1C                     // change remote state from IR to auto
#define IR_STAR    0x42                     // open gripper incrementally
#define IR_ZERO    0x52                     // change state from IR to PS2
#define IR_HASH    0x4A                     // close gripper decrementally
#define IR_FIVE    0x18                     // change state from auto to IR

/* DIRECTIONAL MACROS */
#define RIGHT 0                             // we use 0 for right turns/spins
#define LEFT  1                             // we use 1 for left turns/spins

/* REMOTE STATE MACHINE */
#define REMOTE  0                           // the infrared remote state is 0 (defining our states for type of control)
#define PS2     1                           // the PS2 controller state is 1
#define AUTO    2                           // the automatic movement state is 2
int STATE = PS2;                            // sets the starting state to be PS2 controlled

/* LINE FOLLOW STATE MACHINE */
#define ON         1                        // we use 1 for on the line
#define SPINRIGHT  2                        // we use 2 for spinning right at a corner
#define SPINLEFT   3                        // we use 3 for spinning left at a corner
#define DROPOFF    4                        // we use 4 for autonomous pickup or dropoff
#define LINERIGHT  5                        // we use 5 for the line being a bit to the robot's right
#define LINELEFT   6                        // we use 6 for the line being a bit to the robot's left
int CURSTATE = ON;                          // sets starting state arbitrarily to be on the line (the state machine will adjust accordingly anyway)

/* LIBRARY IMPORT STATEMENTS */
#include "SimpleRSLK.h"                     // import SimpleRSLK library for robot
#include "Servo.h"                          // import Servo library for gripper
#include "Romi_Motor_Power.h"               // import Romi_Motor_Power library for movement
#include "TinyIR.h"                         // import infrared sensor library
#include "PS2X_lib.h"                       // import PS2 library

/* GRIPPER VARIABLES */
const int closedGripperAngle = 140;         // write this value to gripper to close it
const int openGripperAngle = 0;             // write this vallue to gripper to open it
const int gripperChange = 35;               // change the gripper angle in increments/decrements of 35 degrees
int gripperAngle = 0;                       // tracks gripper's angle

/* SPEED VARIABLES */
const int fullSpeed = 80;                   // full movement speed of robot
const int halfSpeed = 50;                   // smaller movement speed of robot

/* DELAY VARIABLES */
const int tenthSec = 100;                   // delay of 0.1 seconds
const int halfSec = 500;                    // delay of 0.5 seconds
const int oneSec = 1000;                    // delay of 1 second
const int remoteDelay = 25;                 // sets the 0.025 second delay for remote inputs
const int lineFollowDelay = 50;             // delay of 0.05 seconds

/* PIN DEFINITIONS */
const int IRpin = 33;                       // pin 5.1 on the board

/* CREACTION OF OBJECTS */
Romi_Motor_Power rightWheel;                // create rightWheel object from Romi_Motor_Power class
Romi_Motor_Power leftWheel;                 // create leftWheel object from Romi_Motor_Power class
Servo gripper;                              // create gripper object from Servo class
IRData IRresults;                           // global variable to track inputs from infrared remote
PS2X ps2x;                                  // create PS2 Controller Class

/* PS2 CONFIGURATION */
#define pressures false                     // don't need to take into account button pressure
#define rumble    false                     // don't need rumble

/* PS2 VARIABLES */
const int restingCutoff = 5;                // accounts for calibration errors for reading joystick position
const int stopCutoff = 100;                 // defines how far the right joystick needs to be pressed downwards to stop the robot's movement
const int minJoystick = 0;                  // minimum raw value from PS2 joysticks
const int maxJoystick = 255;                // maximum raw value from PS2 joysticks
const int stopConversion = 127;             // maximum mapped value for stopping
const int spinConversion = 100;             // maximum speed for spinning
const int forwardBackwardConversion = 150;  // maximum speed for forward/backward movement
const int turnConversion = 100;             // maximum speed for turning

/* LINE SENSOR LIBRARY VARIABLES */
uint16_t sensorVal[LS_NUM_SENSORS];         // creates an array of the values from the line sensors
uint16_t sensorCalVal[LS_NUM_SENSORS];      // an array of the calibrated values from the previous array
uint16_t sensorMaxVal[LS_NUM_SENSORS];      // the maximum sensor value from the line sensors
uint16_t sensorMinVal[LS_NUM_SENSORS];      // the minimum sensor value from the line sensors
const int numVals = 8;                      // number of values in sensorVal
const uint8_t lineColor = 1;                // white line of black background
const int calibrationValues = 100;          // the number of values taken from the line sensors during calibration
const int calibrateSpeed = 20;              // the speed that will be used when calibrating the line following sensors

/* PHYSICAL LINE FOLLOWING VARIABLES */
const int leftCutoff = 3000;                // if the linePos value is less than this and greater than zero, there is a line the the left of the center of the robot
const int rightCutoff = 3500;               // if the linePos value is greater than this, there is a line the the right of the center of the robot
const int fastLeftCutoff = 2900;            // if we're "locked" on a line, we will be a little more lenient to speed up the robot's progress on the mission
const int fastRightCutoff = 3600;           // if we're "locked" on a line, we will be a little more lenient to speed up the robot's progress on the mission
const int normalSpeed = 10;                 // the normal speed at which the wheels will turn when line following
const int fastSpeed = 16;                   // the faster speed at which the wheels will turn when adjusting its position a bit to stay on the line
const int spinTrigger = 2500;               // if a line sensor detects a line, it will return a value less than 2500; if it doesn't, it returns 2500
const int triggerCutoff = 4;                // if 5 of the 8 sensors see a line, it is the dropoff zone
int numTimesLocked = 0;                     // tracks the number of loops we've gone through while being in the ON state for following the line
const int lockedCutoff = 10;                // number of loops we need to be ON for to be considered "locked" on a line
bool highSpeed = false;                     // tracks whether or not the robot has "locked" on a line and is moving faster
const int lowSpeedForward = 2.5             // distance the robot moves forward after encountering a spin condition

/* ENCODER VARIABLES */
const float wheelDiameter = 2.7559055;      // the diameter of the wheel
const int cntPerRevolution = 360;           // degrees in a 360 degree spin
const float distAfterSpin = 1;              // the distance the robot will move after turning, so it doesn't trigger another spin accidentally
const float offset90 = 0.7;                 // the extra distance the robot travels after encountering a 90 degree corner so that it is centered on the next line
const float spinInches = 4.3;               // the distance the robot will spin for a 90 degree angle, in inches, along a circular path
const float spin180Inches = 8.7;            // the distance the robot will spin for a 180 degree angle, in inches, along a circular path
const float diff180 = 2;                    // helps with the slightly different dropoff process for the first spelunker
const int wheelSpeed = fullSpeed;           // initial wheelSpeed for encoder movement
const int slowWheelSpeed = halfSpeed;
const int fineWheelSpeed = 25;
uint16_t rCount = 0;                        // total amount of encoder pulses received from right wheel
uint16_t lCount = 0;                        // total amount of encoder pulses received from left wheel

/* Serial1 MONITOR CONFIGURATION */
const int baudRate = 9600;                  // set baud rate for the Seriall Monitor (bluetooth)

/* Line Calibration Tracking */
bool isCalibrationComplete = false;         // inital state of calibration; will make the robot line calibrate only once

void setup()
{
  /* Serial1 MONITOR */
  Serial1.begin(baudRate);                  // configure Serial1 Monitor with baudRate

  /* GRIPPER */
  gripper.attach(SRV_0);                    // this is pin 38, from RSLK_Pins.h

  /* WHEELS */
  rightWheel.begin(MOTOR_R_SLP_PIN,
                   MOTOR_R_DIR_PIN,
                   MOTOR_R_PWM_PIN);        // referred to RSLK_Pins.h for this initialization of the right wheel
  leftWheel.begin(MOTOR_L_SLP_PIN,
                  MOTOR_L_DIR_PIN,
                  MOTOR_L_PWM_PIN);         // referred to RSLK_Pins.h for this initialization of the left wheel

  stopMovement();                           // make sure the robot does not move before we start controlling it

  /* PINS */
  pinMode(IRpin, INPUT);                    // set IR pin as an input

  /* IR Receiver */
  initTinyIRReceiver();                     // initialize the infrared receiver

  /* PS2 */
  ps2x.config_gamepad(PS2_CLK,
                      PS2_CMD,
                      PS2_SEL,
                      PS2_DAT,
                      pressures,
                      rumble);              // configure the PS2 controller

  setupRSLK();                              // sets up the robot
  clearMinMax(sensorMinVal, sensorMaxVal);  // clears any stored min and max line sensor values

  delayMicroseconds(oneSec * MS);           // delays 1 second to ensure configuration is complete

  openGripperFull();                        // ensure the gripper is open before the robot starts its mission (first step is clearing debris)
}

/**************/
/*****LOOP*****/
/**************/
void loop()
{
  switch (STATE)                                            // check the current state...
  {
    case REMOTE:                                            // if we are in IR REMOTE state...
      if (decodeIR(&IRresults))                             // decodeIR() updates results and retunrs true if a new command is available...
      {
        IRMode();                                           // calls IRMode() to perform the appropriate function based on which infrared remote button is pressed
      }
      delayMicroseconds(remoteDelay * MS);                  // delays for 0.025 seconds so that the robot isn't overwhelmed with inputs
      break;                                                // go back
      
    case PS2:                                               // if we are in PS2 state...
      PS2Mode();                                            // calls PS2Mode() to perform the appropriate function based on inputs on the PS2 controller
      delayMicroseconds(remoteDelay * MS);                  // delays for 0.025 seconds so that the robot isn't overwhelmed with inputs
      break;                                                // go back

    case AUTO:                                              // if we are in AUTO state...
      checkPS2();                                           // check the PS2 for a potential state change (the X button)
      autoDelivery();                                       // calls the autoDelivery() function, delegating all autonomous movement to that function
      if (decodeIR(&IRresults))                             // decodeIR() updates results and retunrs true if a new command is available for a potential state change...
      {
        activateIR();                                       // calls activateIR() which will toggle state to REMOTE only if 5 was pressed
      }
      break;                                                // go back
  }
}

/***************/
/*****AUTO******/
/***************/

/*
 * This function autonomously delivers a splunker to the rescue zone
 */
void autoDelivery()
{
  checkPS2();                                               // check the PS2 for a potential state change (the X button)
  enableMotor(BOTH_MOTORS);
  CURSTATE = getLineState(leftCutoff, rightCutoff);         // gets the current line state and updates CURSTATE accordingly
  if (CURSTATE == SPINRIGHT)                                // if the corner is a 90 degree right spin...
  {
    if (adjustment())                                       // if we need to drop off...            
    {
      execute180();                                         // do a 180 and drop off the spelunker
    }
    else                                                    // otherwise, this is a regular corner...
    {
      executeSpin(RIGHT);                                   // execute a spin to the right
    }
  } 
  else if (CURSTATE == SPINLEFT)                            // otherwise, if the corner is a 90 degree left spin...
  {
    if (adjustment())                                       // if we need to drop off...
    {
      execute180();                                         // do a 180 and drop off the spelunker
    }
    else                                                    // otherwise, this is a regular corner...
    {
      executeSpin(LEFT);                                    // execute a spin to the left 
    }
  }
  else if (CURSTATE == DROPOFF)                             // otherwise, if we need to drop off the spelunker...
  {
    execute180();                                           // do a 180 and drop it off
  }
  else if (CURSTATE == LINELEFT  ||
             CURSTATE == LINERIGHT ||
             CURSTATE == ON)                                // otherwise, if we actually have to follow the line normally...
  {
    lineFollow();                                           // calls lineFollow() to handle line following
  }
  delayMicroseconds(lineFollowDelay * MS);                  // delay 0.05 seconds for stability
}

/*
 * This function is called after a required spin is encountered.
 * It goes a little bit forward to check if there is no line or the rescue zone.
 * If there is no line, we have a normal 90 degree spin because the line has turned a corner.
 * If all sensors detect a line, we have a dropoff required since we're in the rescue zone.
 * It returns true if the dropoff is required and false if we just need a normal 90 degree spin.
 */
bool adjustment()
{
  bool rescue = false;                                      // this variable determines our return value of whether or not we're in the rescue zone
  float forwardDistance = lowSpeedForward;                  // forward distance to travel if we were just moving slowly
  float backwardDistance = lowSpeedForward / TWO;           // backward distance to travel back if we were just moving slowly
  if (highSpeed)                                            // if we were just moving quickly...
  {
    forwardDistance = distAfterSpin;                        // forward distance to travel if we were just moving quickly
    backwardDistance = distAfterSpin;                       // backward distance to travel if we were just moving quickly
  }
  resetVars();                                              // reflects that we're no longer locked onto a line and also resets encoder pulses
  stopMovement();                                           // disable motors
  delayMicroseconds(tenthSec * MS);
  align();                                                  // make sure wheels are aligned
  delayMicroseconds(tenthSec * MS);                         // delay for stability
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);        // we will move forward
  goDistance(forwardDistance);                              // move forward a little
  if (dropOff())                                            // if all sensors see a line...
  {
    rescue = true;                                          // we're in the rescue zone
  }
  delayMicroseconds(tenthSec * MS);                         // delay for stability
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);       // we will move backward
  goDistance(backwardDistance);                             // go backward a little
  return rescue;                                            // return our final answer
}

void align()
{
  lCount = getEncoderLeftCnt();                             // get the left encoder pulses count, set to lCount variable
  rCount = getEncoderRightCnt();                            // get the right encoder pulses count, set to rCount variable
  while (lCount != rCount)                                  // while our wheels are misaligned...
  {
    checkPS2();                                             // check the PS2 for a potential state change (the X button)
    lCount = getEncoderLeftCnt();                           // get the left encoder pulses count, set to lCount variable
    rCount = getEncoderRightCnt();                          // get the right encoder pulses count, set to rCount variable
  
    if (lCount < rCount)                                    // if there are less left encoder pulses than right...
    {
      stopMovement();                                       // stop all movement
      spinOne(LEFT, fineWheelSpeed);                        // only spin the left wheel so it can catch up
    }
    else if (lCount > rCount)                               // if there are more left encoder pulses than right...
    {
      stopMovement();                                       // stop all movement
      spinOne(RIGHT, fineWheelSpeed);                       // only spin the right wheel so it can catch up
    }
    else                                                    // otherwise, there are an equal number of left and right encoder pulses...
    {
      spinBoth(fineWheelSpeed, fineWheelSpeed);             // spin both wheels at the preestablished wheelSpeed
    }
  }
  stopMovement();                                           // disable motors
}

/*
 * This function helps the robot follow a white line on a black background using the line sensors
 */
void lineFollow()
{
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  if (CURSTATE == LINELEFT)                                                                     // if the line is a little to the left...
  {
    setMotorSpeed(LEFT_MOTOR, normalSpeed);                                                     // left motor will spin at a normal speed
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);                                                      // right motor will spin at a fast speed to adjust
    resetVars();                                                                                // reflects that we're no longer locked onto a line and also resets encoder pulses
    highSpeed = false;
  }
  else if (CURSTATE == LINERIGHT)                                                               // otherwise, if the line is a little to the right...
  {
    setMotorSpeed(LEFT_MOTOR, fastSpeed);                                                       // left motor will spin at a fast speed to adjust
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);                                                    // right motor will spin at a normal speed
    resetVars();                                                                                // reflects that we're no longer locked onto a line and also resets encoder pulses
    highSpeed = false;
  }
  else if (CURSTATE == ON)                                                                      // otherwise, we know we are ON the line...
  {
    numTimesLocked++;                                                                           // increment the number of loops we've been locked ON the line
    if (numTimesLocked > lockedCutoff)                                                          // if we've been locked on for a decent amount of time (i.e. 10 iterations of line following)...
    {
      highSpeed = true;
      while (getLineState(fastLeftCutoff, fastRightCutoff) == ON)                               // while we're still on the line, using the lenient cutoffs
      {
        checkPS2();                                                                             // check the PS2 for a potential state change (the X button)
        if (STATE == PS2)                                                                       // if we did change states...
        {
          break;                                                                                // force this function to stop so we can switch to PS2Mode()
        }
        lCount = getEncoderLeftCnt();                                                           // get the left encoder pulses count, set to lCount variable
        rCount = getEncoderRightCnt();                                                          // get the right encoder pulses count, set to rCount variable
      
        if (lCount < rCount)                                                                    // if there are less left encoder pulses than right...
        {
          stopMovement();                                                                       // stop all movement
          spinOne(LEFT, wheelSpeed);                                                            // only spin the left wheel so it can catch up
        }
        else if (lCount > rCount)                                                               // if there are more left encoder pulses than right...
        {
          stopMovement();                                                                       // stop all movement
          spinOne(RIGHT, wheelSpeed);                                                           // only spin the right wheel so it can catch up
        }
        else                                                                                    // otherwise, there are an equal number of left and right encoder pulses...
        {
          spinBoth(wheelSpeed, wheelSpeed);                                                     // spin both wheels at the preestablished wheelSpeed
        }
      }
    }
    else                                                                                        // otherwise, we haven't been locked on the line long enough to increase movement speed...
    {
      checkPS2();                                                                               // check the PS2 for a potential state change (the X button)
      setMotorSpeed(LEFT_MOTOR, normalSpeed);                                                   // left motor will spin at a normal speed
      setMotorSpeed(RIGHT_MOTOR, normalSpeed);                                                  // right motor will spin at a normal speed
      resetLeftEncoderCnt();                                                                    // set the right encoder pulses count back to zero
      resetRightEncoderCnt();                                                                   // set the right encoder pulses count back to zero
      // we don't call resetVars() because we don't want to reset numTimesLocked!
    }
  }
}

/*
 * This function executes the following sequence:
 * 1) Stop moving forward
 * TWO) Open the gripper to let go of the spelunker
 * THREE) Move backwards a small amount
 * FOUR) Close the gripper, getting a better hold of it
 */
void regain()
{
  checkPS2();                                           // check the PS2 for a potential state change (the X button)
  stopMovement();                                       // disable motors
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  openGripperFull();                                    // open the gripper
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);   // we will go backward
  goDistance(offset90);                                 // go a small distance to get around the spelunker better
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  closeGripperFull();                                   // close the gripper, gaining a better hold of the spelunker
  delayMicroseconds(tenthSec * MS);                     // delay for stability
}

/*
 * This function checks the PS2 for the press of the X button to switch from AUTO to PS2 state
 */
void checkPS2()
{
  ps2x.read_gamepad();                                  // read the PS2 gamepad for a potential state change
  if (ps2x.ButtonPressed(PSB_CROSS))                    // if the X button on the PS2 controller has been pressed...
  {
    stopMovement();                                     // stop all movement for the moment
    STATE = PS2;                                        // toggle to PS2 state
    PS2Mode();                                          // immediately call PS2Mode() to override any remaining automatic functions
  }
}

/*
 * This function executes a 90 degree spin in the specified direction
 */
void executeSpin(int RL)
{
  checkPS2();                                           // check the PS2 for a potential state change (the X button)
  goDistance(offset90);                                 // go a small distance so we land on the middle of the next line
  resetVars();                                          // reflects that we're no longer locked onto a line and also resets encoder pulses
  stopMovement();                                       // disable motors
  if (RL == RIGHT)                                      // if we need to spin right...
  {
    spin90(RIGHT);                                      // spin right 90 degrees
  }
  else if (RL == LEFT)                                  // otherwise, if we need to spin left...
  {
    spin90(LEFT);                                       // spin left 90 degrees
  }
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);    // we will go forward
  goDistance(distAfterSpin);                            // goes a small extra distance to ensure that the robot locks on to the next line, not the previous one
}

/*
 * This function executes the following sequence:
 * 1) Stop moving forward
 * 2) Go backwards a small distance
 * 3) Stop moving backward
 * 4) Spin 180 degrees
 * 5) Drop off the spelunker by opening the gripper
 * 6) If this is the first delivery, move the spelunker to the side so it doesn't get in the way of future deliveries
 */
void execute180()
{
  checkPS2();                                           // check the PS2 for a potential state change (the X button)
  resetVars();                                          // reflects that we're no longer locked onto a line and also resets encoder pulses
  stopMovement();                                       // disable motors
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  align();                                              // make sure wheels are aligned
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);   // we will go backward
  goDistance(distAfterSpin * THREE);                    // go backwards a small distance so we don't hit the wall with the gripper while spinning
  stopMovement();                                       // disable motors
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  spin180(spin180Inches);                               // spin 180 degrees
  openGripperFull();                                    // open the gripper to drop off the spelunker
  delayMicroseconds(tenthSec * MS);                     // delay for stability
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);    // we will go forward
}

/*
 * This function returns the line state based on specified cutoffs
 * Note that there are some overrides necessary which are handled using the normal and finalStretch variables
 */
int getLineState(int lCutoff, int rCutoff)
{
  checkPS2();
  readLineSensor(sensorVal);                                              // gets values from line sensor value array
  readCalLineSensor(sensorVal,                                            // gets values from calibrated line sensor value array
            sensorCalVal,
            sensorMinVal,
            sensorMaxVal,
            lineColor);
  uint32_t linePos = getLinePosition(sensorCalVal,lineColor);             // gets a line position which is 0 or between 1000 - 7000
  if (sensorVal[5] < spinTrigger && 
      sensorVal[6] < spinTrigger && 
      sensorVal[7] < spinTrigger)                                         // if sensors on right side see a line...
  {
    return SPINRIGHT;                                                     // we will spin 90 degrees right
  }
  else if (sensorVal[0] < spinTrigger && 
           sensorVal[1] < spinTrigger && 
           sensorVal[2] < spinTrigger)                                    // otherwise, if sensors on left side see a line...
  {
    return SPINLEFT;                                                      // we will spin 90 degrees left
  }
  else if (linePos > 0 && linePos < lCutoff)                              // otherwise, if the line position is enough to the left...
  {
    return LINELEFT;                                                      // we will say the line is to our left
  }
  else if (linePos > rCutoff)                                             // otherwise, if the line position is enough to the right...
  {
    return LINERIGHT;                                                     // we will say the line is to our right
  }
  else                                                                    // otherwise, the line position is just underneath the middle of the robot...
  {
    return ON;                                                            // we will say we're centered on the line
  }
}

/*
 * This function checks if 5 or more sensors see a line
 */
bool dropOff()
{
  checkPS2();
  readLineSensor(sensorVal);                                              // gets values from line sensor value array
  readCalLineSensor(sensorVal,                                            // gets values from calibrated line sensor value array
            sensorCalVal,
            sensorMinVal,
            sensorMaxVal,
            lineColor);
  int numTriggers = 0;                                                    // tracks the number of sensors which see a line
  for (int i = 0; i < numVals; i++)
  {
    if (sensorVal[i] < spinTrigger)                                       // if this sensor sees a line...
    {
      numTriggers++;                                                      // increment the number of sensors that see a line
    }
  }
  return (numTriggers == numVals);                                        // returns whether or not all sensors see the line
}

/*
 * This function resets the appropriate autonomous global variables
 */
void resetVars()
{
  checkPS2();
  numTimesLocked = 0;                                                     // reset numTimesLocked 
  resetLeftEncoderCnt();                                                  // set the right encoder pulses count back to zero
  resetRightEncoderCnt();                                                 // set the right encoder pulses count back to zero
}

/*
 * This function calibrates the line sensors while in PS2 state before we can switch it to AUTO state
 */
void calibrateLine() 
{
  if (!isCalibrationComplete)                                             // if we haven't already calibrated...
  {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);                    // we will go forward
    enableMotor(BOTH_MOTORS);                                             // go forward
    setMotorSpeed(BOTH_MOTORS, calibrateSpeed);                           // set the appropriate speed for getting calibration data
    for (int i = 0; i < calibrationValues; i++)                           // takes 100 readings
    {
      readLineSensor(sensorVal);                                          // reads line sensors
      setSensorMinMax(sensorVal, sensorMinVal, sensorMaxVal);             // sets appropriate minimum and maximum values
    }
    disableMotor(BOTH_MOTORS);                                            // disable motors
    isCalibrationComplete = true;                                         // say that we have calibrated the line sensors
  }                                                       
}

/*
 * This function merely spins 180 degrees
 */
void spin180(float amtInches)                                              
{
  checkPS2();
  rightWheel.directionBackward();                           // right wheel will spin backward
  leftWheel.directionForward();                             // left wheel will spin forward

  goDistance(amtInches);                                    // spin halfway around, 180 degrees

  delayMicroseconds(tenthSec * MS);                         // delay for 0.1 seconds
}

/*
 * This function merely spins the robot 90 degrees in the specified direction
 */
void spin90(int RL)                          
{
  checkPS2();
  if (RL == RIGHT)
  {
    rightWheel.directionBackward();                         // right wheel will spin backward
    leftWheel.directionForward();                           // left wheel will spin forward
  }
  else if (RL == LEFT)
  {
    rightWheel.directionForward();                          // right wheel will spin forward
    leftWheel.directionBackward();                          // left wheel will spin backward
  }

  goDistance(spinInches);                                   // spin a quarter of the way around

  delayMicroseconds(tenthSec * MS);                         // delay for 0.1 seconds
}

/*
 * This function determines how many encoder pulses correspond to a certain distance in inches
 */
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance)           // integer to calculate the distance the robot needs to spin, based on its physical qualities
{
  checkPS2();                                                                               // check the PS2 for a potential state change (the X button)
  float temp = (wheel_diam * PI) / cnt_per_rev;                                             // circumference calculations
  temp = distance / temp;                                                                   // number of ticks based on distance parameter
  return (int) temp;                                                                        // cast to integer since ticks are in whole numbers
}

/*
 * This function moves the robot a given distance.
 * The timeConstraint helps the robot exit if it gets stuck on the wall and keeps trying to go the correct distance.
 */
void goDistance(float goInches)                                                             
{
  checkPS2();
  uint16_t rCount = 0;                                                                      // total amount of encoder pulses received
  uint16_t lCount = 0;                                                                      // total amount of encoder pulses received
  
  uint16_t ticks = countForDistance(wheelDiameter, cntPerRevolution, goInches);             // amount of encoder pulses needed to achieve distance 

  resetLeftEncoderCnt();                                                                    // Set the right encoder pulses count back to zero
  resetRightEncoderCnt();                                                                   // Set the right encoder pulses count back to zero

  rightWheel.setRawSpeed(slowWheelSpeed);                                                   // set right motor speed
  leftWheel.setRawSpeed(slowWheelSpeed);                                                    // set left motor speed

  rightWheel.enableMotor();                                                                 // turns on right motor
  leftWheel.enableMotor();                                                                  // turns on left motor

  while (lCount < ticks - (halfSpeed / 12.0) || rCount < ticks - (halfSpeed / 12.0))        // drive motor until it has received ticks pulses from both wheels. Subtraction is done because the robot keeps going a little even after we disable motors
  {
    checkPS2();                                                                             // check the PS2 for a potential state change (the X button)
    if (STATE == PS2)                                                                       // if we did change states...
    {
      break;                                                                                // force this function to stop so we can switch to PS2Mode()
    }
    lCount = getEncoderLeftCnt();                                                           // get the left encoder pulses count, set to lCount variable
    rCount = getEncoderRightCnt();                                                          // get the right encoder pulses count, set to rCount variable

    if (lCount < rCount)                                                                    // if there are less left encoder pulses than right
    {
      stopMovement();                                                                       // stop all movement
      spinOne(LEFT, slowWheelSpeed);                                                        // turn slightly to the right
    }
    else if (lCount > rCount)                                                               // if there are more left encoder pulses than right
    {
      stopMovement();                                                                       // stop all movement
      spinOne(RIGHT, slowWheelSpeed);                                                       // turn slightly to the left
    }
    else                                                                                    // if there are an equal number of left and right encoder pulses
    {
      spinBoth(slowWheelSpeed, slowWheelSpeed);                                             // spin both wheels at the preestablished speed
    }
  }

  rightWheel.disableMotor();                                                                // turn off right motor
  leftWheel.disableMotor();                                                                 // turn off left motor
}

/**************************************/
/*****PS2******REMOTE*****CONTROLS*****/
/**************************************/

/*
   This function takes the appropriate action based on which buttons are pressed.
   It also uses helper functions to interpret the joystick positions since that is more complex.
*/
void PS2Mode()
{
  ps2x.read_gamepad();                                      // read the PS2 gamepad for button presses and joystick positions

  if (ps2x.ButtonPressed(PSB_L1))                           // if the L1 button is pressed...
  {
    closeGripperFull();                                     // close the gripper all the way
  }
  else if (ps2x.ButtonPressed(PSB_R1))                      // otherwise, if the L2 button is pressed...
  {
    openGripperFull();                                      // open the gripper all the way
  }
  else if (ps2x.ButtonPressed(PSB_R2))                      // otherwise, if the R2 button is pressed...
  {
    closeGripper();                                         // close the gripper decrementally
  }
  else if (ps2x.ButtonPressed(PSB_SQUARE))                  // otherwise, if the square button is pressed...
  {
    calibrateLine();                                        // calibrates the line sensor values, which is used in autonomous mode
  }
  else if (ps2x.ButtonPressed(PSB_CIRCLE))                  // otherwise, if the circle button is pressed...
  {
    stopMovement();                                         // stop all movement for the moment
    STATE = AUTO;                                           // toggle out of PS2 state to AUTO state
  }
  else if (ps2x.ButtonPressed(PSB_TRIANGLE))                // otherwise, if the triangle button is pressed...
  {
    stopMovement();                                         // stop all movement for the moment
    STATE = REMOTE;                                         // toggle to REMOTE state
  }
  PS2Move();                                                // calls the PS2Move() function which handles all movement aspects of the robot according to joystick positions
}

void PS2Move()
{
  // change the raw values from the RIGHT joystick into mapped and constrained values for the motors
  int rightXvalue = constrain(                              // handles the X direction of the right joystick
                              map(
                                  ps2x.Analog(PSS_RX), 
                                  minJoystick, maxJoystick, 
                                  -spinConversion, spinConversion), 
                              -spinConversion, spinConversion); 
  int rightYvalue = constrain(                              // handles the Y direction of the right joystick
                              map(ps2x.Analog(PSS_RY), 
                                  minJoystick, maxJoystick, 
                                  stopConversion, -stopConversion), 
                              -stopConversion, stopConversion); 

  // Changes the raw values from the LEFT joystick into mapped and constrained values for the motor
  int leftXvalue = constrain(                               // handles the X direction of the left joystick
                             map(ps2x.Analog(PSS_LX), 
                                 minJoystick, maxJoystick, 
                                 -turnConversion, turnConversion), 
                             -turnConversion, turnConversion); 
  int leftYvalue = constrain(                               // handles the Y direction of the left joystick
                             map(ps2x.Analog(PSS_LY), 
                                 minJoystick, maxJoystick, 
                                 forwardBackwardConversion, -forwardBackwardConversion), 
                             -forwardBackwardConversion, forwardBackwardConversion); 

  if (rightYvalue < -stopCutoff)                            // if the right joystick is significantly pressed downwards...
  {
    stopMovement();                                         // stop the robot's movement
  }
  else if (leftYvalue < -restingCutoff || 
           leftYvalue > restingCutoff)                      // otherwise, if the left joystick is pushed vertically (we use -5 and 5 to account for calibration error in the joysticks
  {
    PS2TranslationMovement(leftXvalue, leftYvalue);         // translate the robot according to the left joystick's X and Y position
  }
  else                                                      // otherwise, we know that the left joystick is in its resting position at the center
  {
    stopMovement();                                         // therefore, stop the robot's movement
    if (rightXvalue > restingCutoff || 
        rightXvalue < -restingCutoff)                       // if the right joystick is pushed horizontally (we use -5 and 5 to account for calibration error in the joysticks)
    {
      PS2Spin(rightXvalue);                                 // spin the robot accoding to the right joystick's X position
    }
  }
}

/***********************************/
/*****PS2******REMOTE*****ROBOT*****/
/***********************************/

/*
 * This function handles translational rover movement when in PS2 mode.
 * All joystick positions used have been mapped and constrained in PS2Move().
 * NOTE: this function treats the gripper as the front.
 * The RSLK library uses directionForward/Backward treating the gripper as the back.
 * There are only turning capabilities when the robot moves forward (treating the gripper as the front).
 */
void PS2TranslationMovement(int x, int y)
{
  if (y < -restingCutoff)                                   // if the Y position of the left joystick is less than -5 (if it is pressed downwards), we will reverse...
  {
    rightWheel.directionForward();                          // set the right wheel's direction as forward
    leftWheel.directionForward();                           // set the left wheel's direction as forward
    spinBoth(-y, -y);                                       // rotate both wheels at a speed determined by the absolute value of the Y position of the left joystick
  }
  else if (y > restingCutoff)                               // otherwise, if the Y position of the left joystick is greater than 5 (if it is pressed upwards)...
  {
    rightWheel.directionBackward();                         // set the right wheel's direction as backward
    leftWheel.directionBackward();                          // set the left wheel's direction as backward
    if (x > restingCutoff)                                  // if the X position of the left joystick is greater than 5 (if it is pressed to the right), we have to turn right, too...
    {
      /* 
       * Rotate both wheels at a BASE speed determined by the absolute value of the Y position of the left joystick.
       * If we are turning right, the left wheel's speed should be increased by the absolute value of the X position of the left joystick
       */
      spinBoth(y + x, y);                                   
    }
    else if (x < -restingCutoff)                            // otherwise, if the X position of the left joystick is less than -5 (if it is pressed to the left), we have to turn left...
    {
      /* 
       * Rotate both wheels at a BASE speed determined by the absolute value of the Y position of the left joystick.
       * If we are turning left, the right wheel's speed should be increased by the absolute value of the X position of the left joystick
       */
      spinBoth(y, y + (-x));
    }
    else                                                    // otherwise, we know we do not have to turn...
    {
      spinBoth(y, y);                                       // rotate both wheels at an equal speed determined by the absolute value of the Y position of the left joystick
    }
  }
}
/*
 * This function spins the robot while in PS2 mode.
 * All joystick positions used have been mapped and constrained in PS2Move().
 * NOTE: this function treats the gripper as the front.
 * The reversed right/leftWheel and directionForward/Backward from the RSLK library cancel each other out. 
 */
void PS2Spin(int val)
{
  if (val < -restingCutoff)                                 // if the X position of the right joystick is less than -5 (if it is pressed to the left), we have to spin left... 
  {
    rightWheel.directionForward();                          // to spin left, we would make the right wheel rotate forward
    leftWheel.directionBackward();                          // make the left wheel rotate backward
  }

  else if (val > restingCutoff)                             // otherwise, if the X position of the right joystick is greater than 5 (if it is pressed to the right), we have to spin right...
  {
    leftWheel.directionForward();                           // to spin right, we would make the left wheel rotate forward
    rightWheel.directionBackward();                         // to spin right, we would make the right wheel rotate backward
  }

  spinBoth(abs(val), abs(val));                             // rotate both wheels at a speed determined by the absolute value of the X position of the right joystick
}

/*******************************************/
/*****INFRARED******REMOTE*****CONTROLS*****/
/*******************************************/

/*
   This function toggles to the REMOTE state if we're in another state and press 0 on the infrared remote
*/
void activateIR()
{
  switch (IRresults.command)                                // check for the new command from the infrared remote
  {
    case IR_FIVE:                                           // if (and only if) 5 is pressed...
      stopMovement();
      STATE = REMOTE;                                       // toggle state to REMOTE
      break;                                                // go back

    default:                                                // default case...
      break;                                                // go back
  }
  delayMicroseconds(remoteDelay);                           // delay for the IR sensor
}

/*
   This function takes the appropriate action based on the IR code received
*/
void IRMode()
{
  switch (IRresults.command)                                // check for the new command from the infrared remote
  {
    case IR_UP:                                             // if up arrow is pressed...
      moveForward();                                        // move the robot forward
      break;                                                // go back

    case IR_DOWN:                                           // if down arrow is pressed...
      moveBackward();                                       // move the robot backward
      break;                                                // go back

    case IR_LEFT:                                           // if left arrow is pressed...
      spin(LEFT);                                           // spin the robot left
      break;                                                // go back

    case IR_RIGHT:                                          // if right arrow is pressed...
      spin(RIGHT);                                          // spin the robot right
      break;                                                // go back

    case IR_OK:                                             // if OK is pressed...
      stopMovement();                                       // stop the robot's movement
      break;                                                // go back

    case IR_ONE:                                            // if 1 is pressed...
      turn(LEFT);                                           // turn the robot left
      break;                                                // go back

    case IR_THREE:                                          // if 3 is pressed...
      turn(RIGHT);                                          // turn the robot right
      break;                                                // go back

    case IR_EIGHT:                                          // if 8 is pressed...
      stopMovement();                                       // stop all movement for the moment
      STATE = AUTO;                                         // toggle to the automatic state
      break;                                                // go back

    case IR_STAR:                                           // if * is pressed...
      openGripper();                                        // open the gripper
      break;                                                // go back

    case IR_HASH:                                           // if # is pressed...
      closeGripper();                                       // close the gripper
      break;                                                // go back

    case IR_ZERO:                                           // if 0 is pressed...
      stopMovement();                                       // stop all movement for the moment
      STATE = PS2;                                          // toggle to PS2 state
      break;                                                // go back

    default:                                                // default case...
      break;                                                // go back
  }
}

/****************************************/
/*****INFRARED******REMOTE*****ROBOT*****/
/****************************************/

/*
 * This function spins the wheels forward for a specified duration in seconds.
 * NOTE: this function treats the gripper as the front.
 * The RSLK library uses directionBackward treating the gripper as the back.
 */
void moveForward()
{
  rightWheel.directionBackward();           // set rightWheel's direction to "backward"
  leftWheel.directionBackward();            // set leftWheel's direction to "backward"

  spinBoth(fullSpeed, fullSpeed);           // rotate both wheels at full speed
}

/*
   This function spins the wheels backward for a specified duration in seconds.
   NOTE: this function treats the gripper as the front.
   The RSLK library uses directionForward treating the gripper as the back.
*/
void moveBackward()
{
  rightWheel.directionForward();            // set rightWheel's direction to "forward"
  leftWheel.directionForward();             // set leftWheel's direction to "forward"

  spinBoth(fullSpeed, fullSpeed);           // rotate both wheels at full speed
}

/*
 * This function spins the robot in a specified direction.
 * NOTE: this function treats the gripper as the front.
 * The reversed right/leftWheel and directionForward/Backward from the RSLK library cancel each other out.
 */
void spin(int RL)
{
  if (RL == RIGHT)                          // if we are told to spin right...
  {
    rightWheel.directionBackward();         // to spin right, we would make the right wheel rotate backward
    leftWheel.directionForward();           // make the left wheel rotate forward
  }
  else if (RL == LEFT)                      // if we are told to spin right...
  {
    rightWheel.directionForward();          // to spin left, we would make the right wheel rotate forward
    leftWheel.directionBackward();          // make the left wheel rotate backward
  }
  spinBoth(halfSpeed, halfSpeed);           // rotate both wheels at half speed
}

/*
   This function turns the robot in a specified direction.
   NOTE: this function treats the gripper as the front.
   The RSLK library function directionBackward() will make the robot move with the gripper in front.
*/
void turn(int RL)
{
  if (RL == RIGHT)                          // if we are told to turn right...
  {
    rightWheel.directionBackward();         // set the direction to "backward" (due to nature of the RSLK library)
    leftWheel.directionBackward();          // set the direction to "backward"
    spinBoth(fullSpeed, halfSpeed);         // we would turn the left wheel (treating the gripper as the front) faster to turn the robot right
  }
  else if (RL == LEFT)                      // if we are told to turn right...
  {
    rightWheel.directionBackward();         // set the direction to "backward"
    leftWheel.directionBackward();          // set the direction to "backward"
    spinBoth(halfSpeed, fullSpeed);         // turn the right wheel (treating the gripper as the front) faster to turn the robot left
  }
}

/*******************************************/
/***********UNIVERSAL***FUNCTIONS***********/
/*******************************************/

void spinBoth(int rSpeed, int lSpeed)       // This function spins both wheels at specified speeds.   
{
  rightWheel.setRawSpeed(rSpeed);           // set the rightWheel's raw rotational speed to rValue
  leftWheel.setRawSpeed(lSpeed);            // set the leftWheel's raw rotational speed to lValue

  rightWheel.enableMotor();                 // start rotating rightWheel at the given speed
  leftWheel.enableMotor();                  // start rotating leftWheel at the given speed
}

void spinOne(int RL, int speed)             // spins the robot very slightly in either direction, based on what is given
{
  if (RL == RIGHT)                          // if you want to spin to the right
  {
    rightWheel.setRawSpeed(speed);          // set the right wheel to the given speed
    leftWheel.disableMotor();               // turn off the left motor
    rightWheel.enableMotor();               // turn on the right motor
  }
  else if (RL == LEFT)                      // if you want to spin to the left
  {
    leftWheel.setRawSpeed(speed);           // set the left wheel to the given speed
    rightWheel.disableMotor();              // turn off the right motor
    leftWheel.enableMotor();                // turn on the left motor
  }
}

void stopMovement()                         // function to stop all movement    
{
  rightWheel.disableMotor();                // disable the right motor
  leftWheel.disableMotor();                 // disable the left motor
}

void openGripperFull()                      // function to open the gripper all the way
{
  ps2x.read_gamepad();                      // checks if a button is pressed on the PS2 controller
  gripper.write(openGripperAngle);          // writes the maximum angle to the gripper servo
  gripperAngle = openGripperAngle;          // sets the gripperAngle variable to this new value
}

void closeGripperFull()                     // function to close the gripper all the way
{
  ps2x.read_gamepad();                      // check if a button is pressed on the PS2 controller
  gripper.write(closedGripperAngle);        // writes the minimum angle to the servo gripper
  gripperAngle = closedGripperAngle;        // sets the gripperAngle variable to this new value
}

void openGripper()                          // function to incrementally open the gripper
{
  ps2x.read_gamepad();                      // checks if a button is pressed on the PS2 controller
  if (gripperAngle - gripperChange <        // if the gripper would be opened beyond its maximum angle
      openGripperAngle)
  {
    gripper.write(openGripperAngle);        // set the gripper to its maximum angle
    gripperAngle = openGripperAngle;
  }
  else                                      // if the gripper would not be opened beyond its maximum angle
  {
    gripper.write(gripperAngle -            // increase the gripper's angle by a predetermined amount
                  gripperChange);           
    gripperAngle -= gripperChange;          // write the new angle to the gripperAngle variable
  }
}

void closeGripper()                         // function to incrementally close the gripper
{
  ps2x.read_gamepad();                      // checks if a button is pressed on the PS2 controller
  if (gripperAngle + gripperChange >        // if the gripper would be closed beyond its minimum angle
      closedGripperAngle)                    
  {
    gripper.write(closedGripperAngle);      // set the gripper to its minimum angle
    gripperAngle = closedGripperAngle;
  }
  else                                      // if the gripper would not be closed beyond its minimum angle
  {
    gripper.write(gripperAngle +            // decrease the gripper's angle by a predetermined amount
                  gripperChange);          
    gripperAngle += gripperChange;          // write the new angle to the gripperAngle variable
  }
}
