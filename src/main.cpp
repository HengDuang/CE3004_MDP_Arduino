#include <Arduino.h>
#include "DualVNH5019MotorShield.h"
#include <EnableInterrupt.h>

DualVNH5019MotorShield md;
//Kp = proportional gain
//Ki = integral gain
//Kd = derivative gain

///////////////////////////////////////////////////////////////////////////////////////////////////////////         CONSTANTS VALUE FOR PID CONSTANT VALUES     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//max range -> 6.65 v

////Left //red line

//6.65 - 6.45v //overload
// double Kp = 11.5; //10 increase to reduce spike at the start
// double Ki = 0.55; //0.9 increase to pump up to able to reach desired speed
// double Kd = 0.35; //increase to improve stablity and prevent overshoot from the target

// //Right //blue line
// double Kpr = 10.5;
// double Kir = 0.62;
// double Kdr = 0.35;
// < 6.4
//ideael 6.35v ~ 6.2 (optimal)  , < 6.2 not optimal
// double Kp = 10.5; //10 increase to reduce spike at the start
// double Ki = 0.52; //0.9 increase to pump up to able to reach desired speed
// double Kd = 0.35; //increase to improve stablity and prevent overshoot from the target

// //Right //blue line
// double Kpr = 9.5;
// double Kir = 0.55;
// double Kdr = 0.35;

double Kp = 5; //10 increase to reduce spike at the start
 double Ki = 0.35; //0.9 increase to pump up to able to reach desired speed
 double Kd = 0.3; //increase to improve stablity and prevent overshoot from the target

 //Right //blue line
 double Kpr = 5;
 double Kir = 0.6;
 double Kdr = 0.45;


//wheel at 5.2 cm away from left sensor
///////////////////////////////////////////////////////////////////////////////////////////////////////////        PID DECLARATIONS      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
double TargetRPM = 0;
double right_motorerror;
double new_right_motor_RPM;
double left_motorerror;
double new_left_motorRPM;
double left_previous_motorerror;
double right_previous_motorerror;
double all_left_motorerror;
double all_right_motorerror;

///////////////////////////////////////////////////////////////////////////////////////////////////////////         RPM COUNTER DECLARATION     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Switch Cases for Robot Movement
char directions;
int choices;

///////////////////////////////////////////////////////////////////////////////////////////////////////////       TIMER FOR RPM  DECLARATION      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long RPMTimerMR = 0;
unsigned long StartRPMTimerMR = 0;
unsigned long EndRPMTimerMR = 0;

unsigned long RPMTimerML = 0;
unsigned long StartRPMTimerML = 0;
unsigned long EndRPMTimerML = 0;

double mrRPM = 0;
double mlRPM = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////         PIN FOR ENCODER DECLARATION      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Pin for Encoder
int rightEncoder = 3; //Right Wheel
int leftEncoder = 11; //Left Wheel

///////////////////////////////////////////////////////////////////////////////////////////////////////////         ENCODER COUNTER FOR PID CONTROL      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Encoder Counter that will be use for PID control later
volatile int right_motor_counter = 0; //Counter Value for number of High pulse for Right Motor
volatile int left_motor_counter = 0;  //Counter Value for number of High pulse for left Motor

///////////////////////////////////////////////////////////////////////////////////////////////////////////         ROBOT MOVEMENT DECLARATION      ///////////////////////////////////////////////////////////////////////////////////////////////////////////

//Movement for Robot
volatile double turning_Counter_R = 0;
volatile double turning_Counter_L = 0;
// double CalibrateTurn = 0;
volatile double grid_Counter = 0;
// double TotalDistanceTravelled = 0;
volatile boolean distance_Stop = false;
volatile boolean angle_R = false;
volatile boolean angle_L = false;
// int count = 0;

//Robot Sensors
double front_left_sensor_gradient = 0.00009433724900356394;
double front_left_sensor_constant = 0.01032998424387418;

double front_right_sensor_gradient = 0.00009474947099698279;
double front_right_sensor_constant = 0.008744034252092175;

double front_center_sensor_gradient = 0.00009463778522375842;
double front_center_sensor_constant = 0.009640479002695677;

double left_short_sensor_R_Gradient = 0.00008886449832183208;
double left_short_sensor_R_constant = 0.01077191214530613;

double right_long_sensor_gradient = 0.0000636965275884001;
double right_long_sensor_constant = 0.0004925657171665529;

double left_short_sensor_gradient = 0.00009553611726362321;
double left_short_Sensor_Constant = 0.007772562954454363;

int forward_L;
int forward_R;
int forward_C;
// int Distance;
int LeftShortSensorL;
int LeftShortSensorR;
int RightLongSensor;
double LeftSensorLCal;
double LeftSensorRcal;
double FrontSenSorLCal;
double FrontSensorRCal;
double FrontSenSorCCal;

enum StateModeFP
{
  Stop,
  stop_SendSensorData,
  ForwardStop,
  movement_Backwards,
  movement_Turn_left,
  turnLeft_1,
  movement_Turn_right,
  turnRight_1,
  movement_Forward,
  movement_One_grid,
  half_calibrate,
  full_Calibrate,
  calibrate_FrontWall,
  CheckFullCalibrationEP,
  CalibrateFrontLeftMiddle,
  CalibrateFrontLeftRight,
  CalibrateFrontRightMiddle,
  
  empty_function
};
StateModeFP State = Stop;

enum ModeStateCal
{
  turn_Left_cal1,
  turn_right_cal,
  turn_right_Cal1,
  calibrate_Sidecal,
  calibrate_Full_cal,
  calibrate_Frontcal,
  calibrate_Frontwall_cal1,
  stop_Dontsendsensorcal
};
ModeStateCal state_Cal = stop_Dontsendsensorcal;
//PID computation
void compute_PID();
void compute_MotorError();
void compute_PreviousMotorError();
void compute_AllPreviousMotorError();

//Robot movements
void moveRobot_Forward();
void moveRobot_ClockWise();
void moveRobot_AntiClockWise();
void moveRobot_Backward();
void robotStop_BothMotors();
void robotStop_LeftMotor(); // for left turn
void robotStop_RightMotor(); // for right turn
void robotStop_OneGrid();

//Sensor Calibrations
double calibrate_FrontLeftShortSensor();
double calibrate_FrontRightShortSensor();
double calibrate_LeftShortSensorR();
double calibrate_RightLongSensor();
double Calibrate_LeftShortSensor();

//Raw sensor Values
double sideSensor_LCal();
double sideSensor_RCal();
double frontSensor_LCal();
double frontSensor_RCal();
double frontSensor_CCal();


//RPM computation
void leftMotorCount();
void rightMotorCount();
double calculateRPM(double encoderValue);

void get_sensorData();
void SendSensorData();

// int DistanceTracker;

// boolean OneGridCheck;
char movements;
int numberOfGrid;
boolean Main = false;
boolean Calibration = false;
// boolean Step2Check = false;
boolean move_forward = false;
// boolean gridtracker = false;
// boolean Leftwallexist = false;
// boolean rightwallexist = false;


int dist[15] = {0, 23, 44, 66, 88, 125, 150, 175, 200, 225};
int grid = 0;
int count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////         VOID SETUP     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  enableInterrupt(rightEncoder, rightMotorCount, RISING); //Set Rising Edge as interrupt
  enableInterrupt(leftEncoder, leftMotorCount, RISING);   //Set Rising Edge as interrupt
  md.init();
  sei();
}
////////////////////////////////////////////////////////////////////////////// Main Function ////////////////////////////////////////////////////////////////////////////////
void loop()
{
  

  // delay(10);
  // return;
  //5.2 cm from left sensor , 13 cm from back
  // get_sensorData();
  // SendSensorData();
  if (Serial.available() > 0)
  {
    char Modes = Serial.read();
    switch (Modes)
    {
    case 'i':
      Main = true;
      Serial.println("Bot_Start");
      SendSensorData();
      break;

    case 'z':
      Serial.println("Bot_calibrating");
      state_Cal = calibrate_Full_cal;
      break;
    }
  }
  switch (state_Cal)
  {
  case calibrate_Full_cal:
  {
    TargetRPM = 80;
    moveRobot_AntiClockWise();
    if (angle_L == true)
    {
      right_motor_counter = 0;
      left_motor_counter = 0;
      left_motorerror = 0;
      right_motorerror = 0;
      turning_Counter_L = 0;
      turning_Counter_R = 0;
      right_previous_motorerror = right_motorerror;
      left_previous_motorerror = left_motorerror;
      new_right_motor_RPM = 0;
      new_left_motorRPM = 0;
      angle_L = false;
      angle_R = false;
      robotStop_LeftMotor();
      delay(500);
      state_Cal = turn_Left_cal1;
    }
  }
  break;

  case turn_Left_cal1:
  {
    TargetRPM = 80;
    moveRobot_AntiClockWise();
    if (angle_L == true)
    {
      right_motor_counter = 0;
      left_motor_counter = 0;
      left_motorerror = 0;
      right_previous_motorerror = right_motorerror;
      left_previous_motorerror = left_motorerror;
      new_right_motor_RPM = 0;
      new_left_motorRPM = 0;
      right_motorerror = 0;
      turning_Counter_L = 0;
      turning_Counter_R = 0;
      angle_L = false;
      angle_R = false;
      robotStop_LeftMotor();
      delay(500);
      state_Cal = calibrate_Frontwall_cal1;
    }
  }
  break;

  case calibrate_Frontwall_cal1:
  {
    get_sensorData();
    while (FrontSenSorLCal < 8.7 && FrontSensorRCal < 9.5)
    {
      get_sensorData();
      md.setSpeeds(70, 70); // difference of 11
    }

    while (FrontSensorRCal - FrontSenSorLCal <= 1.2 || FrontSensorRCal - FrontSenSorLCal >= 1.3)
    {
      get_sensorData();
      if (FrontSensorRCal - FrontSenSorLCal <= 1.2)
      {
        md.setSpeeds(-80, 80);
      }
      else if (FrontSensorRCal - FrontSenSorLCal >= 1.3)
      {
        md.setSpeeds(80, -80);
      }
    }

    while (FrontSenSorLCal > 8.8 && FrontSensorRCal > 9.6)
    {
      get_sensorData();
      md.setSpeeds(-70, -80); // difference of 11
    }

    robotStop_BothMotors();
    delay(500);
    state_Cal = turn_right_cal;
  }
  break;

  case turn_right_cal:
  {
    TargetRPM = 100;
    moveRobot_ClockWise();
    if (angle_R == true)
    {
      right_motor_counter = 0;
      left_motor_counter = 0;
      left_motorerror = 0;
      right_motorerror = 0;
      turning_Counter_R = 0;
      right_previous_motorerror = right_motorerror;
      left_previous_motorerror = left_motorerror;
      new_right_motor_RPM = 0;
      new_left_motorRPM = 0;
      angle_R = false;
      // Step2Check = false;
      robotStop_LeftMotor();
      delay(500);
      state_Cal = calibrate_Frontcal;
    }
  }
  break;

  case calibrate_Frontcal:
  {
    get_sensorData();
    while (FrontSenSorLCal < 8.9 && FrontSensorRCal < 9.6)
    {
      get_sensorData();
      md.setSpeeds(70, 80);
    }

    while ((FrontSensorRCal - FrontSenSorLCal <= 0.7) || (FrontSensorRCal - FrontSenSorLCal >= 0.8))
    {
      get_sensorData();
      if (FrontSensorRCal - FrontSenSorLCal <= 0.7)
      {
        md.setSpeeds(-80, 80);
      }
      else if (FrontSensorRCal - FrontSenSorLCal >= 0.8)
      {
        md.setSpeeds(80, -80);
      }
    }

    while (FrontSenSorLCal > 9 && FrontSensorRCal > 9.7)
    {
      get_sensorData();
      md.setSpeeds(-70, -80);
    }

    robotStop_BothMotors();
    delay(500);
    state_Cal = turn_right_Cal1;
  }
  break;

  case turn_right_Cal1:
  {
    TargetRPM = 100;
    moveRobot_ClockWise();
    if (angle_R == true)
    {
      right_motor_counter = 0;
      left_motor_counter = 0;
      left_motorerror = 0;
      right_motorerror = 0;
      turning_Counter_R = 0;
      grid_Counter =0;
      right_previous_motorerror = right_motorerror;
      left_previous_motorerror = left_motorerror;
      new_right_motor_RPM = 0;
      new_left_motorRPM = 0;
      angle_R = false;
      // Step2Check = false;
      robotStop_LeftMotor();
      delay(500);
      state_Cal = calibrate_Sidecal;
    }
  }
  break;

  case calibrate_Sidecal:
  {
    get_sensorData();
    while ((LeftSensorRcal - LeftSensorLCal <= 0.81) || (LeftSensorRcal - LeftSensorLCal >= 0.83))
    {
      get_sensorData();
      if (LeftSensorRcal - LeftSensorLCal >= 0.81)
      {
        md.setSpeeds(65, -65);
      }
      if (LeftSensorRcal - LeftSensorLCal <= 0.83)
      {
        md.setSpeeds(-65, 65);
      }
    }
    robotStop_BothMotors();
    delay(500);
    state_Cal = stop_Dontsendsensorcal;
  }
  break;

  case stop_Dontsendsensorcal:
  {
    right_motor_counter = 0;
    left_motor_counter = 0;
    right_motorerror = 0;
    left_motorerror = 0;
    turning_Counter_R = 0;
    turning_Counter_L = 0;
    grid_Counter = 0;
    angle_L = false;
    angle_R = false;
    robotStop_BothMotors();
    state_Cal = stop_Dontsendsensorcal;
    break;
  }
}
  while (Main)
  {
    //  Serial.print(LeftSensorLCal);
    //   Serial.print("  ");
    //   Serial.println(LeftSensorRcal);
   
    get_sensorData();
    // SendSensorData();
    if (Serial.available() > 0)
    {
      if (Serial.peek() >= 97 && Serial.peek() < 123)
      { // For normal commands
        movements = Serial.read();
      }
      if (Serial.peek() >= 48 && Serial.peek() < 58)
      { // For 0-9 , append char 'A' and int will be the Gridvalue
        movements = 'f';
        numberOfGrid = Serial.parseInt();
      }
      switch (movements)
      {
      case 'f':
        State = movement_One_grid;
        break;

      case 'w':
        State = movement_Forward;
        break;

      case 'a':
        State = movement_Turn_left;
        break;

      case 's':
        State = Stop;
        break;

      case 'd':
        State = movement_Turn_right;
        break;

      case 'y':
        State = movement_Backwards;
        break;

      case 'v':
        State = full_Calibrate;
        // Step2Check = false;
        break;

      case 'c':
        State = half_calibrate;
        break;

      case 'b':
        State = CalibrateFrontLeftMiddle;
        break;

      case 'n':
        State = CalibrateFrontLeftRight;
        break;

      case 'm':
        State = CalibrateFrontRightMiddle;
        break;
      }
    }
    switch (State)
    {
    case Stop:
    {
      right_motor_counter = 0;
      left_motor_counter = 0;
      right_motorerror = 0;
      left_motorerror = 0;
      turning_Counter_R = 0;
      turning_Counter_L = 0;
      grid_Counter = 0;
      angle_L = false;
      angle_R = false;
      State = Stop;
      robotStop_BothMotors();
    }
    break;
////////////////////////////////////////////        Basic movements     ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    case movement_Forward:
    {
      TargetRPM = 100;
      
      moveRobot_Forward();
      if (forward_L == 1 || forward_R == 1 || forward_C == 1)
      {
        robotStop_BothMotors();
        State = stop_SendSensorData;
      }
    }
    break;

    case movement_One_grid:
    {
      // Serial.println("test1");
      if (move_forward == false) // to reset all counter values in the interrupt
      {
        right_motor_counter = 0;
        left_motor_counter = 0;
        right_motorerror = 0;
        left_motorerror = 0;
        grid_Counter = 0;
        distance_Stop = false;
        move_forward = true;
        right_previous_motorerror = right_motorerror;
        left_previous_motorerror = left_motorerror;
        new_right_motor_RPM = 0;
        new_left_motorRPM = 0;
        grid = dist[numberOfGrid]; // 0-9
      }
      get_sensorData();
      if (grid == 23) // set speed for one grid movements
      {
        // md.setSpeeds(-362, -382);
        md.setSpeeds(-362,-382);
        // md.setSpeeds(-370,-400);
      }
      else if (grid != 23) // set speed for fp
      {
        TargetRPM = 100;
      }
      // moveRobot_Forward();
      if (distance_Stop == true) // if grid number hit > distancestop = true , stop motor movement
      {
        distance_Stop = false;
        // Serial.println("test");
        right_motor_counter = 0;
        left_motor_counter = 0;
        right_motorerror = 0;
        left_motorerror = 0;
        right_previous_motorerror = right_motorerror;
        left_previous_motorerror = left_motorerror;
        new_right_motor_RPM = 0;
        new_left_motorRPM = 0;
        robotStop_BothMotors();
        move_forward = false;
        if (grid == 23)
        {
          delay(100);
          State = CheckFullCalibrationEP; //exploration

          // State = Stop; // fastest path
        }
        if (grid != 23) //if setting is not set to one grid only do one instruction
        {
          delay(100);
          State = stop_SendSensorData;
        }
      }
    }
    break;

    case movement_Turn_left:
    {
      TargetRPM = 80;
      moveRobot_AntiClockWise();
      if (angle_L == true)
      {
        robotStop_LeftMotor();
        right_motor_counter = 0;
        left_motor_counter = 0;
        right_motorerror = 0;
        left_motorerror = 0;
        turning_Counter_L = 0;
        right_previous_motorerror = right_motorerror;
        left_previous_motorerror = left_motorerror;
        new_right_motor_RPM = 0;
        new_left_motorRPM = 0;
        all_right_motorerror =0;
        all_left_motorerror =0;
        angle_L = false;
        robotStop_LeftMotor();
        delay(100);
        State = CheckFullCalibrationEP;
      }
    }
    break;

    case movement_Turn_right:
    {
      TargetRPM = 80;
      moveRobot_ClockWise();
      // md.setSpeeds(-400,400);
      if (angle_R == true)
      {
        robotStop_RightMotor();
        right_motor_counter = 0;
        left_motor_counter = 0;
        right_motorerror = 0;
        left_motorerror = 0;
        turning_Counter_R = 0;
        right_previous_motorerror = right_motorerror;
        left_previous_motorerror = left_motorerror;
        new_right_motor_RPM = 0;
        new_left_motorRPM = 0;
        all_right_motorerror = 0;
        all_left_motorerror = 0;
        robotStop_RightMotor();
        angle_R = false;
        delay(100);
        State = CheckFullCalibrationEP;
      }
    }
    break;

    case empty_function:
    {
      right_motor_counter = 0;
      left_motor_counter = 0;
      right_motorerror = 0;
      left_motorerror = 0;
      turning_Counter_R = 0;
      turning_Counter_L = 0;
      robotStop_LeftMotor();
      angle_R = false;
      angle_L = false;
      grid_Counter = 0;
      State = stop_SendSensorData;
    }
    break;

    case movement_Backwards:
    {
      md.setSpeeds(80, 80);
      count++;
      if (count == 70)
      {
        count = 0;
        md.setBrakes(400, 400);
        robotStop_BothMotors();
        State = stop_SendSensorData;
      }
    }
    break;

    case full_Calibrate:
    {
      get_sensorData();
      double offset = FrontSenSorLCal - FrontSensorRCal;

      while (FrontSenSorLCal > 10 && FrontSensorRCal > 10)
      {
        get_sensorData();
        md.setSpeeds(-70, -70); // difference of 11
      }

      while (FrontSenSorLCal < 9 && FrontSensorRCal < 9)
      {
        get_sensorData();
        md.setSpeeds(70, 70);
      }

      // Serial.println(abs(offset));
      while (  (FrontSensorRCal-FrontSenSorLCal >= 0.70) ||   ((FrontSensorRCal - FrontSenSorLCal) <= 0.55))
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSensorRCal - FrontSenSorLCal) >= 0.70)
        {
          md.setSpeeds(70, -70); // Turn Left
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) <= 0.55)
        {
          md.setSpeeds(-70, 70); // Turn Right
        }
      }

      while (FrontSenSorLCal > 10 && FrontSensorRCal > 10)
      {
        get_sensorData();
        md.setSpeeds(-70, -70); // difference of 11
      }

      while (FrontSenSorLCal < 9 && FrontSensorRCal < 9)
      {
        get_sensorData();
        md.setSpeeds(70, 70);
      }

      while (abs(offset) >= 0.75)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSenSorLCal - FrontSensorRCal) >= 0.2)
        {
          md.setSpeeds(80, -80); // Turn Left
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) >= 0.2)
        {
          md.setSpeeds(-80, 80); // Turn Right
        }
      }
      robotStop_BothMotors();
      State = turnLeft_1;
    }
    break;

    case turnLeft_1:
    {
      TargetRPM = 80;
      moveRobot_AntiClockWise();
      if (angle_L == true)
      {
        robotStop_RightMotor();
        right_motor_counter = 0;
        left_motor_counter = 0;
        right_motorerror = 0;
        left_motorerror = 0;
        turning_Counter_R = 0;
        right_previous_motorerror = right_motorerror;
        left_previous_motorerror = left_motorerror;
        new_right_motor_RPM = 0;
        new_left_motorRPM = 0;
        all_right_motorerror =0;
        all_left_motorerror =0;
        angle_L = false;
        robotStop_RightMotor();
        
        delay(100);
        // State = CheckFullCalibrationEP;
        State = calibrate_FrontWall;
      }
    }
    break;

    case calibrate_FrontWall:
    {
     get_sensorData();
      double offset = FrontSenSorLCal - FrontSensorRCal;

      while (FrontSenSorLCal > 10 && FrontSensorRCal > 10)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 9 && FrontSensorRCal < 9)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (  (FrontSensorRCal-FrontSenSorLCal >= 0.60) ||   ((FrontSensorRCal - FrontSenSorLCal) <= 0.30))
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSensorRCal - FrontSenSorLCal) >= 0.60)
        {
          md.setSpeeds(80, -80);; // Turn Left
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) <= 0.30)
        {
          md.setSpeeds(-80, 80); // Turn Right
        }
      }

      while (FrontSenSorLCal > 10 && FrontSensorRCal > 10)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 9 && FrontSensorRCal < 9)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      while ( (FrontSensorRCal-FrontSenSorLCal >= 0.60) ||   ((FrontSensorRCal - FrontSenSorLCal) <= 0.30) )
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSensorRCal - FrontSenSorLCal) >= 0.60)
        {
          md.setSpeeds(80, -80); // Turn Left
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) <= 0.30)
        {
          md.setSpeeds(-80, 80); // Turn Right
        }
      }
      robotStop_BothMotors();
      State = turnRight_1;
    }
    break;

    case turnRight_1:
    {
      // Serial.println(turning_Counter_R);
      TargetRPM = 80;
      moveRobot_ClockWise();
      if (angle_R == true)
      {
        robotStop_RightMotor();
        right_motor_counter = 0;
        left_motor_counter = 0;
        right_motorerror = 0;
        left_motorerror = 0;
        turning_Counter_R = 0;
        right_previous_motorerror = right_motorerror;
        left_previous_motorerror = left_motorerror;
        new_right_motor_RPM = 0;
        new_left_motorRPM = 0;
        all_right_motorerror =0;
        all_left_motorerror =0;
        robotStop_RightMotor();
        angle_R = false;
        delay(100);
        State = half_calibrate;
        // State = half_calibrate;
      }
    }
    break;

    case half_calibrate:
    {
     
      get_sensorData();
      while ((LeftSensorRcal - LeftSensorLCal <= 0.5) || (LeftSensorRcal - LeftSensorLCal >= 1.0))
      {
      get_sensorData();
      if (LeftSensorRcal - LeftSensorLCal >= 1.0)
      {
        md.setSpeeds(65, -65); // turn left
      }
      if (LeftSensorRcal - LeftSensorLCal <= 0.5)
      {
        md.setSpeeds(-65, 65); 
      }
    }
      robotStop_BothMotors();
      delay(50);
      State = stop_SendSensorData;
    }
    break;

    case stop_SendSensorData:
    {
      robotStop_OneGrid();
      SendSensorData();
      right_motor_counter = 0;
      left_motor_counter = 0;
      right_motorerror = 0;
      grid_Counter =0;
      left_motorerror = 0;
      turning_Counter_R = 0;
      turning_Counter_L = 0;
      State = Stop;
    }
    break;

    case CheckFullCalibrationEP: //
    {
      get_sensorData();
      if (LeftShortSensorL == 1 && LeftShortSensorR == 1)
      {
        if (LeftSensorLCal > 16 && LeftSensorRcal > 16)
        {
          State = turnLeft_1;
        }
        else if (LeftSensorLCal < 14 && LeftSensorRcal < 14)
        {
          State = turnLeft_1;
        }
        else
        {
          State = half_calibrate;
        }
      }
      else
      {
        State = empty_function;
      }
    }
      break;

    case CalibrateFrontLeftMiddle:
    {
      get_sensorData();
      double offset = FrontSenSorLCal - FrontSenSorCCal;

      while (FrontSenSorLCal > 13.1 && FrontSenSorCCal > 7.7)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 12.1 && FrontSenSorCCal < 6.7)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 4.85 || abs(offset) >= 4.9)
      {
        offset = FrontSenSorLCal - FrontSenSorCCal;
        get_sensorData();
        if ((FrontSenSorLCal - FrontSenSorCCal) >= 4.9)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSenSorCCal - FrontSenSorLCal) <= 4.85)
        {
          md.setSpeeds(80, -80);
        }
      }

      while (FrontSenSorLCal > 13.1 && FrontSenSorCCal > 7.7)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 12.1 && FrontSenSorCCal < 6.7)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) <= 5.01 || abs(offset) >= 5.06)
      {
        offset = FrontSenSorLCal - FrontSenSorCCal;
        get_sensorData();
        if ((FrontSenSorLCal - FrontSenSorCCal) >= 5.06)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSenSorCCal - FrontSenSorLCal) <= 5.01)
        {
          md.setSpeeds(80, -80);
        }
      }
      State = stop_SendSensorData;
    }
    break;

    case CalibrateFrontLeftRight:
    {
      get_sensorData();
      double offset = FrontSenSorLCal - FrontSensorRCal;

      while (FrontSenSorLCal > 12.8 && FrontSensorRCal > 13.1)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 11.8 && FrontSensorRCal < 12.1)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 0.4)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSenSorLCal - FrontSensorRCal) >= 0.7)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) >= 0.7)
        {
          md.setSpeeds(80, -80);
        }
      }

      while (FrontSenSorLCal > 11.8 && FrontSensorRCal > 12.1)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 10.8 && FrontSensorRCal < 11.1)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) >= 0.2)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSenSorLCal - FrontSensorRCal) >= 0.2)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) >= 0.2)
        {
          md.setSpeeds(80, -80);
        }
      }
      State = stop_SendSensorData;
    }
    break;

    case CalibrateFrontRightMiddle:
    {
      get_sensorData();
      double offset = FrontSenSorCCal - FrontSensorRCal;

      while (FrontSenSorCCal > 6.5 && FrontSensorRCal > 11.8)
      {
        get_sensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorCCal < 7 && FrontSensorRCal < 12)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 5.4 || abs(offset) >= 5.6)
      {
        offset = FrontSenSorCCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSensorRCal - FrontSenSorCCal) <= 5.4)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSensorRCal - FrontSenSorCCal) >= 5.6)
        {
          md.setSpeeds(80, -80);
        }
      }

      while (FrontSenSorCCal > 6.5 && FrontSensorRCal > 11.8)
      {
        get_sensorData();
        md.setSpeeds(-70, -80);
      }

      while (FrontSenSorCCal < 7 && FrontSensorRCal < 12)
      {
        get_sensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) <= 5.5 || abs(offset) >= 5.7)
      {
        offset = FrontSenSorCCal - FrontSensorRCal;
        get_sensorData();
        if ((FrontSensorRCal - FrontSenSorCCal) <= 5.5)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSensorRCal - FrontSenSorCCal) >= 5.7)
        {
          md.setSpeeds(80, -80);
        }
      }
      State = stop_SendSensorData;
    }
    break;

    case ForwardStop:
    {
      robotStop_OneGrid();
      SendSensorData();
      right_motor_counter = 0;
      left_motor_counter = 0;
      right_motorerror = 0;
      left_motorerror = 0;
      turning_Counter_R = 0;
      turning_Counter_L = 0;
      grid_Counter = 0;
      angle_L = false;
      angle_R = false;
      State = Stop;
    }
    break;
    }
    delay(10);
  }
}
////////////////////////////////////////////         COMPUTING PID VALUES      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void compute_PID()
{
  new_right_motor_RPM = (right_motorerror * Kpr) + (right_previous_motorerror * Kdr) + (all_right_motorerror * Kir);
  new_left_motorRPM = (left_motorerror * Kp) + (left_previous_motorerror * Kd) + (all_left_motorerror * Ki);
}
void compute_MotorError()
{
  right_motorerror = TargetRPM - mrRPM;
  left_motorerror = TargetRPM - mlRPM;
}
void compute_PreviousMotorError()
{

  right_previous_motorerror = right_motorerror;
  left_previous_motorerror = left_motorerror;
}
void compute_AllPreviousMotorError()
{
  all_left_motorerror += left_previous_motorerror;
  all_right_motorerror += right_previous_motorerror;
}
///////////////////////////////////////////         PID CONTROL FOR ROBOT MOVEMENT    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveRobot_Forward()
{
  //Compute MotorError
  compute_MotorError();

  //Compute PID
  compute_PID();

  //StoreMotorError
  compute_PreviousMotorError();

  //SumofAllPreviousMotorError
  compute_AllPreviousMotorError();
  md.setM2Speed(-new_right_motor_RPM);
  md.setM1Speed(-new_left_motorRPM);
  // Serial.print(mrRPM);
  // Serial.print(" ");
  // Serial.println(mlRPM);
}
void moveRobot_ClockWise()
{
  //Compute MotorError
  compute_MotorError();

  //Compute PID
  compute_PID();

  //StoreMotorError
  compute_PreviousMotorError();

  //SumofAllPreviousMotorError
  compute_AllPreviousMotorError();

  md.setM2Speed(new_right_motor_RPM);
  md.setM1Speed(-new_left_motorRPM);
  //  Serial.print(mrRPM);
  //  Serial.print(" ");
  //  Serial.println(mlRPM);
}
void moveRobot_AntiClockWise()
{
  compute_MotorError();
  compute_PID();
  compute_PreviousMotorError();
  compute_AllPreviousMotorError();
  md.setM2Speed(-1 * new_right_motor_RPM);
  md.setM1Speed(new_left_motorRPM);
}
void moveRobot_Backward()
{
  //Compute MotorError
  compute_MotorError();

  //Compute PID
  compute_PID();

  //StoreMotorError
  compute_PreviousMotorError();

  //SumofAllPreviousMotorError
  compute_AllPreviousMotorError();
  md.setM2Speed(new_right_motor_RPM);
  md.setM1Speed(new_left_motorRPM);
}
void robotStop_BothMotors()
{
  right_motor_counter = 0;
  left_motor_counter = 0;
  right_motorerror = 0;
  left_motorerror = 0;
  turning_Counter_L = 0;
  turning_Counter_R = 0;
  angle_L = false;
  angle_R = false;
  grid_Counter = 0;
  // DistanceTracker = 0;
  distance_Stop = false;
  md.setBrakes(370, 400);
}
void robotStop_LeftMotor() // for left turn
{
  right_motor_counter = 0;
  left_motor_counter = 0;
  right_motorerror = 0;
  left_motorerror = 0;
  turning_Counter_L = 0;
  turning_Counter_R = 0;
  angle_L = false;
  angle_R = false;
  grid_Counter = 0;
  distance_Stop = false;
  md.setBrakes(385, 400);
}
void robotStop_RightMotor() //for right turn
{
  right_motor_counter = 0;
  left_motor_counter = 0;
  right_motorerror = 0;
  left_motorerror = 0;
  turning_Counter_L = 0;
  turning_Counter_R = 0;
  angle_L = false;
  angle_R = false;
  grid_Counter = 0;
  distance_Stop = false;
  md.setBrakes(400, 400);
}
void robotStop_OneGrid()
{
  right_motor_counter = 0;
  left_motor_counter = 0;
  right_motorerror = 0;
  left_motorerror = 0;
  turning_Counter_L = 0;
  turning_Counter_R = 0;
  angle_L = false;
  angle_R = false;
  grid_Counter = 0;
  distance_Stop = false;
  // md.setBrakes(385, 355);
  // md.setBrakes(385,370);
  md.setBrakes(400, 390);
  // md.setBrakes(385,325);
}
//////////////////////////////////////////         Calibrate Sensor      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
double calibrate_FrontLeftShortSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / front_left_sensor_gradient;
    double q = front_left_sensor_constant / front_left_sensor_gradient;
    double _output = (M / (analogRead(A0) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 14)
  {
    return 1;
  }
  else if (Output <= 24)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double calibrate_FrontRightShortSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / front_right_sensor_gradient;
    double q = front_right_sensor_constant / front_right_sensor_gradient;
    double _output = (M / (analogRead(A1) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 15)
  {
    return 1;
  }
  else if (Output <= 25)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double calibrateFrontCenterShortSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / front_center_sensor_gradient;
    double q = front_center_sensor_constant / front_center_sensor_gradient;
    double _output = (M / (analogRead(A4) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 10)
  {
    return 1;
  }
  else if (Output <= 18)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double calibrate_LeftShortSensorR()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / left_short_sensor_gradient;
    double q = left_short_Sensor_Constant / left_short_sensor_gradient;
    double _output = (M / (analogRead(A3) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 21)
  {
    return 1;
  }
  else if (Output <= 30)
  {
    return 2;
  }
  return 0; //no object after 3 grids
  // return Output;
}
double calibrate_LeftShortSensorL()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / left_short_sensor_R_Gradient;
    double q = left_short_sensor_R_constant / left_short_sensor_R_Gradient;
    double _output = (M / (analogRead(A5) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 19)
  {
    return 1;
  }
  else if (Output <= 27)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double calibrate_RightLongSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / right_long_sensor_gradient;
    double q = right_long_sensor_constant / right_long_sensor_gradient;
    double _output = (M / (analogRead(A2) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 27)
  {
    return 3;
  }
  else if (Output <= 38)
  {
    return 4;
  }
  else if (Output <= 50)
  {
    return 5;
  }
  else
  {
    return 0;
  }
  // return Output;
}
///////////////////////////////////////////         RPM COUNTER      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void leftMotorCount()
{
  left_motor_counter++;
  if (left_motor_counter == 1)
  {
    StartRPMTimerML = micros();
  }
  else if (left_motor_counter == 11)
  {
     grid_Counter++;
    if (grid_Counter == grid)
    {
      distance_Stop = true;
      grid_Counter = 0;
      // Serial.println("one grid");
    }
    EndRPMTimerML = micros();
    RPMTimerML = (EndRPMTimerML - StartRPMTimerML) / 10.00;
    mlRPM = calculateRPM(RPMTimerML);
    left_motor_counter = 0;
  }
}
void rightMotorCount()
{
  // Serial.println(right_motor_counter);
  right_motor_counter++;
  if (right_motor_counter == 1)
  {
    StartRPMTimerMR = micros();
  }
  else if (right_motor_counter == 11)
  {
    turning_Counter_R++;
    turning_Counter_L++;
   
    // Serial.println(turning_Counter_R);
    if (turning_Counter_R == 36) // [36[+1},72[+2],108[+4],144[+4],180[+6],216[+6],252[+6],288[+7],324[+9],360[+10],396[+10],432[+11]// [90,180,270,360,450,540,600,720,810,900,990,1080]
    {
      angle_R = true;
      turning_Counter_R = 0;
    }
    if (turning_Counter_L == 36)
    {
      angle_L = true;
      turning_Counter_L = 0;
    }
    EndRPMTimerMR = micros();
    RPMTimerMR = (EndRPMTimerMR - StartRPMTimerMR) / 10.00;
    mrRPM = calculateRPM(RPMTimerMR);
    right_motor_counter = 0;
  }
}
double calculateRPM(double encoderValue)
{
  if (encoderValue == 0)
  {
    return 0;
  }
  else
  {
    return (60.00 / ((encoderValue * 562.25) / 1000000.00));
  }
}
///////////////////////////////////////////         Sending Sensor & getting Sensor Values      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void get_sensorData()
{
  forward_L = calibrate_FrontLeftShortSensor(); //-5
  forward_R = calibrate_FrontRightShortSensor();
  LeftShortSensorL = calibrate_LeftShortSensorL();
  RightLongSensor = calibrate_RightLongSensor();
  LeftShortSensorR = calibrate_LeftShortSensorR();
  forward_C = calibrateFrontCenterShortSensor();

  LeftSensorLCal = sideSensor_LCal();
  LeftSensorRcal = sideSensor_RCal();
  FrontSenSorLCal = frontSensor_LCal();
  FrontSensorRCal = frontSensor_RCal();
  FrontSenSorCCal = frontSensor_CCal();

  // Serial.println(FrontSenSorCCal);
  // Serial.print(LeftSensorLCal);
  // Serial.print("  ");
  // Serial.println(LeftSensorRcal);
}
void SendSensorData()
{
  forward_L = calibrate_FrontLeftShortSensor(); //-5
  forward_R = calibrate_FrontRightShortSensor();
  LeftShortSensorL = calibrate_LeftShortSensorL();
  RightLongSensor = calibrate_RightLongSensor();
  LeftShortSensorR = calibrate_LeftShortSensorR();
  forward_C = calibrateFrontCenterShortSensor();
  // Serial.println(forward_C);
  String s = "SDATA:" + String(forward_C) + ":" + String(forward_L) + ":" + String(forward_R) + ":" + String(LeftShortSensorL) + ":" + String(LeftShortSensorR) + ":" + String(RightLongSensor);
  Serial.println(s);
}
///////////////////////////////////////////         Raw Sensor      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
// double FrontSensorLeftCal()
// {
//   double M = 1 / front_left_sensor_gradient;
//   double q = front_left_sensor_constant / front_left_sensor_gradient;
//   double Output = (M / (analogRead(A0) + q)) - 8;

//   return Output;
// }
// double FrontSensorRightCal()
// {
//   double M = 1 / left_short_sensor_gradient;
//   double q = left_short_Sensor_Constant / left_short_sensor_gradient;
//   double Output = (M / (analogRead(A3) + q)) - 8;

//   return Output;
// }
// double FrontSensorCentreCal()
// {
//   double M = 1 / front_center_sensor_gradient;
//   double q = front_center_sensor_constant / front_center_sensor_gradient;
//   double Output = (M / (analogRead(A4) + q)) - 8;

//   return Output;
// }
double sideSensor_LCal()
{
  double M = 1 / left_short_sensor_R_Gradient;
  double q = left_short_sensor_R_constant / left_short_sensor_R_Gradient;
  double Output = (M / (analogRead(A5) + q)) - 8;
  return Output;
}
double sideSensor_RCal()
{
  double M = 1 / left_short_sensor_gradient;
  double q = left_short_Sensor_Constant / left_short_sensor_gradient;
  double Output = (M / (analogRead(A3) + q)) - 8;
  return Output;
}
double frontSensor_LCal()
{
  double M = 1 / front_left_sensor_gradient;
  double q = front_left_sensor_constant / front_left_sensor_gradient;
  double Output = (M / (analogRead(A0) + q)) - 8;

  return Output;
}
double frontSensor_RCal()
{
  double M = 1 / front_right_sensor_gradient;
  double q = front_right_sensor_constant / front_right_sensor_gradient;
  double Output = (M / (analogRead(A1) + q)) - 8;

  return Output;
}
double frontSensor_CCal()
{
  double M = 1 / front_center_sensor_gradient;
  double q = front_center_sensor_constant / front_center_sensor_gradient;
  double Output = (M / (analogRead(A4) + q)) - 8;

  return Output;
}