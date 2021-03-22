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
double RightMotorError;
double newRightMotorRPM;
double LeftMotorError;
double newLeftMotorRPM;
double LeftPreviousMotorError;
double RightPreviousMotorError;
double AllLeftMotorError;
double AllRightMotorError;

///////////////////////////////////////////////////////////////////////////////////////////////////////////         RPM COUNTER DECLARATION     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Switch Cases for Robot Movement
char Directions;
int Choices;

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
int RightEncoder = 3; //Right Wheel
int LeftEncoder = 11; //Left Wheel

///////////////////////////////////////////////////////////////////////////////////////////////////////////         ENCODER COUNTER FOR PID CONTROL      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Encoder Counter that will be use for PID control later
volatile int RightMotorCounter = 0; //Counter Value for number of High pulse for Right Motor
volatile int LeftMotorCounter = 0;  //Counter Value for number of High pulse for left Motor

///////////////////////////////////////////////////////////////////////////////////////////////////////////         ROBOT MOVEMENT DECLARATION      ///////////////////////////////////////////////////////////////////////////////////////////////////////////

//Movement for Robot
volatile double TurningCounter = 0;
volatile double TurningCounterL = 0;
// double CalibrateTurn = 0;
volatile double DistanceCounter = 0;
// double TotalDistanceTravelled = 0;
volatile boolean DistanceStop = false;
volatile boolean Angle = false;
volatile boolean AngleL = false;
// int count = 0;

//Robot Sensors
double FrontLeftSensorGradient = 0.00009433724900356394;
double FrontLeftSensorConstant = 0.01032998424387418;

double FrontRightSensorGradient = 0.00009474947099698279;
double FrontRightSensorConstant = 0.008744034252092175;

double FrontCenterSensorGradient = 0.00009463778522375842;
double FrontCenterSensorConstant = 0.009640479002695677;

double LeftShortSensorRGradient = 0.00008886449832183208;
double LeftShortSensorRConstant = 0.01077191214530613;

double RightLongSensorGradient = 0.0000636965275884001;
double RightLongSensorConstant = 0.0004925657171665529;

double LeftShortSensorGradient = 0.00009553611726362321;
double LeftShortSensorConstant = 0.007772562954454363;

int ForwardL;
int ForwardR;
int ForwardC;
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
  Stop_SendSensorData,
  ForwardStop,
  BackWard,
  TurnLeft,
  TurnLeft1,
  TurnRight,
  TurnRight1,
  Forward,
  ForwardGrid,
  Halfcalibrate,
  Fullcalibrate,
  CalibrateFrontEP,
  CheckFullCalibrationEP,
  CalibrateFrontLeftMiddle,
  CalibrateFrontLeftRight,
  CalibrateFrontRightMiddle,
  
  abcd
};
StateModeFP State = Stop;

enum ModeStateCal
{
  TurnLeftCal1,
  TurnRightCal,
  TurnRightCal1,
  CalibrateSideCal,
  FullCalibrateCal,
  CalibrateFrontCal,
  CalibrateFrontCal1,
  StopDontSendDataCal
};
ModeStateCal StateCal = StopDontSendDataCal;
//PID computation
void ComputePID();
void ComputeMotorError();
void ComputePreviousMotorError();
void ComputeAllPreviousMotorError();

//Robot Movements
void MoveRobot_Forward();
void MoveRobot_ClockWise();
void MoveRobot_AntiClockWise();
void MoveRobot_Backward();
void RobotStop_BothMotors();
void RobotStop_LeftMotor(); // for left turn
void RobotStop_RightMotor(); // for right turn
void RobotStop_OneGrid();

//Sensor Calibrations
double Calibrate_FrontLeftShortSensor();
double Calibrate_FrontRightShortSensor();
double Calibrate_LeftShortSensorR();
double Calibrate_RightLongSensor();
double Calibrate_LeftShortSensor();

//Raw sensor Values
double SideSensor_LCal();
double SideSensor_RCal();
double FrontSensor_LCal();
double FrontSensor_RCal();
double FrontSensor_CCal();


//RPM computation
void LeftMotorCount();
void RightMotorCount();
double calculateRPM(double encoderValue);

void GetSensorData();
void SendSensorData();

// int DistanceTracker;

// boolean OneGridCheck;
char Movements;
int NumberOfGrid;
boolean Main = false;
boolean Calibration = false;
// boolean Step2Check = false;
boolean MoveForward = false;
// boolean gridtracker = false;
// boolean Leftwallexist = false;
// boolean rightwallexist = false;


int Dist[15] = {0, 22, 44, 66, 88, 125, 150, 175, 200, 225};
int Grid = 0;
int count = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////         VOID SETUP     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  enableInterrupt(RightEncoder, RightMotorCount, RISING); //Set Rising Edge as interrupt
  enableInterrupt(LeftEncoder, LeftMotorCount, RISING);   //Set Rising Edge as interrupt
  md.init();
}
////////////////////////////////////////////////////////////////////////////// Main Function ////////////////////////////////////////////////////////////////////////////////
void loop()
{
  //5.2 cm from left sensor , 13 cm from back
  GetSensorData();
  // Serial.print(LeftSensorLCal);
  // Serial.print("  ");
  // Serial.println(LeftSensorRcal);
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
      StateCal = FullCalibrateCal;
      break;
    }
  }
  switch (StateCal)
  {
  case FullCalibrateCal:
  {
    TargetRPM = 100;
    MoveRobot_AntiClockWise();
    if (AngleL == true)
    {
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      LeftMotorError = 0;
      RightMotorError = 0;
      TurningCounterL = 0;
      TurningCounter = 0;
      RightPreviousMotorError = RightMotorError;
      LeftPreviousMotorError = LeftMotorError;
      newRightMotorRPM = 0;
      newLeftMotorRPM = 0;
      AngleL = false;
      Angle = false;
      RobotStop_LeftMotor();
      delay(500);
      StateCal = TurnLeftCal1;
    }
  }
  break;

  case TurnLeftCal1:
  {
    TargetRPM = 100;
    MoveRobot_AntiClockWise();
    if (AngleL == true)
    {
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      LeftMotorError = 0;
      RightPreviousMotorError = RightMotorError;
      LeftPreviousMotorError = LeftMotorError;
      newRightMotorRPM = 0;
      newLeftMotorRPM = 0;
      RightMotorError = 0;
      TurningCounterL = 0;
      TurningCounter = 0;
      AngleL = false;
      Angle = false;
      RobotStop_LeftMotor();
      delay(500);
      StateCal = CalibrateFrontCal1;
    }
  }
  break;

  case CalibrateFrontCal1:
  {
    GetSensorData();
    while (FrontSenSorLCal < 8.7 && FrontSensorRCal < 9.5)
    {
      GetSensorData();
      md.setSpeeds(70, 70); // difference of 11
    }

    while (FrontSensorRCal - FrontSenSorLCal <= 1.2 || FrontSensorRCal - FrontSenSorLCal >= 1.3)
    {
      GetSensorData();
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
      GetSensorData();
      md.setSpeeds(-70, -80); // difference of 11
    }

    RobotStop_BothMotors();
    delay(500);
    StateCal = TurnRightCal;
  }
  break;

  case TurnRightCal:
  {
    TargetRPM = 100;
    MoveRobot_ClockWise();
    if (Angle == true)
    {
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      LeftMotorError = 0;
      RightMotorError = 0;
      TurningCounter = 0;
      RightPreviousMotorError = RightMotorError;
      LeftPreviousMotorError = LeftMotorError;
      newRightMotorRPM = 0;
      newLeftMotorRPM = 0;
      Angle = false;
      // Step2Check = false;
      RobotStop_LeftMotor();
      delay(500);
      StateCal = CalibrateFrontCal;
    }
  }
  break;

  case CalibrateFrontCal:
  {
    GetSensorData();
    while (FrontSenSorLCal < 8.9 && FrontSensorRCal < 9.6)
    {
      GetSensorData();
      md.setSpeeds(70, 80);
    }

    while ((FrontSensorRCal - FrontSenSorLCal <= 0.7) || (FrontSensorRCal - FrontSenSorLCal >= 0.8))
    {
      GetSensorData();
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
      GetSensorData();
      md.setSpeeds(-70, -80);
    }

    RobotStop_BothMotors();
    delay(500);
    StateCal = TurnRightCal1;
  }
  break;

  case TurnRightCal1:
  {
    TargetRPM = 100;
    MoveRobot_ClockWise();
    if (Angle == true)
    {
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      LeftMotorError = 0;
      RightMotorError = 0;
      TurningCounter = 0;
      DistanceCounter =0;
      RightPreviousMotorError = RightMotorError;
      LeftPreviousMotorError = LeftMotorError;
      newRightMotorRPM = 0;
      newLeftMotorRPM = 0;
      Angle = false;
      // Step2Check = false;
      RobotStop_LeftMotor();
      delay(500);
      StateCal = CalibrateSideCal;
    }
  }
  break;

  case CalibrateSideCal:
  {
    GetSensorData();
    while ((LeftSensorRcal - LeftSensorLCal <= 0.81) || (LeftSensorRcal - LeftSensorLCal >= 0.83))
    {
      GetSensorData();
      if (LeftSensorRcal - LeftSensorLCal >= 0.81)
      {
        md.setSpeeds(65, -65);
      }
      if (LeftSensorRcal - LeftSensorLCal <= 0.83)
      {
        md.setSpeeds(-65, 65);
      }
    }
    RobotStop_BothMotors();
    delay(500);
    StateCal = StopDontSendDataCal;
  }
  break;

  case StopDontSendDataCal:
  {
    RightMotorCounter = 0;
    LeftMotorCounter = 0;
    RightMotorError = 0;
    LeftMotorError = 0;
    TurningCounter = 0;
    TurningCounterL = 0;
    DistanceCounter = 0;
    AngleL = false;
    Angle = false;
    RobotStop_BothMotors();
    StateCal = StopDontSendDataCal;
    break;
  }
}
  while (Main)
  {
    //  Serial.print(LeftSensorLCal);
    //   Serial.print("  ");
    //   Serial.println(LeftSensorRcal);
   
    GetSensorData();
    // SendSensorData();
    if (Serial.available() > 0)
    {
      if (Serial.peek() >= 97 && Serial.peek() < 123)
      { // For normal commands
        Movements = Serial.read();
      }
      if (Serial.peek() >= 48 && Serial.peek() < 58)
      { // For 0-9 , append char 'A' and int will be the Gridvalue
        Movements = 'f';
        NumberOfGrid = Serial.parseInt();
      }
      switch (Movements)
      {
      case 'f':
        State = ForwardGrid;
        break;

      case 'w':
        State = Forward;
        break;

      case 'a':
        State = TurnLeft;
        break;

      case 's':
        State = Stop;
        break;

      case 'd':
        State = TurnRight;
        break;

      case 'y':
        State = BackWard;
        break;

      case 'v':
        State = Fullcalibrate;
        // Step2Check = false;
        break;

      case 'c':
        State = Halfcalibrate;
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
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      RightMotorError = 0;
      LeftMotorError = 0;
      TurningCounter = 0;
      TurningCounterL = 0;
      DistanceCounter = 0;
      AngleL = false;
      Angle = false;
      RobotStop_BothMotors();
      State = Stop;
    }
    break;
////////////////////////////////////////////        Basic Movements     ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    case Forward:
    {
      TargetRPM = 100;
      
      MoveRobot_Forward();
      if (ForwardL == 1 || ForwardR == 1 || ForwardC == 1)
      {
        RobotStop_BothMotors();
        State = Stop_SendSensorData;
      }
    }
    break;

    case ForwardGrid:
    {
      // Serial.println("test1");
      if (MoveForward == false) // to reset all counter values in the interrupt
      {
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        DistanceCounter = 0;
        DistanceStop = false;
        MoveForward = true;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        Grid = Dist[NumberOfGrid]; // 0-9
      }
      GetSensorData();
      if (Grid == 22) // set speed for one grid movements
      {
        // md.setSpeeds(-362, -382);
        // md.setSpeeds(-362,-382);
        md.setSpeeds(-370,-400);
      }
      else if (Grid != 22) // set speed for fp
      {
        TargetRPM = 100;
      }
      // MoveRobot_Forward();
      if (DistanceStop == true) // if grid number hit > distancestop = true , stop motor movement
      {
        DistanceStop = false;
        // Serial.println("test");
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        RobotStop_OneGrid();
        MoveForward = false;
        if (Grid == 22)
        {
          delay(350);
          State = CheckFullCalibrationEP; //exploration

          // State = Stop; // fastest path
        }
        if (Grid != 22) //if setting is not set to one grid only do one instruction
        {
          delay(350);
          State = Stop_SendSensorData;
        }
      }
    }
    break;

    case TurnLeft:
    {
      TargetRPM = 80;
      MoveRobot_AntiClockWise();
      if (AngleL == true)
      {
        RobotStop_LeftMotor();
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        TurningCounterL = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        AngleL = false;
        RobotStop_LeftMotor();
        delay(350);
        State = CheckFullCalibrationEP;
      }
    }
    break;

    case TurnRight:
    {
      TargetRPM = 80;
      MoveRobot_ClockWise();
      if (Angle == true)
      {
        RobotStop_RightMotor();
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        TurningCounter = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        RobotStop_RightMotor();
        Angle = false;
        delay(350);
        State = CheckFullCalibrationEP;
      }
    }
    break;

    case abcd:
    {
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      RightMotorError = 0;
      LeftMotorError = 0;
      TurningCounter = 0;
      TurningCounterL = 0;
      RobotStop_LeftMotor();
      Angle = false;
      AngleL = false;
      DistanceCounter = 0;
      State = Stop_SendSensorData;
    }
    break;

    case BackWard:
    {
      md.setSpeeds(80, 80);
      count++;
      if (count == 70)
      {
        count = 0;
        md.setBrakes(400, 400);
        RobotStop_BothMotors();
        State = Stop_SendSensorData;
      }
    }
    break;

    case Fullcalibrate:
    {
      GetSensorData();
      double offset = FrontSenSorLCal - FrontSensorRCal;

      while (FrontSenSorLCal > 10.4 && FrontSensorRCal > 11.5)
      {
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 10.3 && FrontSensorRCal < 11.4)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 0.4)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        GetSensorData();
        if ((FrontSenSorLCal - FrontSensorRCal) >= 0.7)
        {
          md.setSpeeds(-80, 80); // Turn Left
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) >= 0.7)
        {
          md.setSpeeds(80, -80); // Turn Right
        }
      }

      while (FrontSenSorLCal > 10.5 && FrontSensorRCal > 11.6)
      {
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 10.8 && FrontSensorRCal < 11.1)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) >= 0.2)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        GetSensorData();
        if ((FrontSenSorLCal - FrontSensorRCal) >= 0.2)
        {
          md.setSpeeds(-80, 80); // Turn Left
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) >= 0.2)
        {
          md.setSpeeds(80, -80); // Turn Right
        }
      }
      State = TurnLeft1;
    }
    break;

    case TurnLeft1:
    {
      TargetRPM = 80;
      MoveRobot_AntiClockWise();
      if (AngleL == true)
      {
        RobotStop_LeftMotor();
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        TurningCounterL = 0;
        TurningCounter = 0;
        DistanceCounter = 0;
        AngleL = false;
        Angle = false;
        RobotStop_LeftMotor();
        delay(350);
        State = CalibrateFrontEP;
      }
    }
    break;

    case CalibrateFrontEP:
    {
      GetSensorData();
      double offset = FrontSenSorLCal - FrontSensorRCal;
      while (abs(offset) >= 0.05)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        GetSensorData();
        if (FrontSenSorLCal > FrontSensorRCal)
        {
          md.setSpeeds(-80, 80); //Turn Left
        }
        if (FrontSensorRCal > FrontSenSorLCal)
        {
          md.setSpeeds(80, -80); //Turn Right
        }
      }

      while (FrontSenSorLCal > 8 && FrontSensorRCal > 8.5)
      {
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 7.8 && FrontSensorRCal < 8.3)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }
      RobotStop_BothMotors();
      delay(350);
      State = TurnRight1;
    }
    break;

    case TurnRight1:
    {
      Serial.println(TurningCounter);
      TargetRPM = 80;
      MoveRobot_ClockWise();
      if (Angle == true)
      {
        RobotStop_LeftMotor();
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        TurningCounter = 0;
        DistanceCounter = 0;
        Angle = false;
        RobotStop_LeftMotor();
        delay(350);
        State = Halfcalibrate;
      }
    }
    break;

    case Halfcalibrate:
    {
     
      GetSensorData();
      // double offset = LeftSensorLCal - LeftSensorRcal;
      // while(abs(offset) >= 0.05)
      // {
      //   GetSensorData();
      //   offset = LeftSensorLCal - LeftSensorRcal;
      //   if (offset > 0.05)
      //   {
      //   md.setSpeeds(65, -65);
      //     }
      //   if (offset < 0.05)
      // {
      //   md.setSpeeds(-65, 65);
      // }
      // }
      while ((LeftSensorRcal - LeftSensorLCal <= 1.5) || (LeftSensorRcal - LeftSensorLCal >= 1.6))
      {
      GetSensorData();
      if (LeftSensorRcal - LeftSensorLCal >= 1.6)
      {
        md.setSpeeds(65, -65);
      }
      if (LeftSensorRcal - LeftSensorLCal <= 1.5)
      {
        md.setSpeeds(-65, 65);
      }
    }
      RobotStop_BothMotors();
      delay(350);
      State = Stop_SendSensorData;
    }
    break;

    case Stop_SendSensorData:
    {
      RobotStop_OneGrid();
      SendSensorData();
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      RightMotorError = 0;
      DistanceCounter =0;
      LeftMotorError = 0;
      TurningCounter = 0;
      TurningCounterL = 0;
      State = Stop;
    }
    break;

    case CheckFullCalibrationEP: //
    {
      GetSensorData();
      if (LeftShortSensorL == 1 && LeftShortSensorR == 1)
      {
        if (LeftSensorLCal > 20.2 || LeftSensorRcal > 20.8)
        {
          State = TurnLeft1;
        }
        else if (LeftSensorLCal < 14.5 || LeftSensorRcal < 14.5)
        {
          State = TurnLeft1;
        }
        else
        {
          State = Halfcalibrate;
        }
      }
      else
      {
        State = abcd;
      }
    }
      break;

    case CalibrateFrontLeftMiddle:
    {
      GetSensorData();
      double offset = FrontSenSorLCal - FrontSenSorCCal;

      while (FrontSenSorLCal > 13.1 && FrontSenSorCCal > 7.7)
      {
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 12.1 && FrontSenSorCCal < 6.7)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 4.85 || abs(offset) >= 4.9)
      {
        offset = FrontSenSorLCal - FrontSenSorCCal;
        GetSensorData();
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
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 12.1 && FrontSenSorCCal < 6.7)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) <= 5.01 || abs(offset) >= 5.06)
      {
        offset = FrontSenSorLCal - FrontSenSorCCal;
        GetSensorData();
        if ((FrontSenSorLCal - FrontSenSorCCal) >= 5.06)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSenSorCCal - FrontSenSorLCal) <= 5.01)
        {
          md.setSpeeds(80, -80);
        }
      }
      State = Stop_SendSensorData;
    }
    break;

    case CalibrateFrontLeftRight:
    {
      GetSensorData();
      double offset = FrontSenSorLCal - FrontSensorRCal;

      while (FrontSenSorLCal > 12.8 && FrontSensorRCal > 13.1)
      {
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 11.8 && FrontSensorRCal < 12.1)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 0.4)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        GetSensorData();
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
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorLCal < 10.8 && FrontSensorRCal < 11.1)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) >= 0.2)
      {
        offset = FrontSenSorLCal - FrontSensorRCal;
        GetSensorData();
        if ((FrontSenSorLCal - FrontSensorRCal) >= 0.2)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSensorRCal - FrontSenSorLCal) >= 0.2)
        {
          md.setSpeeds(80, -80);
        }
      }
      State = Stop_SendSensorData;
    }
    break;

    case CalibrateFrontRightMiddle:
    {
      GetSensorData();
      double offset = FrontSenSorCCal - FrontSensorRCal;

      while (FrontSenSorCCal > 6.5 && FrontSensorRCal > 11.8)
      {
        GetSensorData();
        md.setSpeeds(-70, -80); // difference of 11
      }

      while (FrontSenSorCCal < 7 && FrontSensorRCal < 12)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      // Serial.println(abs(offset));
      while (abs(offset) <= 5.4 || abs(offset) >= 5.6)
      {
        offset = FrontSenSorCCal - FrontSensorRCal;
        GetSensorData();
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
        GetSensorData();
        md.setSpeeds(-70, -80);
      }

      while (FrontSenSorCCal < 7 && FrontSensorRCal < 12)
      {
        GetSensorData();
        md.setSpeeds(70, 80);
      }

      while (abs(offset) <= 5.5 || abs(offset) >= 5.7)
      {
        offset = FrontSenSorCCal - FrontSensorRCal;
        GetSensorData();
        if ((FrontSensorRCal - FrontSenSorCCal) <= 5.5)
        {
          md.setSpeeds(-80, 80);
        }
        else if ((FrontSensorRCal - FrontSenSorCCal) >= 5.7)
        {
          md.setSpeeds(80, -80);
        }
      }
      State = Stop_SendSensorData;
    }
    break;

    case ForwardStop:
    {
      RobotStop_OneGrid();
      SendSensorData();
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      RightMotorError = 0;
      LeftMotorError = 0;
      TurningCounter = 0;
      TurningCounterL = 0;
      DistanceCounter = 0;
      AngleL = false;
      Angle = false;
      State = Stop;
    }
    break;
    }
    delay(10);
  }
}
////////////////////////////////////////////         COMPUTING PID VALUES      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void ComputePID()
{
  newRightMotorRPM = (RightMotorError * Kpr) + (RightPreviousMotorError * Kdr) + (AllRightMotorError * Kir);
  newLeftMotorRPM = (LeftMotorError * Kp) + (LeftPreviousMotorError * Kd) + (AllLeftMotorError * Ki);
}
void ComputeMotorError()
{
  RightMotorError = TargetRPM - mrRPM;
  LeftMotorError = TargetRPM - mlRPM;
}
void ComputePreviousMotorError()
{

  RightPreviousMotorError = RightMotorError;
  LeftPreviousMotorError = LeftMotorError;
}
void ComputeAllPreviousMotorError()
{
  AllLeftMotorError += LeftPreviousMotorError;
  AllRightMotorError += RightPreviousMotorError;
}
///////////////////////////////////////////         PID CONTROL FOR ROBOT MOVEMENT    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void MoveRobot_Forward()
{
  //Compute MotorError
  ComputeMotorError();

  //Compute PID
  ComputePID();

  //StoreMotorError
  ComputePreviousMotorError();

  //SumofAllPreviousMotorError
  ComputeAllPreviousMotorError();
  md.setM2Speed(-newRightMotorRPM);
  md.setM1Speed(-newLeftMotorRPM);
  // Serial.print(mrRPM);
  // Serial.print(" ");
  // Serial.println(mlRPM);
}
void MoveRobot_ClockWise()
{
  //Compute MotorError
  ComputeMotorError();

  //Compute PID
  ComputePID();

  //StoreMotorError
  ComputePreviousMotorError();

  //SumofAllPreviousMotorError
  ComputeAllPreviousMotorError();

  md.setM2Speed(newRightMotorRPM);
  md.setM1Speed(-newLeftMotorRPM);
  //  Serial.print(mrRPM);
  //  Serial.print(" ");
  //  Serial.println(mlRPM);
}
void MoveRobot_AntiClockWise()
{
  ComputeMotorError();
  ComputePID();
  ComputePreviousMotorError();
  ComputeAllPreviousMotorError();
  md.setM2Speed(-1 * newRightMotorRPM);
  md.setM1Speed(newLeftMotorRPM);
}
void MoveRobot_Backward()
{
  //Compute MotorError
  ComputeMotorError();

  //Compute PID
  ComputePID();

  //StoreMotorError
  ComputePreviousMotorError();

  //SumofAllPreviousMotorError
  ComputeAllPreviousMotorError();
  md.setM2Speed(newRightMotorRPM);
  md.setM1Speed(newLeftMotorRPM);
}
void RobotStop_BothMotors()
{
  RightMotorCounter = 0;
  LeftMotorCounter = 0;
  RightMotorError = 0;
  LeftMotorError = 0;
  TurningCounterL = 0;
  TurningCounter = 0;
  AngleL = false;
  Angle = false;
  DistanceCounter = 0;
  // DistanceTracker = 0;
  DistanceStop = false;
  md.setBrakes(370, 400);
}
void RobotStop_LeftMotor() // for left turn
{
  RightMotorCounter = 0;
  LeftMotorCounter = 0;
  RightMotorError = 0;
  LeftMotorError = 0;
  TurningCounterL = 0;
  TurningCounter = 0;
  AngleL = false;
  Angle = false;
  DistanceCounter = 0;
  DistanceStop = false;
  md.setBrakes(400, 400);
}
void RobotStop_RightMotor() //for right turn
{
  RightMotorCounter = 0;
  LeftMotorCounter = 0;
  RightMotorError = 0;
  LeftMotorError = 0;
  TurningCounterL = 0;
  TurningCounter = 0;
  AngleL = false;
  Angle = false;
  DistanceCounter = 0;
  DistanceStop = false;
  md.setBrakes(400, 400);
}
void RobotStop_OneGrid()
{
  RightMotorCounter = 0;
  LeftMotorCounter = 0;
  RightMotorError = 0;
  LeftMotorError = 0;
  TurningCounterL = 0;
  TurningCounter = 0;
  AngleL = false;
  Angle = false;
  DistanceCounter = 0;
  DistanceStop = false;
  // md.setBrakes(385, 355);
  // md.setBrakes(385,370);
  md.setBrakes(400, 390);
  // md.setBrakes(385,325);
}
//////////////////////////////////////////         Calibrate Sensor      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
double Calibrate_FrontLeftShortSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / FrontLeftSensorGradient;
    double q = FrontLeftSensorConstant / FrontLeftSensorGradient;
    double _output = (M / (analogRead(A0) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 17)
  {
    return 1;
  }
  else if (Output <= 26)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double Calibrate_FrontRightShortSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / FrontRightSensorGradient;
    double q = FrontRightSensorConstant / FrontRightSensorGradient;
    double _output = (M / (analogRead(A1) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 18)
  {
    return 1;
  }
  else if (Output <= 28)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double CalibrateFrontCenterShortSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / FrontCenterSensorGradient;
    double q = FrontCenterSensorConstant / FrontCenterSensorGradient;
    double _output = (M / (analogRead(A4) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 13)
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
double Calibrate_LeftShortSensorR()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / LeftShortSensorGradient;
    double q = LeftShortSensorConstant / LeftShortSensorGradient;
    double _output = (M / (analogRead(A3) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 23)
  {
    return 1;
  }
  else if (Output <= 33)
  {
    return 2;
  }
  return 0; //no object after 3 grids
  // return Output;
}
double Calibrate_LeftShortSensorL()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / LeftShortSensorRGradient;
    double q = LeftShortSensorRConstant / LeftShortSensorRGradient;
    double _output = (M / (analogRead(A5) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 22)
  {
    return 1;
  }
  else if (Output <= 30)
  {
    return 2;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double Calibrate_RightLongSensor()
{
  int SensorCount = 0;
  double Output = 0;
  while (SensorCount != 32)
  {
    double M = 1 / RightLongSensorGradient;
    double q = RightLongSensorConstant / RightLongSensorGradient;
    double _output = (M / (analogRead(A2) + q)) - 8;
    SensorCount++;
    Output += _output;
  }
  Output = Output / 32;

  if (Output <= 23)
  {
    return 3;
  }
  else if (Output <= 32)
  {
    return 4;
  }
  else if (Output <= 43)
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
void LeftMotorCount()
{
  // Serial.println("Left Motor Count");
  LeftMotorCounter++;
  if (LeftMotorCounter == 1)
  {
    StartRPMTimerML = micros();
  }
  else if (LeftMotorCounter == 11)
  {
     DistanceCounter++;
    if (DistanceCounter == Grid)
    {
      DistanceStop = true;
      DistanceCounter = 0;
      // Serial.println("one grid");
    }
    EndRPMTimerML = micros();
    RPMTimerML = (EndRPMTimerML - StartRPMTimerML) / 10.00;
    mlRPM = calculateRPM(RPMTimerML);
    LeftMotorCounter = 0;
  }
}
void RightMotorCount()
{
  
  RightMotorCounter++;
  if (RightMotorCounter == 1)
  {
    StartRPMTimerMR = micros();
  }
  else if (RightMotorCounter == 11)
  {
    TurningCounter++;
    TurningCounterL++;
   
    // Serial.println(TurningCounter);
    if (TurningCounter == 35) // [36[+1},72[+2],108[+4],144[+4],180[+6],216[+6],252[+6],288[+7],324[+9],360[+10],396[+10],432[+11]// [90,180,270,360,450,540,600,720,810,900,990,1080]
    {
      Angle = true;
      TurningCounter = 0;
    }
    if (TurningCounterL == 35)
    {
      AngleL = true;
      TurningCounterL = 0;
    }
    EndRPMTimerMR = micros();
    RPMTimerMR = (EndRPMTimerMR - StartRPMTimerMR) / 10.00;
    mrRPM = calculateRPM(RPMTimerMR);
    RightMotorCounter = 0;
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
void GetSensorData()
{
  ForwardL = Calibrate_FrontLeftShortSensor(); //-5
  ForwardR = Calibrate_FrontRightShortSensor();
  LeftShortSensorL = Calibrate_LeftShortSensorL();
  RightLongSensor = Calibrate_RightLongSensor();
  LeftShortSensorR = Calibrate_LeftShortSensorR();
  ForwardC = CalibrateFrontCenterShortSensor();

  LeftSensorLCal = SideSensor_LCal();
  LeftSensorRcal = SideSensor_RCal();
  FrontSenSorLCal = FrontSensor_LCal();
  FrontSensorRCal = FrontSensor_RCal();
  FrontSenSorCCal = FrontSensor_CCal();
}
void SendSensorData()
{
  ForwardL = Calibrate_FrontLeftShortSensor(); //-5
  ForwardR = Calibrate_FrontRightShortSensor();
  LeftShortSensorL = Calibrate_LeftShortSensorL();
  RightLongSensor = Calibrate_RightLongSensor();
  LeftShortSensorR = Calibrate_LeftShortSensorR();
  ForwardC = CalibrateFrontCenterShortSensor();
  // Serial.println(ForwardC);
  String s = "SDATA:" + String(ForwardC) + ":" + String(ForwardL) + ":" + String(ForwardR) + ":" + String(LeftShortSensorL) + ":" + String(LeftShortSensorR) + ":" + String(RightLongSensor);
  Serial.println(s);
}
///////////////////////////////////////////         Raw Sensor      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
double FrontSensorLeftCal()
{
  double M = 1 / FrontLeftSensorGradient;
  double q = FrontLeftSensorConstant / FrontLeftSensorGradient;
  double Output = (M / (analogRead(A0) + q)) - 8;

  return Output;
}
double FrontSensorRightCal()
{
  double M = 1 / LeftShortSensorGradient;
  double q = LeftShortSensorConstant / LeftShortSensorGradient;
  double Output = (M / (analogRead(A3) + q)) - 8;

  return Output;
}
double FrontSensorCentreCal()
{
  double M = 1 / FrontCenterSensorGradient;
  double q = FrontCenterSensorConstant / FrontCenterSensorGradient;
  double Output = (M / (analogRead(A4) + q)) - 8;

  return Output;
}
double SideSensor_LCal()
{
  double M = 1 / LeftShortSensorRGradient;
  double q = LeftShortSensorRConstant / LeftShortSensorRGradient;
  double Output = (M / (analogRead(A5) + q)) - 8;
  return Output;
}
double SideSensor_RCal()
{
  double M = 1 / LeftShortSensorGradient;
  double q = LeftShortSensorConstant / LeftShortSensorGradient;
  double Output = (M / (analogRead(A3) + q)) - 8;
  return Output;
}
double FrontSensor_LCal()
{
  double M = 1 / FrontLeftSensorGradient;
  double q = FrontLeftSensorConstant / FrontLeftSensorGradient;
  double Output = (M / (analogRead(A0) + q)) - 8;

  return Output;
}
double FrontSensor_RCal()
{
  double M = 1 / FrontRightSensorGradient;
  double q = FrontRightSensorConstant / FrontRightSensorGradient;
  double Output = (M / (analogRead(A1) + q)) - 8;

  return Output;
}
double FrontSensor_CCal()
{
  double M = 1 / FrontCenterSensorGradient;
  double q = FrontCenterSensorConstant / FrontCenterSensorGradient;
  double Output = (M / (analogRead(A4) + q)) - 8;

  return Output;
}