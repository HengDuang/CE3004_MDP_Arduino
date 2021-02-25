#include <Arduino.h>
#include "DualVNH5019MotorShield.h"
#include <EnableInterrupt.h>

DualVNH5019MotorShield md;
//Kp = proportional gain
//Ki = integral gain
//Kd = derivative gain

///////////////////////////////////////////////////////////////////////////////////////////////////////////         CONSTANTS VALUE FOR PID CONSTANT VALUES     ///////////////////////////////////////////////////////////////////////////////////////////////////////////

////Left //red line
double Kp = 11.5; //10 increase to reduce spike at the start
double Ki = 0.52; //0.9 increase to pump up to able to reach desired speed
double Kd = 0.35; //increase to improve stablity and prevent overshoot from the target

//Right //blue line
double Kpr = 10.5;
double Kir = 0.61;
double Kdr = 0.35;
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
int RightMotorCounter = 0; //Counter Value for number of High pulse for Right Motor
int LeftMotorCounter = 0;  //Counter Value for number of High pulse for left Motor

///////////////////////////////////////////////////////////////////////////////////////////////////////////         ROBOT MOVEMENT DECLARATION      ///////////////////////////////////////////////////////////////////////////////////////////////////////////

//Movement for Robot
double TurningCounter = 0;
double TurningCounterL = 0;
double CalibrateTurn = 0;
double DistanceCounter = 0;
double TotalDistanceTravelled = 0;
boolean DistanceStop = false;
boolean Angle = false;
boolean AngleL = false;
int count = 0;

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
int Distance;
int LeftShortSensorL;
int LeftShortSensorR;
int RightLongSensor;

enum StateModeFP
{
  Stop,
  BackWard,
  TurnLeft,
  TurnRight,
  Forward,
  ForwardGrid,
  CalibrateFront,
  CalibrateSide,
  Exit
}; //go forward
//Switch Cases for Robot Movement
StateModeFP State = Stop;

enum StateModeEP
{
  StopEP,
  BackWardEP,
  TurnLeftEP,
  TurnRightEP,
  ForwardEP,
  CalibrateFrontEP,
  CalibrateSideEP,
  ExitEP
}; //go forward
StateModeEP StateEP = StopEP;

void ComputePID();
void ComputeMotorError();
void ComputePreviousMotorError();
void ComputeAllPreviousMotorError();
void MoveRobotForward();
void MoveRobotClockWise();
void MoveRobotAntiClockWise();
void MoveRobotBackward();
void RobotStop();
double CalibrateFrontLeftShortSensor();
double CalibrateFrontRightShortSensor();
double CalibrateLeftShortSensorR();
double CalibrateRightLongSensor();
double CalibrateLeftShortSensor();
void LeftMotorCount();
void RightMotorCount();
double calculateRPM(double encoderValue);
void GetSensorData();
void SendSensorData();

boolean MoveForward = false;

boolean Exploration = false;
boolean FastestPath = false;
char Movements;
int NumberOfGrid;
//need a way to store 10 grid movement for fastest path
int Dist[15] = {0, 26 , 52 , 78 , 102};
int Grid = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////         VOID SETUP     ///////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Angle = false;
  AngleL = false;
  DistanceStop = false;
  LeftMotorCounter = 0;
  RightMotorCounter = 0;
  DistanceCounter = 0;
  TurningCounter = 0;
  TurningCounterL = 0;
  Serial.begin(115200);
  enableInterrupt(RightEncoder, RightMotorCount, RISING); //Set Rising Edge as interrupt
  enableInterrupt(LeftEncoder, LeftMotorCount, RISING);   //Set Rising Edge as interrupt
  md.init();
}
////////////////////////////////////////////////////////////////////////////// Main Function ////////////////////////////////////////////////////////////////////////////////
void loop()
{
  GetSensorData();
  // if (Serial.available() > 0)
  // {
  //   char Modes = Serial.read();
  //   switch (Modes)
  //   {
  //   case '1':
  //     FastestPath = true;
  //     Serial.println("FP_START");
  //     break;

  //   case '2':
  //     Exploration = true;
  //     Serial.println("EX_START");
  //     SendSensorData();
  //     break;
  //   }
  // }
  // while (FastestPath)
  // {
    if (Serial.available() > 0)
    {
      if(Serial.peek() >= 97 && Serial.peek() < 123){   // For normal commands
       Movements = Serial.read();
      }
      else if (Serial.peek() >= 48 && Serial.peek() < 58){   // For 0-9 , append char 'A' and int will be the Gridvalue
       Movements = 'l';
       NumberOfGrid = Serial.parseInt();
      }
      switch (Movements)
      {
      case 'l':
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

      case 'x':
        State = BackWard;
        break;

      case 'z':
        State = CalibrateFront;
        break;

      case 'c':
        State = CalibrateSide;
        break;

      case 'e':
        State = Exit;
        break;
      }
    }
    switch (State)
    {
    case Stop:
      RightMotorCounter = 0;
      LeftMotorCounter = 0;
      RightMotorError = 0;
      LeftMotorError = 0;
      TurningCounter = 0;
      TurningCounterL = 0;
      AngleL = false;
      Angle = false;
      RobotStop();
      State = Stop;
      break;

    case Forward:
      TargetRPM = 100;
      MoveRobotForward();
      if (ForwardL == 1 || ForwardR == 1 ||ForwardC == 1)
      {
        RobotStop();
        SendSensorData();
        State = Stop;
      }
      break;

    case ForwardGrid:
      if (MoveForward == false)
      {
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        DistanceCounter = 0;
        DistanceStop = false;
        MoveForward = true;
        Grid = Dist[NumberOfGrid]; // 0-9
      }
      GetSensorData();
      TargetRPM = 90;
      // md.setSpeeds(-250 , -270);
      MoveRobotForward();
      if (ForwardL == 1 || ForwardR == 1 ||ForwardC == 1)
      {
        RobotStop();
        SendSensorData();
        State = Stop;
        
      }
      if (DistanceStop == true)
      {
        DistanceStop = false;
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        RobotStop();
        MoveForward = false;
        State = ForwardGrid;
        SendSensorData();
        delay(1000);
      }
      break;


    case TurnLeft:

      TargetRPM = 120;
      MoveRobotAntiClockWise();
      if (AngleL == true)
      {
        // Serial.println("Robot TurningLeft. . .");
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        TurningCounterL = 0;
        AngleL = false;
        RobotStop();
        State = Stop;
      }
      break;

    case TurnRight:
      // Serial.println("Robot Turning Right. . .");
      TargetRPM = 120;
      MoveRobotClockWise();
      if (Angle == true)
      {
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        TurningCounter = 0;
        RobotStop();
        Angle = false;
        State = Stop;
      }
      break;

    case BackWard:
      // Serial.println("Robot Backwards. . .");
      TargetRPM = 100;
      MoveRobotBackward();
      break;

    case CalibrateFront:
      GetSensorData();
      if ((ForwardL > 1 && ForwardR > 1) || (ForwardL == 0 && ForwardR == 0)) //Front Sensor too far to obstacle
      {
        md.setSpeeds(-115, -120);
        count = 0;
      }
      if (ForwardL == -1 || ForwardR == -1)
      {
        md.setSpeeds(115, 120);
        count = 0;
      }
      if (ForwardL == 1 || ForwardR == 1)
      {
        if (count == 10)
        {
          RobotStop();
          count = 0;
          State = Stop;
        }
        count++;
      }
      break;

    case CalibrateSide:
      GetSensorData();
      Serial.print(LeftShortSensorL);
      Serial.print("");
      Serial.println(LeftShortSensorR);
      if (LeftShortSensorR > 1)
      {
        md.setSpeeds(55, -60);
        count = 0;
        delay(500);
      }
      if (LeftShortSensorL > 1)
      {
        md.setSpeeds(-55, 60);
        count = 0;
        delay(500);
      }
      if (LeftShortSensorL == -1)
      {
        md.setSpeeds(55, -60);
        count = 0;
        delay(500);
      }
      if (LeftShortSensorR == -1)
      {
        md.setSpeeds(-55, 60);
        count = 0;
        delay(500);
      }
      if (LeftShortSensorL == 1 && LeftShortSensorR == 1)
      {
        if (count == 20)
        {
          RobotStop();
          count = 0;
          State = Stop;
        }
        count++;
      }
      break;

    case Exit:
      FastestPath = false;
      State = Stop;
      break;
    }
    delay(10);
  }

  // while (Exploration)
  // {
  //   if (Serial.available() > 0)
  //   {

  //     char MovementsEP = Serial.read();
  //     switch (MovementsEP)
  //     {
  //     case 'w':
  //       StateEP = ForwardEP;
  //       break;

  //     case 'a':
  //       StateEP = TurnLeftEP;
  //       break;

  //     case 's':
  //       StateEP = StopEP;
  //       break;

  //     case 'd':
  //       StateEP = TurnRightEP;
  //       break;

  //     case 'x':
  //       StateEP = BackWardEP;
  //       break;

  //     case 'e':
  //       StateEP = ExitEP;
  //       break;
  //     }
  //   }
  //   switch (StateEP)
  //   {
  //   case StopEP:
  //     RightMotorCounter = 0;
  //     LeftMotorCounter = 0;
  //     RightMotorError = 0;
  //     LeftMotorError = 0;
  //     TurningCounter = 0;
  //     TurningCounterL = 0;
  //     DistanceCounter = 0;
  //     AngleL = false;
  //     Angle = false;
  //     MoveForward = false;
  //     DistanceStop = false;
  //     RobotStop();
  //     StateEP = StopEP;
  //     break;

  //   case ForwardEP:
  //     if (MoveForward == false)
  //     {
  //       RightMotorCounter = 0;
  //       LeftMotorCounter = 0;
  //       RightMotorError = 0;
  //       LeftMotorError = 0;
  //       RightPreviousMotorError = RightMotorError;
  //       LeftPreviousMotorError = LeftMotorError;
  //       DistanceCounter = 0;
  //       DistanceStop = false;
  //       MoveForward = true;
  //       Grid = Dist[1];
  //     }
  //     GetSensorData();
  //     md.setSpeeds(-250 , -270);
  //     // MoveRobotForward();
  //     if (ForwardL == 1 || ForwardR == 1 ||ForwardC == 1)
  //     {
  //       RobotStop();
  //       SendSensorData();
  //       StateEP = StopEP;
  //     }
  //     if (DistanceStop == true)
  //     {
  //       DistanceStop = false;
  //       RightMotorCounter = 0;
  //       LeftMotorCounter = 0;
  //       RightMotorError = 0;
  //       LeftMotorError = 0;
  //       RightPreviousMotorError = RightMotorError;
  //       LeftPreviousMotorError = LeftMotorError;
  //       RobotStop();
  //       MoveForward = false;
  //       State = Forward;
  //       SendSensorData();
  //       delay(1000);
  //     }
  //     break;

  //   case TurnLeftEP:

  //     TargetRPM = 120;
  //     MoveRobotAntiClockWise();
  //     if (AngleL == true)
  //     {
  //       // Serial.println("Turning Left. . .");
  //       RightMotorCounter = 0;
  //       LeftMotorCounter = 0;
  //       RightPreviousMotorError = RightMotorError;
  //       LeftPreviousMotorError = LeftMotorError;
  //       newRightMotorRPM = 0;
  //       newLeftMotorRPM = 0;
  //       TurningCounterL = 0;
  //       AngleL = false;
  //       RobotStop();
  //       StateEP = StopEP;
  //       SendSensorData();
  //     }
  //     break;

  //   case TurnRightEP:
  //     TargetRPM = 120;
  //     MoveRobotClockWise();
  //     if (Angle == true)
  //     {
  //       RightMotorCounter = 0;
  //       LeftMotorCounter = 0;
  //       RightPreviousMotorError = RightMotorError;
  //       LeftPreviousMotorError = LeftMotorError;
  //       TurningCounter = 0;
  //       RobotStop();
  //       Angle = false;
  //       StateEP = StopEP;
  //       SendSensorData();
  //     }
  //     break;

  //   case BackWardEP:
  //     TargetRPM = 100;
  //     MoveRobotBackward();
  //     break;

  //   case ExitEP:
  //     Exploration = false;
  //     StateEP = StopEP;
  //     break;

  //   default:
  //     State = Stop;
  //     break;
  //   }
  //   delay(10);
  // }

///////////////////////////////////////////////////////////////////////////////////////////////////////////         COMPUTING PID VALUES      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////         PID CONTROL FOR ROBOT MOVEMENT    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void MoveRobotForward()
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
  Serial.print(mrRPM);
  Serial.print(" ");
  Serial.println(mlRPM);
}
void MoveRobotClockWise()
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
void MoveRobotAntiClockWise()
{
  ComputeMotorError();
  ComputePID();
  ComputePreviousMotorError();
  ComputeAllPreviousMotorError();
  md.setM2Speed(-1 * newRightMotorRPM);
  md.setM1Speed(newLeftMotorRPM);
}
void MoveRobotBackward()
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
  //  Serial.print(mrRPM);
  //  Serial.print(" ");
  //  Serial.println(mlRPM);
}
void RobotStop()
{
  md.setBrakes(400, 400);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////         Calibrate Sensor      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
double CalibrateFrontLeftShortSensor()
{
  double M = 1 / FrontLeftSensorGradient;
  double q = FrontLeftSensorConstant / FrontLeftSensorGradient;
  double Output = (M / (analogRead(A0) + q)) - 8;

  // if (Output <= 8)
  // {
  //   return -1;
  // }
 if (Output <= 10)
  {
    return 1;
  }
  else if (Output <= 20)
  {
    return 2;
  }
  else if (Output <= 30)
  {
    return 3;
  }
  else
    return 0;
}
double CalibrateFrontRightShortSensor() //lousy until 24 cm toh
{
  double M = 1 / FrontRightSensorGradient;
  double q = FrontRightSensorConstant / FrontRightSensorGradient;
  double Output = (M / (analogRead(A1) + q)) - 8;

  // if (Output <= 8)
  // {
  //   return -1;
  // }
 if (Output <= 12)
  {
    return 1;
  }
  else if (Output <= 22)
  {
    return 2;
  }
  else if (Output <= 30)
  {
    return 3;
  }
  else
  {
    return 0;
  }
}
double CalibrateLeftShortSensorR()
{
  double M = 1 / LeftShortSensorGradient;
  double q = LeftShortSensorConstant / LeftShortSensorGradient;
  double Output = (M / (analogRead(A3) + q)) - 8;

  // if (Output >= 80)
  // {
  //   return -1; //too close to sensor
  // }
 if (Output <= 11)
  {
    return 1;
  }
  else if (Output <= 23)
  {
    return 2;
  }
  else if (Output <= 32)
  {
    return 3;
  }
  return 0; //no object after 3 grids
}
double CalibrateLeftShortSensorL()
{
  double M = 1 / LeftShortSensorRGradient;
  double q = LeftShortSensorRConstant / LeftShortSensorRGradient;
  double Output = (M / (analogRead(A5) + q)) - 8;

  // if (Output >= 70)
  // {
  //   return -1;
  // }
  if (Output <= 11)
  {
    return 1;
  }
  else if (Output <= 22)
  {
    return 2;
  }
  else if (Output <= 32)
  {
    return 3;
  }
  else
  {
    return 0;
  }
}
double CalibrateRightLongSensor()
{
  double M = 1 / RightLongSensorGradient;
  double q = RightLongSensorConstant / RightLongSensorGradient;
  double Output = (M / (analogRead(A2) + q)) - 8;
  if (Output <= 22)
  {
    return 1;
  }
  else if (Output <= 31)
  {
    return 2;
  }
  else if (Output <= 42)
  {
    return 3;
  }
  else if (Output <= 53)
  {
    return 4;
  }
  else if (Output <= 64)
  {
    return 5;
  }
  else
  {
    return 0;
  }
}
double CalibrateFrontCenterShortSensor()
{
  double M = 1 / FrontCenterSensorGradient;
  double q = FrontCenterSensorConstant / FrontCenterSensorGradient;
  double Output = (M / (analogRead(A4) + q)) - 8;
  // if(Output <= 8)
  // {
  //   return -1;
  // }
  if (Output <= 10)
  {
    return 1;
  }
  else if (Output <= 22)
  {
    return 2;
  }
  else if (Output <= 31)
  {
    return 3;
  }
  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////         RPM COUNTER      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void LeftMotorCount()
{
  LeftMotorCounter++;
  if (LeftMotorCounter == 1)
  {
    StartRPMTimerML = micros();
  }
  else if (LeftMotorCounter == 11)
  {
    DistanceCounter++;
    //       if(DistanceCounter == 27) // 4 grids -> 40cm
    // if (DistanceCounter == 380) //1 grid ->10 cm(26) , 150cm -> 390 100 cm -> 260
    if (DistanceCounter == Grid)
    {
      DistanceStop = true;
      DistanceCounter = 0;
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
  { //1 tick = 5degree
    TurningCounter++;
    TurningCounterL++;
    if (TurningCounter == 34) // [36[+1},72[+2],108[+4],144[+4],180[+6],216[+6],252[+6],288[+7],324[+9],360[+10],396[+10],432[+11]// [90,180,270,360,450,540,600,720,810,900,990,1080]
    {
      Angle = true;
      TurningCounter = 0;
    }
    if (TurningCounterL == 33)
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
void GetSensorData()
{
  ForwardL = CalibrateFrontLeftShortSensor(); //-5
  ForwardR = CalibrateFrontRightShortSensor();
  LeftShortSensorL = CalibrateLeftShortSensorL();
  RightLongSensor = CalibrateRightLongSensor();
  LeftShortSensorR = CalibrateLeftShortSensorR();
  ForwardC = CalibrateFrontCenterShortSensor();
}
void SendSensorData()
{
  ForwardL = CalibrateFrontLeftShortSensor(); //-5
  ForwardR = CalibrateFrontRightShortSensor();
  LeftShortSensorL = CalibrateLeftShortSensorL();
  RightLongSensor = CalibrateRightLongSensor();
  LeftShortSensorR = CalibrateLeftShortSensorR();
  ForwardC = CalibrateFrontCenterShortSensor();

  String s = "SDATA:" + String(ForwardL) + ":" + String(ForwardR) + ":" + String(ForwardC)+":" + String(LeftShortSensorL) + ":" + String(LeftShortSensorR) + ":" + String(RightLongSensor);
  Serial.println(s);
}