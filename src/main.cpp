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
double Ki = 0.48; //0.9 increase to pump up to able to reach desired speed
double Kd = 0.35; //increase to improve stablity and prevent overshoot from the target

//Right //blue line
double Kpr = 10.5;
double Kir = 0.62;
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
  Halfcalibrate,
  Fullcalibrate,
  CalibrateFrontEP,
  TurnRightEP,
  CalibrateSideEP
};
StateModeFP State = Stop;

//PID computation
void ComputePID();
void ComputeMotorError();
void ComputePreviousMotorError();
void ComputeAllPreviousMotorError();

//Robot Movements
void MoveRobotForward();
void MoveRobotClockWise();
void MoveRobotAntiClockWise();
void MoveRobotBackward();
void RobotStop();

//Sensor Calibrations
double CalibrateFrontLeftShortSensor();
double CalibrateFrontRightShortSensor();
double CalibrateLeftShortSensorR();
double CalibrateRightLongSensor();
double CalibrateLeftShortSensor();

double FrontSensorLeftCal();
double FrontSensorRightCal();
double FrontSensorCentreCal();
double SideSensorL();
double SideSensorR();
double FrontSensorLCal();
double FrontSensorRCal();
double FrontSensorCCal();

//RPM computation
void LeftMotorCount();
void RightMotorCount();
double calculateRPM(double encoderValue);

void GetSensorData();
void SendSensorData();

void CalibrateFront();
void CalibrateSide();

char Movements;
int NumberOfGrid;
int DistanceTracker;
boolean OneGridCheck;
boolean Main = false;
int track = 0;
boolean Calibration = false;
boolean Step2Check = false;
boolean MoveForward = false;
boolean FastestPath = false;
boolean Exploration = false;
//need a way to store 10 grid movement for fastest path
// int Dist[15] = {0, 30, 54, 81, 109, 135, 163, 190, 217, 244};
int Dist[15] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270};
int Grid = 0;
// double offset = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////         VOID SETUP     ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Angle = false;
  AngleL = false;
  DistanceStop = false;
  LeftMotorCounter = 0;
  DistanceTracker = 0;
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
  if (Serial.available() > 0)
  {
    char Modes = Serial.read();
    switch (Modes)
    {
    case 'i':
      // FastestPath = true;
      Main = true;
      Serial.println("Bot_Start");
      SendSensorData();
      break;
    }
  }

  while (Main)
  {
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

      case 'x':
        State = BackWard;
        break;

      case 'z':
        State = Fullcalibrate;
        Step2Check = false;
        break;

      case 'c':
        State = Halfcalibrate;
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
      count = 0;
      DistanceCounter = 0;
      AngleL = false;
      Angle = false;
      RobotStop();
      State = Stop;
      SendSensorData();
    }
    break;

    case Forward:
    {
      TargetRPM = 100;
      MoveRobotForward();
      if (ForwardL == 1 || ForwardR == 1 || ForwardC == 1)
      {
        RobotStop();
        State = Stop;
      }
    }
    break;

    case ForwardGrid:
    {
      if (MoveForward == false)
      {
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightMotorError = 0;
        LeftMotorError = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        DistanceCounter = 0;
        DistanceTracker = 0;
        DistanceStop = false;
        MoveForward = true;
        Grid = Dist[NumberOfGrid]; // 0-9
      }
      GetSensorData();
      if (Grid == 30)
      {
        TargetRPM = 80;
      }
      else if (Grid != 30)
      {
        TargetRPM = 100;
      }
      MoveRobotForward();
      if (ForwardL == 1 || ForwardR == 1 || ForwardC == 1)
      {
        RobotStop();
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
        if (Grid == 30) //if setting is set to one grid
        {
          State = Halfcalibrate;
        }
        if (Grid != 30) //if setting is not set to one grid only do one instruction
        {
          State = Stop;
        }
        delay(1000);
      }
    }
    break;

    case TurnLeft:
    {
      TargetRPM = 120;
      MoveRobotAntiClockWise();
      if (AngleL == true)
      {
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
    }
    break;

    case TurnRight:
    {
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
    }
    break;

    case BackWard:
    {
      TargetRPM = 100;
      MoveRobotBackward();
    }
    break;

    case Fullcalibrate:
    {
      TargetRPM = 120;
      MoveRobotAntiClockWise();
      if (AngleL == true)
      {
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        TurningCounterL = 0;
        count = 0;
        AngleL = false;
        RobotStop();
        State = CalibrateFrontEP;
        delay(1000);
      }
    }
    break;

    case CalibrateFrontEP:
    {
      GetSensorData();
      double FrontSenSorLCal = FrontSensorLCal();
      double FrontSensoraRCal = FrontSensorRCal();
      if(FrontSenSorLCal > 13 && FrontSensoraRCal > 13)
      {
        md.setSpeeds(-80,-80);
      }
      else
      {
        FrontSenSorLCal = FrontSensorLCal();
        FrontSensoraRCal = FrontSensorRCal();
        double offset = FrontSenSorLCal - FrontSensoraRCal;

      while (abs(offset) >= 0.015)
      {
        FrontSenSorLCal = FrontSensorLCal();
        FrontSensoraRCal = FrontSensorRCal();
        offset = FrontSenSorLCal - FrontSensoraRCal;
        if (FrontSenSorLCal < FrontSensoraRCal)
        {
          md.setSpeeds(80, -80);
        }
        if (FrontSensoraRCal < FrontSenSorLCal)
        {
          md.setSpeeds(-80, 80);
        }
      }
      State = TurnRightEP;
      Serial.println("Calibrate finish");
    }
    }
    break;

    case TurnRightEP:
    {
      TargetRPM = 120;
      MoveRobotClockWise();

      if (AngleL == true)
      {
        RightMotorCounter = 0;
        LeftMotorCounter = 0;
        RightPreviousMotorError = RightMotorError;
        LeftPreviousMotorError = LeftMotorError;
        newRightMotorRPM = 0;
        newLeftMotorRPM = 0;
        TurningCounterL = 0;
        AngleL = false;
        Step2Check = false;
        State = CalibrateSideEP;
        count = 0;
        RobotStop();
        delay(1000);
      }
    }
    break;
    case CalibrateSideEP:
    {
        double LeftSensorLCal = SideSensorL();
        double LeftSensorRcal = SideSensorR();
        double offset = LeftSensorLCal - LeftSensorRcal;
      while (abs(offset) >= 0.002)
      {
        LeftSensorLCal = SideSensorL();
        LeftSensorRcal = SideSensorR();
        offset = LeftSensorLCal - LeftSensorRcal;
        if (LeftSensorLCal < LeftSensorRcal)
        {
          md.setSpeeds(80, -80);
        }
        if (LeftSensorRcal < LeftSensorLCal)
        {
          md.setSpeeds(-80, 80);
        }
        Serial.println("Calibrating");
      }
      State = Stop;
      Serial.println("Calibrate finish");
    }
    break;

    case Halfcalibrate: // wall hugging or etc move one grid calibrate one time
    {
      GetSensorData();
      double LeftSensorLCal = SideSensorL();
      double LeftSensorRcal = SideSensorR();
      if(LeftSensorLCal > 10 || LeftSensorRcal > 10)
      {
        State = Fullcalibrate;
      }
      else
      {
        LeftSensorLCal = SideSensorL();
        LeftSensorRcal = SideSensorR();
        double offset = LeftSensorLCal - LeftSensorRcal;
      while (abs(offset) >= 0.002)
      {
        LeftSensorLCal = SideSensorL();
        LeftSensorRcal = SideSensorR();
        offset = LeftSensorLCal - LeftSensorRcal;
        if (LeftSensorLCal < LeftSensorRcal)
        {
          md.setSpeeds(80, -80);
        }
        if (LeftSensorRcal < LeftSensorLCal)
        {
          md.setSpeeds(-80, 80);
        }
        Serial.println("Calibrating");
      }
      State = Stop;
      Serial.println("Calibrate finish");
    }
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
  // Serial.print(mrRPM);
  // Serial.print(" ");
  // Serial.println(mlRPM);
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
  RightMotorCounter = 0;
  LeftMotorCounter = 0;
  RightPreviousMotorError = RightMotorError;
  LeftPreviousMotorError = LeftMotorError;
  newRightMotorRPM = 0;
  newLeftMotorRPM = 0;
  TurningCounterL = 0;
  AngleL = false;
  Angle = false;
  DistanceCounter = 0;
  DistanceStop = false;
  md.setBrakes(400, 400);
}
//////////////////////////////////////////         Calibrate Sensor      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
double CalibrateFrontLeftShortSensor()
{
  double M = 1 / FrontLeftSensorGradient;
  double q = FrontLeftSensorConstant / FrontLeftSensorGradient;
  double Output = (M / (analogRead(A0) + q)) - 8;

  if (Output <= 13)
  {
    return 1;
  }
  else if (Output <= 22)
  {
    return 2;
  }
  else if (Output <= 31) //33 34 32
  {
    return 3;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double CalibrateFrontRightShortSensor() //lousy until 24 cm toh
{
  double M = 1 / FrontRightSensorGradient;
  double q = FrontRightSensorConstant / FrontRightSensorGradient;
  double Output = (M / (analogRead(A1) + q)) - 8;
  if (Output <= 13)
  {
    return 1;
  }
  else if (Output <= 24)
  {
    return 2;
  }
  else if (Output <= 34)
  {
    return 3;
  }
  else
  {
    return 0;
  }
  // return Output;
}
double CalibrateLeftShortSensorR()
{
  double M = 1 / LeftShortSensorGradient;
  double q = LeftShortSensorConstant / LeftShortSensorGradient;
  double Output = (M / (analogRead(A3) + q)) - 8;

  if (Output <= 7)
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
  return 0; //no object after 3 grids
  // return Output;
}
double CalibrateLeftShortSensorL()
{
  double M = 1 / LeftShortSensorRGradient;
  double q = LeftShortSensorRConstant / LeftShortSensorRGradient;
  double Output = (M / (analogRead(A5) + q)) - 8;

  if (Output <= 13)
  {
    return 1;
  }
  else if (Output <= 21)
  {
    return 2;
  }
  else if (Output <= 28)
  {
    return 3;
  }
  else
  {
    return 0;
  }
  // return Output;
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
  if (Output <= 10)
  {
    return 1;
  }
  else if (Output <= 21)
  {
    return 2;
  }
  else if (Output <= 31)
  {
    return 3;
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
  LeftMotorCounter++;
  if (LeftMotorCounter == 1)
  {
    StartRPMTimerML = micros();
  }
  else if (LeftMotorCounter == 11)
  {
    DistanceTracker++;
    if (DistanceTracker == 30)
    {
      SendSensorData();
      DistanceTracker = 0;
    }
    DistanceCounter++;
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
    if (TurningCounterL == 32)
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
  // Serial.println(ForwardC);
  String s = "SDATA:" + String(ForwardC) + ":" + String(ForwardL) + ":" + String(ForwardR) + ":" + String(LeftShortSensorL) + ":" + String(LeftShortSensorR) + ":" + String(RightLongSensor);
  // Serial.println(s);
}
void CalibrateFront()
{
  while (count != 5)
  {
    GetSensorData();
    SendSensorData();
    int moveback = 0;
    while (moveback != 10)
    {
      md.setSpeeds(100, 100);
      moveback++;
    }
    if (ForwardL > 1)
    {
      md.setSpeeds(-100, 100);
      count = 0;
    }
    if (ForwardR > 1)
    {
      md.setSpeeds(100, -100);
      count = 0;
    }
    if (ForwardL > 1 && ForwardR > 1 && ForwardC > 1)
    {
      md.setSpeeds(-115, -120);
      count = 0;
    }
    if (ForwardL == 1 && ForwardR == 1 && ForwardC == 1 && count != 5)
    {
      count++;
    }
    if (count == 5)
    {
      RobotStop();
    }
  }
}
void CalibrateSide()
{

  GetSensorData();
  SendSensorData();
  double LeftSensorLCal = SideSensorL();
  double LeftSensorRcal = SideSensorR();

  double offset = LeftSensorLCal - LeftSensorRcal;

  if (offset < 0)
  {
    md.setSpeeds(80, -80);
  }
  if (offset > 0)
  {
    md.setSpeeds(-80, 80);
  }
  if (offset >= 0.5 || offset <= 1)
  {
    RobotStop();
    State = Stop;
  }
}
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
double SideSensorL()
{
  double M = 1 / LeftShortSensorRGradient;
  double q = LeftShortSensorRConstant / LeftShortSensorRGradient;
  double Output = (M / (analogRead(A5) + q)) - 8;
  return Output;
}
double SideSensorR()
{
  double M = 1 / LeftShortSensorGradient;
  double q = LeftShortSensorConstant / LeftShortSensorGradient;
  double Output = (M / (analogRead(A3) + q)) - 8;
  return Output;
}
double FrontSensorLCal()
{
  double M = 1 / FrontLeftSensorGradient;
  double q = FrontLeftSensorConstant / FrontLeftSensorGradient;
  double Output = (M / (analogRead(A0) + q)) - 8;

  return Output;
}
double FrontSensorRCal()
{
  double M = 1 / FrontRightSensorGradient;
  double q = FrontRightSensorConstant / FrontRightSensorGradient;
  double Output = (M / (analogRead(A1) + q)) - 8;

  return Output;
}
double FrontSensorCcal()
{
  double M = 1 / FrontCenterSensorGradient;
  double q = FrontCenterSensorConstant / FrontCenterSensorGradient;
  double Output = (M / (analogRead(A4) + q)) - 8;

  return Output;
}
