#include <Arduino.h>
#include "DualVNH5019MotorShield.h"
#include <EnableInterrupt.h>

DualVNH5019MotorShield md;
//Kp = proportional gain
//Ki = integral gain
//Kd = derivtive gain

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
char Directions ;
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
int LeftMotorCounter = 0; //Counter Value for number of High pulse for left Motor

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

double LeftShortSensorRGradient = 0.00008886449832183208;
double LeftShortSensorRConstant = 0.01077191214530613;

double RightLongSensorGradient = 0.0000636965275884001;
double RightLongSensorConstant = 0.0004925657171665529;

double LeftShortSensorGradient = 0.00009553611726362321;
double LeftShortSensorConstant = 0.007772562954454363;


int ForwardL;
int ForwardR;
int Distance;
int LeftShortSensorL;
int LeftShortSensorR;
int RightLongSensor;

enum StateMode {Stop , BackWard , TurnLeft , TurnRight , Forward , CalibrateFront , CalibrateSide}; //go forward
//Switch Cases for Robot Movement
StateMode State = Stop;

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
double calculateRPM (double encoderValue);
void GetSensorData();
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
  TurningCounterL =0;
  Serial.begin(115200);
  enableInterrupt(RightEncoder, RightMotorCount, RISING); //Set Rising Edge as interrupt
  enableInterrupt(LeftEncoder, LeftMotorCount, RISING); //Set Rising Edge as interrupt
  md.init();
}
////////////////////////////////////////////////////////////////////////////// Main Function ////////////////////////////////////////////////////////////////////////////////
void loop()
{  
//        GetSensorData();
//        Serial.print(LeftShortSensorL);
//        Serial.print("    ");
//        Serial.println(LeftShortSensorR);
    if (Serial.available() > 0) {

      char Movements = Serial.read();
      switch (Movements)
      {
        case 'w':
          State = Forward;
          break;

        case 's':
          State = Stop;
          break;

        case 'a':
          State = TurnLeft;
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
        Serial.println("Robot Moving. . .");
        GetSensorData();
        TargetRPM = 100;
        MoveRobotForward();
        if(ForwardL == 10 || ForwardR ==10)
        {
          RobotStop();
          State = Stop;
        }
        break;

      case TurnLeft:
        
        TargetRPM = 120;
        MoveRobotAntiClockWise();
        if (AngleL == true)
        {
          Serial.println("Robot TurningLeft. . .");
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
        Serial.println("Robot Turning Right. . .");
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
        Serial.println("Robot Backwards. . .");
        TargetRPM = 100;
        MoveRobotBackward();
        break;
      
      case CalibrateFront:
        GetSensorData();
      if ((ForwardL > 1 && ForwardR > 1) || (ForwardL == 0 && ForwardR ==0)) //Front Sensor too far to obstacle
      {
        md.setSpeeds(-115,-120);
        count =0;
      }
      if(ForwardL == -1 || ForwardR == -1)
      {
        md.setSpeeds(115,120);
        count =0;
      }  
      if(ForwardL == 1 || ForwardR == 1)
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
        if(LeftShortSensorR > 1)
        {
          md.setSpeeds(55,-60);
          count =0;
          delay(500);
        }
        if(LeftShortSensorL > 1)
        {
          md.setSpeeds(-55,60);
          count = 0;
          delay(500);
        }
        if(LeftShortSensorL == -1)
        {
          md.setSpeeds(55,-60);
          count =0;
          delay(500);
        }
        if(LeftShortSensorR == -1)
        {
          md.setSpeeds(-55,60);
          count = 0;
          delay(500);
        }
        if(LeftShortSensorL == 1 && LeftShortSensorR == 1)
        {
          if(count == 20)
          {
            RobotStop();
            count =0;
            State = Stop;
          }
          count++;
        }
        break;
  }
  delay(10);
}
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
  
  if(Output <= 8)
  {
    return -1;
  }
  else if(Output <= 10)
  {
    return 1;
  }
  else if(Output <= 20)
  {
    return 2; 
  }
  else if(Output <=30)
  {
    return 3;
  }
  else return 0;
  
}
double CalibrateFrontRightShortSensor() //lousy until 24 cm toh
{
  double M = 1 / FrontRightSensorGradient;
  double q = FrontRightSensorConstant / FrontRightSensorGradient;
  double Output = (M / (analogRead(A1) + q)) - 8;

  if(Output <= 8)
  {
    return -1;
  }
  else if(Output <=12)
  {
    return 1;
  }
  else if(Output <= 22)
  {
    return 2;
  }
  else if(Output <=30)
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

  if(Output >= 80)
  {
    return -1; //too close to sensor
  }
  else if(Output <=11)
  {
    return 1;
  }
  else if(Output <= 23)
  {
    return 2;
  }
  else if(Output <= 32)
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
  
  if(Output >= 70)
  {
    return -1;
  }
  else if(Output <= 11)
  {
    return 1;
  }
  else if(Output <= 22)
  {
    return 2;
  }
  else if(Output <= 32)
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
    return 7;
  }


}

///////////////////////////////////////////////////////////////////////////////////////////////////////////         RPM COUNTER      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void LeftMotorCount()
{
  LeftMotorCounter++;
  if (LeftMotorCounter == 1) {
    StartRPMTimerML = micros();
  }
  else if (LeftMotorCounter == 11) {
    DistanceCounter++;
    //       if(DistanceCounter == 27) // 4 grids -> 40cm
    if (DistanceCounter == 380) //1 grid ->10 cm(26) , 150cm -> 390 100 cm -> 260
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
  if (RightMotorCounter == 1) {
    StartRPMTimerMR = micros();
  }
  else if (RightMotorCounter == 11) { //1 tick = 5degree
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
double calculateRPM (double encoderValue) {
  if (encoderValue == 0) {
    return 0;
  }
  else {
    return (60.00 / ((encoderValue  * 562.25) / 1000000.00));
  }
}
void GetSensorData()
{
  ForwardL = CalibrateFrontLeftShortSensor(); //-5
  ForwardR = CalibrateFrontRightShortSensor();
  LeftShortSensorL = CalibrateLeftShortSensorL();
  RightLongSensor = CalibrateRightLongSensor();
  LeftShortSensorR = CalibrateLeftShortSensorR();
  String s = "SDATA:" + String(ForwardL) + ":" + String(ForwardR) + ":" + String(LeftShortSensorL) + ":" + String(LeftShortSensorR) + ":" + String(RightLongSensor);
  //  Serial.println(s);
}