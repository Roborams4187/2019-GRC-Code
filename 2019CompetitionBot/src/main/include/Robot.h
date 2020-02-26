/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Pixy2.h"

#include <frc/Joystick.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/Relay.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/DoubleSolenoid.h>
#include <cameraserver/CameraServer.h>

//#include <ctre/Phoenix.h>

class Robot : public frc::TimedRobot {
 public:

  struct driveRate {
    double  left   = 1.0;
    double right   = 1.0;
    double previousSpeed = 0.0;
  }; 
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;



 private:
  int counter = 0;

//****************************************************** Camera Stuff
cs::UsbCamera cargoCam;
cs::UsbCamera hatchCam;
//****************************************************** Global Pneumatic Stuff
  frc::Compressor compressor{0};

//****************************************************** Drivetrain Stuff
  driveRate rate;
  double    leftEncoderSpeed = 0.0;
  double    rightEncoderSpeed = 0.0;

  bool   hatchIsFront  = false;
  bool   manualShift = true;
  int shifter;

  double tempLeftAxis          =  0.0;
  double dCLeftYAxis       =  0.0; 
  double dCRightYAxis      =  0.0;
  double  leftAutoAxis     = -0.7;//  <<Martin
  double rightAutoAxis     =  0.7;//  <<Martin
  double autoTurnAxis      =  0.4;//  <<Martin
  double  leftAutoTurnAxis =  0.0;//  <<Martin
  double rightAutoTurnAxis =  0.0;//  <<Martin
  
  double gyroRead;

  frc::ADXRS450_Gyro gyro;

  frc::Joystick   driverController{0}; // refered to as dC
  frc::Joystick operatorController{1}; // refered to as oC

  frc::Spark  leftTopMotor{2};
  frc::Spark rightTopMotor{1};
  frc::Spark  leftMidMotor{8};
  frc::Spark rightMidMotor{3};
  frc::Spark  leftLowMotor{5};
  frc::Spark rightLowMotor{4};

  frc::Encoder  leftDriveEncoder{0,1};
  frc::Encoder rightDriveEncoder{2,3};
  
  frc::Solenoid shifterValve{7};


//****************************************************** Elevator Stuff
  bool carriageParked             =  true;
  bool parkedOverridden           = false;
  bool manualLiftControl          = false;
  bool elevatorOverridden         =  true;
  bool cargoPosition              = false;
  bool hatchPosition              = false;
  int oCDPad                      =    -1;
  int encoderReading              =     0;
  int elevatorLevel               =     0;
  //int currentLevel                =    20;
  const int maxElevator           =  2476;
  const int rocketCargoLow        =   815;
  const int rocketCargoMid        =  2067;
  const int rocketCargoHigh       =  1935;
  const int rocketHatchLow        =   368;
  const int rocketHatchMid        =  1405;
  const int rocketHatchHigh       =  2465;
  const int shipCargo             =  1435;
  const int shipHatch             =    10;
  const int cargoIntake           =    10;
  const int hatchIntake           =    10;
  double oCLeftYAxis              =   0.0;
  // const int hatchIntake           =   20;  NOTE: This is same as hatchLow.

  frc::Encoder elevatorEncoder{7,8};

  frc::Spark elevatorMotor9{9};
  frc::Spark elevatorMotor7{7};

//****************************************************** Cargo Intake Stuff
  bool rollerArmOverridden = false;
  bool rollerOverridden    = false;
  bool cargoIntakeActive   = false;
  bool okayToShoot         = false;
  bool cargoPresent        = false;
  bool holdArm             = false;
  bool egg                 =  true;
  bool jason               =   egg;
  double oCRightYAxis      =   0.0;

  double cargoTrigger      =   0.0;

  double rollerSpeed       =   0.65;

  frc::Spark rollerMotor{6}; //Both left & right Arm Motors on one Spark
  //TalonSRX armMotor = {0};
  


  frc::DigitalInput rollerSwitch{8};
  frc::DigitalInput iRSensor{9};
  frc::DoubleSolenoid cargoIntakeValve{2,3};
  frc::DoubleSolenoid hatchIntakeValve{4,5};
  frc::Relay shooterWheel{0};
//******************************************************** Pixy!!
  frc::I2C pixy2_0{frc::I2C::kOnboard,84};
  frc::I2C pixy2_1{frc::I2C::kOnboard,85};
  frc::I2C* pixyPointer0 = &pixy2_0;
  frc::I2C* pixyPointer1 = &pixy2_1;
  martin::PixyVector pixyLine0;
  martin::PixyVector pixyLine1;
  bool pixyMode = false;
  int centerX = 38;
};
