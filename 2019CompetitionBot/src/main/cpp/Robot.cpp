/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>



//                                                                         {*** ROBOT INIT ***}
void Robot::RobotInit() {

  compressor.SetClosedLoopControl(true);  //                                    Start Compressor.
  cargoCam = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);  //    Start USB Camera.
  hatchCam = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);  //    Start USB Camera.
  shifterValve.Set(true);  //                                     Pre-Set DriveTrain to LOW Gear.
  cargoIntakeValve.Set(frc::DoubleSolenoid::kForward);  // Pre-Set Cargo Intake for LOADED Cargo.

}

//                                                                       {*** ROBOT PERIODIC ***}
void Robot::RobotPeriodic() {}


//======================================================================================================================================================  

//            A     U      U  TTTTTTT   OOOO    N     N    OOOO    M      M    OOOO    U      U   SSSSS                                                   
//           A A    U      U     T     O    O   NN    N   O    O   MM    MM   O    O   U      U  S     S    
//          A   A   U      U     T    O      O  N N   N  O      O  M M  M M  O      O  U      U  S
//         AAAAAAA  U      U     T    O      O  N  N  N  O      O  M  MM  M  O      O  U      U   SSSSS     
//         A     A  U      U     T    0      0  N   N N  O      O  M      M  O      O  U      U        S
//         A     A   U    U      T     O    O   N    NN   O    O   M      M   O    O    U    U   S     S                             
//         A     A    UUUU       T      OOOO    N     N    OOOO    M      M    OOOO      UUUU     SSSSS                                        



//                                                                     {*** AUTONOMOUS INIT ***}
void Robot::AutonomousInit() {

  cargoIntakeValve.Set(frc::DoubleSolenoid::kForward);  //  Redundant - LOADED Intake.
  shifterValve.Set(true);  //              Redundant - Pre-Set DriveTrain to LOW Gear.
  elevatorEncoder.Reset();  //                            Start Elevator Encoder at 0.

   //                                            Drop Roller Arm into Intake position.
//  armMotor.Set(ControlMode::PercentOutput, -0.1);
//  frc::Wait(0.7);
//  armMotor.Set(ControlMode::PercentOutput, 0.1);
//  frc::Wait(0.5);
//  armMotor.Set(ControlMode::PercentOutput, 0.0);

}



//                                                                 {*** AUTONOMOUS PERIODIC ***}
void Robot::AutonomousPeriodic() {


//                                                                       A    RRRR   M   M                                               
//                                                                      A A   R   R  MM MM                                                 
//                                                                     AAAAA  RRRR   M M M                                                     
//                                                                     A   A  R  R   M   M                                              
//                                                                     A   A  R   R  M   M                                              

  oCRightYAxis = operatorController.GetRawAxis(5); //Define arm axis as right stick Y axis

//  if (oCRightYAxis < -0.2) { //Stick down, put arm down
//    armMotor.Set(ControlMode::PercentOutput, oCRightYAxis*0.5);
//  }
//  else if (oCRightYAxis > 0.2 ) { //Stick up, put arm up
//    armMotor.Set(ControlMode::PercentOutput, oCRightYAxis*0.3);
//  }
//  else { //No input, put no power
//    armMotor.Set(ControlMode::PercentOutput, 0);
//  }

//                                  EEEEE  L      EEEEE  V   V    A    TTTTT   OOO   RRRR                                                                                  
//                                  E      L      E      V   V   A A     T    O   O  R   R                                                                   
//                                  EEE    L      EEE     V V   AAAAA    T    O   O  RRRR                                                                                
//                                  E      L      E       V V   A   A    T    O   O  R  R                                                              
//                                  EEEEE  LLLLL  EEEEE    V    A   A    T     OOO   R   R                                                                              


  if (operatorController.GetRawButton(5)) { //Operator Right bumper pressed, override elevator
    elevatorOverridden = true;  // NOTE: This is normally false so auto elevator functions work
  }
  else if (operatorController.GetRawAxis(2) > 0.8) { //Operator Right trigger squeezed, auto ele
    elevatorOverridden = true;
  }


  if (cargoIntakeActive) { //Decide elevator position. Set ele to intake level 
    elevatorLevel = cargoIntake;
    cargoPosition = false;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(1)) { //A is pressed, ele to rocket cargo low
    elevatorLevel = rocketCargoLow;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(2)) { //B is pressed, ele to rocket cargo mid
    elevatorLevel = rocketCargoMid;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(3)) { //X is pressed, ele to ship cargo
    elevatorLevel = shipCargo;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(4)) { //Y is pressed, ele to rocket cargo high
    elevatorLevel = rocketCargoHigh;
    cargoPosition = true;
    hatchPosition = false;
  }
  else { //Go to DPad buttons
    oCDPad = operatorController.GetPOV(); //Define DPad
    if (oCDPad != -1) {
      switch (oCDPad) {
        case 0: //Top DPad is pressed, ele to rocket hatch high
          elevatorLevel = rocketHatchHigh;
          hatchPosition = true;
          cargoPosition = false;
        break;
        case 90: //Right DPad is pressed, ele to rocket hatch middle
          elevatorLevel = rocketHatchMid;
          hatchPosition = true;
          cargoPosition = false;
        break;
        case 180: //Bottom DPad is pressed, ele to rocket hatch low
          elevatorLevel = rocketHatchLow;
          hatchPosition = true;
          cargoPosition = false;
        break;
      }// end switch
    }// end inner if
  }// end if-else

  encoderReading = elevatorEncoder.Get();  // Get the Elevator ENCODER Value.

  if (elevatorOverridden) {//                                          >>> MANUAL ELEVATOR
    oCLeftYAxis = operatorController.GetRawAxis(1); //Define Y axis on operator left stick
    if (oCLeftYAxis < -0.2 ) {   //       Stick is UPPPPP
      elevatorMotor9.Set(-oCLeftYAxis*0.9); //Stick is negative value (for up), set eleM positive
      elevatorMotor7.Set(-oCLeftYAxis*0.9);
    }
    else if (oCLeftYAxis > 0.2) { //Stick is DOWNNNNNNNNNNNN
      elevatorMotor9.Set(-oCLeftYAxis*0.3); //Stick is positive value (for down), set eleM negative
      elevatorMotor7.Set(-oCLeftYAxis*0.3);
    }
    else {
      elevatorMotor9.Set(0.25);  //Stick is idle - HOLDDDDDD
      elevatorMotor7.Set(0.25); 
    }
  }
  else {//                                                               >>> AUTO ELEVATOR
std::cout << "Auto Ele, ";
std::cout << encoderReading << std::endl;
    if (elevatorLevel > encoderReading) { //Elevator is below target
      carriageParked = false;
      elevatorMotor9.Set(0.9); //Go up 
      elevatorMotor7.Set(0.9);
std::cout << "going up." << std::endl;
    }
    else if (elevatorLevel < encoderReading) { //Elevator is above target
      carriageParked = false;
      elevatorMotor9.Set(-0.3); //Go down
      elevatorMotor7.Set(-0.3);
std::cout << "going down." << std::endl;
    }
    else { //Elevator is at target. Hold.
      elevatorMotor9.Set(0.25); 
      elevatorMotor7.Set(0.25); 
      carriageParked = true;
    }
  }


//          CCCC    A    RRRR    GGGG   OOO       IIIII  N   N  TTTTT    A    K   K  EEEEE                                                                
//         C       A A   R   R  G      O   O        I    NN  N    T     A A   K  K   E                                                 
//         C      AAAAA  RRRR   G  GG  O   O        I    N N N    T    AAAAA  KKK    EEE                                                        
//         C      A   A  R  R   G   G  O   O        I    N  NN    T    A   A  K  K   E                                                 
//          CCCC  A   A  R   R   GGG    OOO       IIIII  N   N    T    A   A  K   K  EEEEE                                                       

  if ((encoderReading < 100) and cargoIntakeActive) {  //Elevator is at good position and intake is true
    rollerMotor.Set(0.75); //                Manual & Auto rollerMotor starter.
  }
  else if (driverController.GetRawButton(3) and not cargoIntakeActive){//Driver X pressed & not in intake
    rollerMotor.Set(-0.75); //Manual control to inverse(?) roller 
  }
  else if (driverController.GetRawButton(4) and not cargoIntakeActive){//Y pressed & not in intake
    rollerMotor.Set(0.45); //Manual control to power roller
  }
 else {
    rollerMotor.Set(0.0); //              Turn roller off if elevator goes up.
  }

  if (elevatorOverridden){ //                                            >>> MANUAL INTAKE
    if (driverController.GetRawButton(6) and iRSensor.Get()) { //Driver Right bumper, no ball present
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse); //Intake position
      cargoIntakeActive = true; 
    }
    else if (driverController.GetRawAxis(3) > 0.8) { //Driver right (?) trigger
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse); //Intake position, to shoot or cancel
      rollerMotor.Set(0.0); //Roller off
      shooterWheel.Set(frc::Relay::Value::kOff); //Shooter wheel off
      cargoIntakeActive = false; //Shoot or cancel. Either one means we are no longer in intake
    }
    else if (cargoIntakeActive and not iRSensor.Get()) { //In intake and ball is present
      cargoIntakeValve.Set(frc::DoubleSolenoid::kForward); //GRAB THE BALL
      rollerMotor.Set(0.0); //Roller off
      shooterWheel.Set(frc::Relay::Value::kReverse); //Shooter wheel on
      counter = 0;
      cargoIntakeActive = false; //Ball caught/tried to catch. Not in intake.
    }
  }
  else { //                                                                >>> AUTO INTAKE
    if (cargoPosition //If in a cargo position, holding, & right trigger squeezed
              and carriageParked 
              and (driverController.GetRawAxis(3) > 0.8)) {
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse); //   Shoot Cargo.
    }
    else if (driverController.GetRawAxis(3) > 0.8) { // Cancel Cargo Intake.
      cargoIntakeActive = false; 
      rollerMotor.Set(0.0);
      shooterWheel.Set(frc::Relay::Value::kOff);
    } 
    else if (driverController.GetRawButton(6) //         Start Cargo Intake.
              and iRSensor.Get()) { //           ONLY when no cargo present.
      cargoIntakeActive = true;
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse);
    }
    else if (cargoIntakeActive and not iRSensor.Get()) { //Intake mode & ball present
      cargoIntakeValve.Set(frc::DoubleSolenoid::kForward); //Grab ball yesssss
      rollerMotor.Set(0); //Off with the roller
      shooterWheel.Set(frc::Relay::Value::kReverse); //Setup shooter wheel
      counter = 0;
      cargoIntakeActive = false; //Grabbing ball, so not in intake
    }
  }

  counter = counter+1; //       After about a second, turn-off shooterWheel. 
  if (counter >= 40) { //50 counter values equals one second
    shooterWheel.Set(frc::Relay::Value::kOff);
    counter = 0;
  }

//                         H   H    A    TTTTT   CCCC  H  H       CCCC  L        A   W   W                                  
//                         H   H   A A     T    C      H  H      C      L       A A  W   W                           
//                         HHHHH  AAAAA    T    C      HHHH      C      L      AAAAA W W W                                
//                         H   H  A   A    T    C      H  H      C      L      A   A WW WW                            
//                         H   H  A   A    T     CCCC  H  H       CCCC  LLLLL  A   A W   W                                      

  if (driverController.GetRawButton(5)) { // Driver left bumper pressed
    hatchIntakeValve.Set(frc::DoubleSolenoid::kForward); //Open claw/Grab hatch
  }
  else if (driverController.GetRawAxis(2) > 0.8) { //Driver left trigger
    hatchIntakeValve.Set(frc::DoubleSolenoid::kReverse); //Close claw/Release hatch
  } 


//                    DDDD   RRRR   IIIII  V   V  EEEEE  TTTTT  RRRR     A    IIIII  N   N                                                       
//                    D   D  R   R    I    V   V  E        T    R   R   A A     I    NN  N                                 
//                    D   D  RRRR     I    V   V  EEE      T    RRRR   AAAAA    I    N N N                                        
//                    D   D  R  R     I     V V   E        T    R  R   A   A    I    N  NN                                 
//                    DDDD   R   R  IIIII    V    EEEEE    T    R   R  A   A  IIIII  N   N                                                 

  dCLeftYAxis = driverController.GetRawAxis(1);  //    Read driver gamepad joysticks.
  dCRightYAxis = -driverController.GetRawAxis(5);



  shifter = driverController.GetPOV();  //          Decide shift mode. (default true)
  if (shifter == 0) {
    manualShift = false;
  }
  else if (shifter == 180) {
    manualShift = true;
  }

  if    ((((dCLeftYAxis < -0.95) and (dCRightYAxis > 0.95))  //    Go to AUTO HIGH Gear.
      or   ((dCLeftYAxis > 0.95) and (dCRightYAxis < -0.95)))
      and (not manualShift)) {
    shifterValve.Set(false);
  }
  else if (not manualShift) {  //                                 Go to AUTO LOW Gear.
    shifterValve.Set(true);
  }
  else if (driverController.GetRawButton(1)) {  //              Go to MANUAL LOW Gear.
    shifterValve.Set(true);
  }
  else if (driverController.GetRawButton(2)) {  //             Go to MANUAL HIGH Gear.
    shifterValve.Set(false);
  }

  if (driverController.GetRawButton(7)) { //       Change FRONT to HATCH perspective.
    hatchIsFront = true;
  }

if (driverController.GetPOV() == 0) { //Enable pixy mode
    pixyMode = true;
    std::cout << "Pixy enabled" << std::endl;
  } else if (driverController.GetPOV() == 180) {
    pixyMode = false;
    std::cout << "Pixy disabled" << std::endl;
  }

  
if ((dCRightYAxis > 0.2) and pixyMode and not hatchIsFront) { //Do the pixy sheet
  dCRightYAxis = dCRightYAxis*0.5;
  std::cout << pixyLine0.m_x0 << std::endl;
  if (pixyLine0.m_x0 == 128) {
    std::cout << "Pixy has lost the line" << std::endl;
    dCRightYAxis = dCRightYAxis*2;
  } else if (pixyLine0.m_x0 < centerX) {
    dCLeftYAxis = -0.4;
    dCRightYAxis = 0.21;
  } else if (pixyLine0.m_x0 > centerX) {
    dCLeftYAxis = -0.21;
    dCRightYAxis = 0.4;
  } else {
    dCLeftYAxis = -dCRightYAxis;
  }
}

  else if (driverController.GetRawButton(8)) {  // Change FRONT to CARGO perspective.
    hatchIsFront = false;
  }

  if (hatchIsFront) {  //   Swap driver gamepad joystick inputs for HATCH perspecive.
    tempLeftAxis = dCLeftYAxis;
    dCLeftYAxis = dCRightYAxis;
    dCRightYAxis = tempLeftAxis;
  }
  
  if ((dCLeftYAxis > 0) and (dCRightYAxis < 0)) {
    dCLeftYAxis = dCLeftYAxis * 0.75;
    dCRightYAxis = dCRightYAxis * 0.75;
  }

  if ((dCLeftYAxis > 0.2) or (dCLeftYAxis < -0.2)) {  //             Run LEFT Motors.
    leftTopMotor.Set(dCLeftYAxis);
    leftMidMotor.Set(dCLeftYAxis);
    leftLowMotor.Set(dCLeftYAxis);
   } else {
    leftTopMotor.Set(0);
    leftMidMotor.Set(0);
    leftLowMotor.Set(0);
   } 
  
  if ((dCRightYAxis > 0.2) or (dCRightYAxis < -0.2)) {  //          Run RIGHT Motors.
    rightTopMotor.Set(dCRightYAxis);
    rightMidMotor.Set(dCRightYAxis);
    rightLowMotor.Set(dCRightYAxis);
  } else {
    rightTopMotor.Set(0);
    rightMidMotor.Set(0);
    rightLowMotor.Set(0);
  }

//                                                  OOO   U   U  TTTTT PPPP   U   U  TTTTT                                                                      
//                                                 O   O  U   U    T   P   P  U   U    T                                                       
//                                                 O   O  U   U    T   PPPP   U   U    T                                                        
//                                                 O   O  U   U    T   P      U   U    T                                                     
//                                                  OOO    UUU     T   P       UUU     T                                                        


 }


//======================================================================================================================================================  

//                                          TTTTTTT  EEEEEEE  L        EEEEEEE           OOOO    PPPPPP                                                                                                          
//                                             T     E        L        E                O    O   P     P                                                                           
//                                             T     E        L        E               O      O  P     P                                                                                
//                                             T     EEEE     L        EEE      =====  O      O  PPPPPP                                                                                        
//                                             T     E        L        E               O      O  P                                                                                      
//                                             T     E        L        E                O    O   P                                                                                      
//                                             T     EEEEEEE  LLLLLLL  EEEEEEE           OOOO    P                                                                                                            



//                                                                        {*** TELE-OP INIT ***}
void Robot::TeleopInit() {

  shifterValve.Set(true); //                       Re-Set DriveTrain to LOW Gear.

}



//                                                                    {*** TELE-OP PERIODIC ***}
void Robot::TeleopPeriodic() {


//                                                                       A    RRRR   M   M                                               
//                                                                      A A   R   R  MM MM                                                 
//                                                                     AAAAA  RRRR   M M M                                                     
//                                                                     A   A  R  R   M   M                                              
//                                                                     A   A  R   R  M   M                                              

  oCRightYAxis = operatorController.GetRawAxis(5); //Define arm axis as right stick Y axis

//  if (oCRightYAxis < -0.2) { //Stick down, put arm down
//    armMotor.Set(ControlMode::PercentOutput, oCRightYAxis*0.5);
//  }
//  else if (oCRightYAxis > 0.2 ) { //Stick up, put arm up
//    armMotor.Set(ControlMode::PercentOutput, oCRightYAxis*0.3);
//  }
//  else { //No input, put no power
//    armMotor.Set(ControlMode::PercentOutput, 0);
//  }

//                                  EEEEE  L      EEEEE  V   V    A    TTTTT   OOO   RRRR                                                                                  
//                                  E      L      E      V   V   A A     T    O   O  R   R                                                                   
//                                  EEE    L      EEE     V V   AAAAA    T    O   O  RRRR                                                                                
//                                  E      L      E       V V   A   A    T    O   O  R  R                                                              
//                                  EEEEE  LLLLL  EEEEE    V    A   A    T     OOO   R   R                                                                              


  if (operatorController.GetRawButton(5)) { //Operator Right bumper pressed, override elevator
    elevatorOverridden = true;  // NOTE: This is normally false so auto elevator functions work
  }
  else if (operatorController.GetRawAxis(2) > 0.8) { //Operator Right trigger squeezed, auto ele
    elevatorOverridden = true;
  }


  if (cargoIntakeActive) { //Decide elevator position. Set ele to intake level 
    elevatorLevel = cargoIntake;
    cargoPosition = false;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(1)) { //A is pressed, ele to rocket cargo low
    elevatorLevel = rocketCargoLow;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(2)) { //B is pressed, ele to rocket cargo mid
    elevatorLevel = rocketCargoMid;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(3)) { //X is pressed, ele to ship cargo
    elevatorLevel = shipCargo;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(4)) { //Y is pressed, ele to rocket cargo high
    elevatorLevel = rocketCargoHigh;
    cargoPosition = true;
    hatchPosition = false;
  }
  else { //Go to DPad buttons
    oCDPad = operatorController.GetPOV(); //Define DPad
    if (oCDPad != -1) {
      switch (oCDPad) {
        case 0: //Top DPad is pressed, ele to rocket hatch high
          elevatorLevel = rocketHatchHigh;
          hatchPosition = true;
          cargoPosition = false;
        break;
        case 90: //Right DPad is pressed, ele to rocket hatch middle
          elevatorLevel = rocketHatchMid;
          hatchPosition = true;
          cargoPosition = false;
        break;
        case 180: //Bottom DPad is pressed, ele to rocket hatch low
          elevatorLevel = rocketHatchLow;
          hatchPosition = true;
          cargoPosition = false;
        break;
      }// end switch
    }// end inner if
  }// end if-else

  encoderReading = elevatorEncoder.Get();  // Get the Elevator ENCODER Value.

  if (elevatorOverridden) {//                                          >>> MANUAL ELEVATOR
    oCLeftYAxis = operatorController.GetRawAxis(1); //Define Y axis on operator left stick
    if (oCLeftYAxis < -0.2 ) {   //       Stick is UPPPPP
      elevatorMotor9.Set(-oCLeftYAxis*0.9); //Stick is negative value (for up), set eleM positive
      elevatorMotor7.Set(-oCLeftYAxis*0.9);
    }
    else if (oCLeftYAxis > 0.2) { //Stick is DOWNNNNNNNNNNNN
      elevatorMotor9.Set(-oCLeftYAxis*0.3); //Stick is positive value (for down), set eleM negative
      elevatorMotor7.Set(-oCLeftYAxis*0.3);
    }
    else {
      elevatorMotor9.Set(0.25);  //Stick is idle - HOLDDDDDD
      elevatorMotor7.Set(0.25); 
    }
  }
  else {//                                                               >>> AUTO ELEVATOR
std::cout << "Auto Ele, ";
std::cout << encoderReading << std::endl;
    if (maxElevator < encoderReading) { //              Elevator has gone too high
    elevatorOverridden = true;
    }
    else if ((elevatorLevel - 15) > encoderReading) { //Elevator is below target
      carriageParked = false;
      elevatorMotor9.Set(0.9); //Go up 
      elevatorMotor7.Set(0.9);
std::cout << "going up." << std::endl;
    }
    else if ((elevatorLevel + 5) < encoderReading) { //Elevator is above target
      carriageParked = false;
      elevatorMotor9.Set(-0.3); //Go Down
      elevatorMotor7.Set(-0.3);
std::cout << "going down." << std::endl;
    }
    else { //                                     Elevator is at target. Hold.
      std::cout << "holding." << std::endl;
      if (encoderReading < 20) {// Zero HOLD Pressure at Bottom Level
        elevatorMotor9.Set(0.0); 
        elevatorMotor7.Set(0.0);
      }
      else {//                    Regular HOLD Pressure
        elevatorMotor9.Set(0.3); 
        elevatorMotor7.Set(0.3);
      } 
      carriageParked = true;
    }
  }


//          CCCC    A    RRRR    GGGG   OOO       IIIII  N   N  TTTTT    A    K   K  EEEEE                                                                
//         C       A A   R   R  G      O   O        I    NN  N    T     A A   K  K   E                                                 
//         C      AAAAA  RRRR   G  GG  O   O        I    N N N    T    AAAAA  KKK    EEE                                                        
//         C      A   A  R  R   G   G  O   O        I    N  NN    T    A   A  K  K   E                                                 
//          CCCC  A   A  R   R   GGG    OOO       IIIII  N   N    T    A   A  K   K  EEEEE                                                       

  if ((encoderReading < 140) and cargoIntakeActive) {  //Elevator is at good position and intake is true
    rollerMotor.Set(rollerSpeed); //                Manual & Auto rollerMotor starter.
  }
  else if (driverController.GetRawButton(3) and not cargoIntakeActive){//Driver X pressed & not in intake
    rollerMotor.Set(-rollerSpeed); //Manual control to inverse(?) roller 
  }
  else if (driverController.GetRawButton(4) and not cargoIntakeActive){//Y pressed & not in intake
    rollerMotor.Set(rollerSpeed); //Manual control to power roller
  }
 else {
    rollerMotor.Set(0.0); //              Turn roller off if elevator goes up.
  }

  if (elevatorOverridden){ //                                            >>> MANUAL INTAKE
    if (driverController.GetRawButton(6) and iRSensor.Get()) { //Driver Right bumper, no ball present
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse); //Intake position
      cargoIntakeActive = true; 
    }
    else if (driverController.GetRawAxis(3) > 0.8) { //Driver right (?) trigger
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse); //Intake position, to shoot or cancel
      rollerMotor.Set(0.0); //Roller off
      shooterWheel.Set(frc::Relay::Value::kOff); //Shooter wheel off
      cargoIntakeActive = false; //Shoot or cancel. Either one means we are no longer in intake
    }
    else if (cargoIntakeActive and not iRSensor.Get()) { //In intake and ball is present
      cargoIntakeValve.Set(frc::DoubleSolenoid::kForward); //GRAB THE BALL
      rollerMotor.Set(0.0); //Roller off
      shooterWheel.Set(frc::Relay::Value::kReverse); //Shooter wheel on
      counter = 0;
      cargoIntakeActive = false; //Ball caught/tried to catch. Not in intake.
    }
  }
  else { //                                                                >>> AUTO INTAKE
    if (cargoPosition //If in a cargo position, holding, & right trigger squeezed
              and carriageParked 
              and (driverController.GetRawAxis(3) > 0.8)) {
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse); //   Shoot Cargo.
    }
    else if (driverController.GetRawAxis(3) > 0.8) { // Cancel Cargo Intake.
      cargoIntakeActive = false; 
      rollerMotor.Set(0.0);
      shooterWheel.Set(frc::Relay::Value::kOff);
    } 
    else if (driverController.GetRawButton(6) //         Start Cargo Intake.
              and iRSensor.Get()) { //           ONLY when no cargo present.
      cargoIntakeActive = true;
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse);
    }
    else if (cargoIntakeActive and not iRSensor.Get()) { //Intake mode & ball present
      cargoIntakeValve.Set(frc::DoubleSolenoid::kForward); //Grab ball yesssss
      rollerMotor.Set(0); //Off with the roller
      shooterWheel.Set(frc::Relay::Value::kReverse); //Setup shooter wheel
      counter = 0;
      cargoIntakeActive = false; //Grabbing ball, so not in intake
    }
  }

  counter = counter+1; //       After about a second, turn-off shooterWheel. 
  if (counter >= 40) { //50 counter values equals one second
    shooterWheel.Set(frc::Relay::Value::kOff);
    counter = 0;
  }

//                         H   H    A    TTTTT   CCCC  H  H       CCCC  L        A   W   W                                  
//                         H   H   A A     T    C      H  H      C      L       A A  W   W                           
//                         HHHHH  AAAAA    T    C      HHHH      C      L      AAAAA W W W                                
//                         H   H  A   A    T    C      H  H      C      L      A   A WW WW                            
//                         H   H  A   A    T     CCCC  H  H       CCCC  LLLLL  A   A W   W                                      

  if (driverController.GetRawButton(5)) { // Driver left bumper pressed
    hatchIntakeValve.Set(frc::DoubleSolenoid::kForward); //Open claw/Grab hatch
  }
  else if (driverController.GetRawAxis(2) > 0.8) { //Driver left trigger
    hatchIntakeValve.Set(frc::DoubleSolenoid::kReverse); //Close claw/Release hatch
  } 

//                    DDDD   RRRR   IIIII  V   V  EEEEE  TTTTT  RRRR     A    IIIII  N   N                                                       
//                    D   D  R   R    I    V   V  E        T    R   R   A A     I    NN  N                                 
//                    D   D  RRRR     I    V   V  EEE      T    RRRR   AAAAA    I    N N N                                        
//                    D   D  R  R     I     V V   E        T    R  R   A   A    I    N  NN                                 
//                    DDDD   R   R  IIIII    V    EEEEE    T    R   R  A   A  IIIII  N   N                                                 

  dCLeftYAxis = driverController.GetRawAxis(1);  //    Read driver gamepad joysticks.
  dCRightYAxis = -driverController.GetRawAxis(5);



  shifter = driverController.GetPOV();  //          Decide shift mode. (default true)
  /*if (shifter == 0) {
    manualShift = false;
  }
  else if (shifter == 180) {
    manualShift = true;
  } */ 

  if    ((((dCLeftYAxis < -0.95) and (dCRightYAxis > 0.95))  //    Go to AUTO HIGH Gear.
      or   ((dCLeftYAxis > 0.95) and (dCRightYAxis < -0.95)))
      and (not manualShift)) {
    shifterValve.Set(false);
  }
  else if (not manualShift) {  //                                 Go to AUTO LOW Gear.
    shifterValve.Set(true);
  }
  else if (driverController.GetRawButton(1)) {  //              Go to MANUAL LOW Gear.
    shifterValve.Set(true);
  }
  else if (driverController.GetRawButton(2)) {  //             Go to MANUAL HIGH Gear.
    shifterValve.Set(false);
  }

  if (driverController.GetRawButton(7)) { //       Change FRONT to HATCH perspective.
    hatchIsFront = true;
  }

if (driverController.GetPOV() == 0) { //Enable pixy mode
    pixyMode = true;
    std::cout << "Pixy on" << std::endl;
  } else if (driverController.GetPOV() == 180) {
    pixyMode = false;
    std::cout << "Pixy off" << std::endl;
  }

  
if ((dCRightYAxis > 0.2) and pixyMode and not hatchIsFront) { //Do the pixy sheet
  dCRightYAxis = dCRightYAxis*0.5;
  std::cout << pixyLine0.m_x0 << std::endl;
  if (pixyLine0.m_x0 == 128) {
    std::cout << "Pixy has lost the line" << std::endl;
    dCRightYAxis = dCRightYAxis*2;
  } else if ((pixyLine0.m_x0 < centerX) and (pixyLine0.m_x0 > 0)) {
    dCLeftYAxis = -0.4;
    dCRightYAxis = 0.21;
  } else if ((pixyLine0.m_x0 > centerX) and (pixyLine0.m_x0 > 0)) {
    dCLeftYAxis = -0.21;
    dCRightYAxis = 0.4;
  } else {
    dCLeftYAxis = -dCRightYAxis;
  }
}

  else if (driverController.GetRawButton(8)) {  // Change FRONT to CARGO perspective.
    hatchIsFront = false;
  }

  if (hatchIsFront) {  //   Swap driver gamepad joystick inputs for HATCH perspecive.
    tempLeftAxis = dCLeftYAxis;
    dCLeftYAxis = dCRightYAxis;
    dCRightYAxis = tempLeftAxis;
  }
  
  if ((dCLeftYAxis > 0) and (dCRightYAxis < 0)) {
    dCLeftYAxis = dCLeftYAxis * 0.75;
    dCRightYAxis = dCRightYAxis * 0.75;
  }

  if ((dCLeftYAxis > 0.2) or (dCLeftYAxis < -0.2)) {  //             Run LEFT Motors.
    leftTopMotor.Set(dCLeftYAxis);
    leftMidMotor.Set(dCLeftYAxis);
    leftLowMotor.Set(dCLeftYAxis);
   } else {
    leftTopMotor.Set(0);
    leftMidMotor.Set(0);
    leftLowMotor.Set(0);
   } 
  
  if ((dCRightYAxis > 0.2) or (dCRightYAxis < -0.2)) {  //          Run RIGHT Motors.
    rightTopMotor.Set(dCRightYAxis);
    rightMidMotor.Set(dCRightYAxis);
    rightLowMotor.Set(dCRightYAxis);
  } else {
    rightTopMotor.Set(0);
    rightMidMotor.Set(0);
    rightLowMotor.Set(0);
  }

//                                                  OOO   U   U  TTTTT PPPP   U   U  TTTTT                                                                      
//                                                 O   O  U   U    T   P   P  U   U    T                                                       
//                                                 O   O  U   U    T   PPPP   U   U    T                                                        
//                                                 O   O  U   U    T   P      U   U    T                                                     
//                                                  OOO    UUU     T   P       UUU     T                                                        
std::cout << encoderReading << std::endl;

 }


//======================================================================================================================================================  

//                                                                    TTTTTTT  EEEEEEE   SSSSS   TTTTTTT                                                                                                          
//                                                                       T     E        S     S     T                                                                           
//                                                                       T     E        S           T                                                                                
//                                                                       T     EEE       SSSSS      T                                                                                        
//                                                                       T     E              S     T                                                                                   
//                                                                       T     E        S     S     T                                                                                   
//                                                                       T     EEEEEEE   SSSSS      T                                                                                                         



//                                                                    {*** TELE-OP PERIODIC ***}
void Robot::TestPeriodic() {}



//======================================================================================================================================================  

//                                                                   MM    MM     A     IIIIIII  N     N                                                                                                          
//                                                                   M M  M M    A A       I     NN    N                                                                          
//                                                                   M  MM  M   A   A      I     N N   N                                                                               
//                                                                   M      M  AAAAAAA     I     N  N  N                                                                                       
//                                                                   M      M  A     A     I     N   N N                                                                                  
//                                                                   M      M  A     A     I     N    NN                                                                                  
//                                                                   M      M  A     A  IIIIIII  N     N                                                                                                       



//                                                                                {*** MAIN ***}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif



//======================================================================================================================================================  
