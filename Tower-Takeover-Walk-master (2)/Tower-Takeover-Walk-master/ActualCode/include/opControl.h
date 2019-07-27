#include "Auton.h"

bool switchMode = false;
bool trayMode = false;
bool hit = false; 
bool tapped = false; 

int toggle() {
  static bool lowTower = false;

  while (true) {

    if (Controller1.ButtonL2.pressing() &&  hit == false) {
      
      hit = 1; 
      vex::task::stop(moveArmHigh);
      vex::task::stop(moveArmDownHigh);

      lowTower = !lowTower;
      switchMode = !switchMode; 

      if (lowTower == true) {
       vex::task::stop(moveArmDown);
       vex::task h = vex::task(moveArm);
      }

      else {
        //vex::task::stop(barFoward);
        vex::task::stop(moveArm);
        vex::task n = vex::task(moveArmDown);
      }
    }

    else if (!Controller1.ButtonL2.pressing() && hit == 1 ){
     hit = 0 ; 
    }
    task::sleep(10);
  }
}

int toggleMid() {
  static bool highTower = false;
  while (true) {

    if (Controller1.ButtonL1.pressing() &&  tapped  == false) {
      
      tapped = 1; 
      vex::task::stop(moveArm);
      vex::task::stop(moveArmDown);
      
      trayMode = !trayMode; 
      highTower = !highTower;

      if (highTower == true) {
      //k = true;             
      vex::task::stop(moveArmDownHigh);
      vex::task w = vex::task(moveArmHigh);
      }

      else {
        vex::task::stop(moveArmHigh);
        vex::task d = vex::task(moveArmDownHigh);
      }
    }

    else if (!Controller1.ButtonL1.pressing() && tapped == 1 ){
     tapped = 0; 
    }
    task::sleep(10);
  }
}

void driverControl() {

  vex::task::stop(myStopIntake);
  vex::task::stop(myIntake);

  /*-----------------------------------------------------------------------------*/
  /** @brief     Stop tasks still running */
  /*-----------------------------------------------------------------------------*/

  // vex::task::stop(barBackwards(0,10));

  /*-----------------------------------------------------------------------------*/
  /** @brief     Base Control */
  /*-----------------------------------------------------------------------------*/

  LeftMotor.setReversed(true);
  RightMotor.setReversed(false);
  LeftmotorBack.setReversed(true);
  RightMotorBack.setReversed(false);

  LeftMotor.spin(vex::directionType::fwd, (Controller1.Axis2.position() + Controller1.Axis1.position())/1.2, vex::velocityUnits::pct); 
  LeftmotorBack.spin(vex::directionType::fwd, (Controller1.Axis2.position() + Controller1.Axis1.position())/1.2, vex::velocityUnits::pct); 
  RightMotor.spin(vex::directionType::fwd, (Controller1.Axis2.position() - Controller1.Axis1.position())/1.2, vex::velocityUnits::pct); 
  RightMotorBack.spin(vex::directionType::fwd, (Controller1.Axis2.position() - Controller1.Axis1.position())/1.2, vex::velocityUnits::pct); 
  /*-----------------------------------------------------------------------------*/
  /** @brief     Intake Spin Direction Control*/
  /*-----------------------------------------------------------------------------*/

  if (Controller1.ButtonR1.pressing() && !trayMode && !switchMode) {

    Intake.spin(vex::directionType::fwd, intakeSpeedPCT,
                vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, intakeSpeedPCT,
                    vex::velocityUnits::pct);
  } else if (Controller1.ButtonR2.pressing() && !trayMode && !switchMode) {

    Intake.spin(vex::directionType::rev, intakeSpeedPCT,
                vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::rev, intakeSpeedPCT,
                    vex::velocityUnits::pct);
  } 
    else if (Controller1.ButtonR1.pressing() && trayMode && !switchMode) {

    Intake.spin(vex::directionType::fwd, intakeHalfSpeedPCT,
                vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, intakeHalfSpeedPCT,
                    vex::velocityUnits::pct);
  } else if (Controller1.ButtonR2.pressing() && trayMode && !switchMode) {

    Intake.spin(vex::directionType::rev, intakeHalfSpeedPCT,
                vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::rev, intakeHalfSpeedPCT,
                    vex::velocityUnits::pct);
  }
   else if (Controller1.ButtonR1.pressing() && !trayMode && switchMode) {

    Intake.spin(vex::directionType::fwd, intakeHalfSpeedPCT,
                vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, intakeHalfSpeedPCT,
                    vex::velocityUnits::pct);
  } else if (Controller1.ButtonR2.pressing() && !trayMode && switchMode) {

    Intake.spin(vex::directionType::rev, intakeHalfSpeedPCT,
                vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::rev, intakeHalfSpeedPCT,
                    vex::velocityUnits::pct);
  }
  else {
    Intake.stop(vex::brakeType::brake);
    IntakeLeft.stop(vex::brakeType::brake);
  }

  /*-----------------------------------------------------------------------------*/
  /** @brief     Switch Functions Between 2bar control and 4bar control */
  /*-----------------------------------------------------------------------------*/
   if(Controller1.Axis3.position() > 10 || Controller1.Axis3.position() < -10){
   Bar.spin(vex::directionType::fwd, (Controller1.Axis3.position()),vex::velocityUnits::pct);
  }
   else if(switchMode == 1 || trayMode == 1) {
    Bar.setVelocity(100, vex::velocityUnits::pct);
    Bar.rotateTo(0.29, vex::rotationUnits::rev); 
  }
  
  else{
    Bar.stop(); 
  }
  /*-----------------------------------------------------------------------------*/
  /** @brief     Macros to help the driver */
  /*-----------------------------------------------------------------------------*/

  vex::task p = vex::task(toggle);
  vex::task b = vex::task(toggleMid);

  /*-----------------------------------------------------------------------------*/
  /** @brief    Sleep the task for a short amount of time to prevent
                        wasted resources.*/
  /*-----------------------------------------------------------------------------*/

  vex::task::sleep(20);
}
/*left top shoulder - intake/outake
right top should - preset for scoring high / descoring high
right bot should - preset for scoring low / descoring low
*/