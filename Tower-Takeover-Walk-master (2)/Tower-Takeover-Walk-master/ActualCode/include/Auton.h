#include "C:/Program Files (x86)/VEX Robotics/VEXcode/sdk/vexv5/include/vex_task.h"
using namespace vex;
#include "Vision.h"
#include "testRerun.h"
#include "vex.h"
#include <algorithm>
#include <cmath>

/*-----------------------------------------------------------------------------*/
/** @brief     Motor / sensor setup */
/*-----------------------------------------------------------------------------*/
vex::motor LeftMotor = vex::motor(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor =
    vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor LeftmotorBack = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor RightMotorBack =
    vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor Intake = vex::motor(vex::PORT3, vex::gearSetting::ratio18_1);
vex::motor IntakeLeft =
    vex::motor(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor Bar = vex::motor(vex::PORT5, vex::gearSetting::ratio36_1, true);
vex::motor towerArm = vex::motor(vex::PORT4, vex::gearSetting::ratio36_1, true);

/*-----------------------------------------------------------------------------*/
/** @brief    Array for color signatures */
/*-----------------------------------------------------------------------------*/
vex::vision::signature mysigs[4] = {
    O,
    O,
    G,
    P,
};

/*-----------------------------------------------------------------------------*/
/** @brief     Variable Defentions */
/*-----------------------------------------------------------------------------*/

bool linedUp = false;
bool reached = false;
bool trayUp = false;
int stackingSpeed = 100;

/*-----------------------------------------------------------------------------*/
/** @brief     Rainbow Logo loop to make the brian presntable while in match */
/*-----------------------------------------------------------------------------*/

int rainbowlogo() {

  while (1 == 1) {
    Brain.Screen.drawImageFromFile("BlueGroup1.png", 100,
                                   0); // each picture is 270 by 258
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup2.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup3.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup4.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup5.png", 100, 0);
    task::sleep(1000);
    Brain.Screen.drawImageFromFile("BlueGroup6.png", 100, 0);
    task::sleep(1000);

    // vex::Gif gif("logoDisinigrate.gif", 120, 0);
  }
  task::sleep(1000);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Necessary Variable Defentions for Motion Profiling */
/*-----------------------------------------------------------------------------*/
const double minimum_velocity = 15.0; // minimum velocity is necessary

double increasing_speed(double starting_point, double current_position) {
  static const double acceleration_constant = 50.0;
  return acceleration_constant * std::abs(current_position - starting_point) +
         minimum_velocity;
}

double decreasing_speed(double ending_point, double current_position) {
  static const double deceleration_constant = 30.0;
  return deceleration_constant * std::abs(current_position - ending_point) +
         minimum_velocity;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Forward / backward functions with motion pofiling */
/*-----------------------------------------------------------------------------*/

void moveForward(double distanceIn, double maxVelocity) {

  static const double circumference = 3.14159 * 3.25;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = distanceIn / circumference;

  LeftMotor.setReversed(false);
  LeftmotorBack.setReversed(false);
  RightMotor.setReversed(true);
  RightMotorBack.setReversed(true);
  RightMotor.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  RightMotorBack.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  LeftMotor.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  LeftmotorBack.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  double leftStartPoint = LeftMotor.rotation(rotationUnits::rev);
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = LeftmotorBack.rotation(rotationUnits::rev);
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = RightMotor.rotation(rotationUnits::rev);
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = RightMotorBack.rotation(rotationUnits::rev);
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  while (
      (direction * (RightMotor.rotation(rotationUnits::rev) - rightStartPoint) <
       direction * wheelRevs) ||
      (direction * (LeftMotor.rotation(rotationUnits::rev) - leftStartPoint) <
       direction * wheelRevs) ||
      (direction *
           (LeftmotorBack.rotation(rotationUnits::rev) - leftStartPoint1) <
       direction * wheelRevs) ||
      (direction *
           (RightMotorBack.rotation(rotationUnits::rev) - rightStartPoint1) <
       direction * wheelRevs)) {

    if (direction * (LeftMotor.rotation(rotationUnits::rev) - leftStartPoint) <
        direction * wheelRevs) {
      LeftMotor.setVelocity(
          direction *
              std::min(
                  maxVelocity,
                  std::min(
                      increasing_speed(leftStartPoint,
                                       LeftMotor.rotation(rotationUnits::rev)),
                      decreasing_speed(leftEndPoint, LeftMotor.rotation(
                                                         rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      LeftMotor.stop(hold);
    }
    
    if (direction *
            (RightMotor.rotation(rotationUnits::rev) - rightStartPoint) <
        direction * wheelRevs) {
      RightMotor.setVelocity(
          direction *
              std::min(maxVelocity,
                       std::min(increasing_speed(
                                    rightStartPoint,
                                    RightMotor.rotation(rotationUnits::rev)),
                                decreasing_speed(
                                    rightEndPoint,
                                    RightMotor.rotation(rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      RightMotor.stop(hold);
    }

    if (direction *
            (LeftmotorBack.rotation(rotationUnits::rev) - leftStartPoint1) <
        direction * wheelRevs) {
      LeftmotorBack.setVelocity(
          direction *
              std::min(maxVelocity,
                       std::min(increasing_speed(
                                    leftStartPoint1,
                                    LeftmotorBack.rotation(rotationUnits::rev)),
                                decreasing_speed(leftEndPoint1,
                                                 LeftmotorBack.rotation(
                                                     rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      LeftmotorBack.stop(hold);
    }

    if (direction *
            (RightMotorBack.rotation(rotationUnits::rev) - rightStartPoint1) <
        direction * wheelRevs) {
      RightMotorBack.setVelocity(
          direction *
              std::min(maxVelocity,
                       std::min(increasing_speed(rightStartPoint1,
                                                 RightMotorBack.rotation(
                                                     rotationUnits::rev)),
                                decreasing_speed(rightEndPoint1,
                                                 RightMotorBack.rotation(
                                                     rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      RightMotorBack.stop(hold);
    }
    task::sleep(10);
  }
  RightMotor.stop(hold);
  RightMotorBack.stop(hold);
  LeftMotor.stop(hold);
  LeftmotorBack.stop(hold);
}

void moveBackward(double distanceIn, double maxVelocity) {

  static const double circumference = 3.14159 * 3.25;
  if (distanceIn == 0)
    return;
  double direction = distanceIn > 0 ? 1.0 : -1.0;
  double wheelRevs = distanceIn / circumference;

  LeftMotor.setReversed(false);
  LeftmotorBack.setReversed(false);
  RightMotor.setReversed(true);
  RightMotorBack.setReversed(true);
  RightMotor.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  RightMotorBack.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  LeftMotor.spin(fwd, direction * minimum_velocity, velocityUnits::pct);
  LeftmotorBack.spin(fwd, direction * minimum_velocity, velocityUnits::pct);

  double leftStartPoint = LeftMotor.rotation(rotationUnits::rev);
  double leftEndPoint = leftStartPoint + wheelRevs;
  double leftStartPoint1 = LeftmotorBack.rotation(rotationUnits::rev);
  double leftEndPoint1 = leftStartPoint1 + wheelRevs;
  double rightStartPoint = RightMotor.rotation(rotationUnits::rev);
  double rightEndPoint = rightStartPoint + wheelRevs;
  double rightStartPoint1 = RightMotorBack.rotation(rotationUnits::rev);
  double rightEndPoint1 = rightStartPoint1 + wheelRevs;

  while (
      (direction * (RightMotor.rotation(rotationUnits::rev) - rightStartPoint) <
       direction * wheelRevs) ||
      (direction * (LeftMotor.rotation(rotationUnits::rev) - leftStartPoint) <
       direction * wheelRevs) ||
      (direction *
           (LeftmotorBack.rotation(rotationUnits::rev) - leftStartPoint1) <
       direction * wheelRevs) ||
      (direction *
           (RightMotorBack.rotation(rotationUnits::rev) - rightStartPoint1) <
       direction * wheelRevs)) {
    if (direction *
            (RightMotor.rotation(rotationUnits::rev) - rightStartPoint) <
        direction * wheelRevs) {
      RightMotor.setVelocity(
          direction *
              std::min(maxVelocity,
                       std::min(increasing_speed(
                                    rightStartPoint,
                                    RightMotor.rotation(rotationUnits::rev)),
                                decreasing_speed(
                                    rightEndPoint,
                                    RightMotor.rotation(rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      RightMotor.stop(hold);
    }
    if (direction *
            (RightMotorBack.rotation(rotationUnits::rev) - rightStartPoint1) <
        direction * wheelRevs) {
      RightMotorBack.setVelocity(
          direction *
              std::min(maxVelocity,
                       std::min(increasing_speed(rightStartPoint1,
                                                 RightMotorBack.rotation(
                                                     rotationUnits::rev)),
                                decreasing_speed(rightEndPoint1,
                                                 RightMotorBack.rotation(
                                                     rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      RightMotorBack.stop(hold);
    }

    if (direction * (LeftMotor.rotation(rotationUnits::rev) - leftStartPoint) <
        direction * wheelRevs) {
      LeftMotor.setVelocity(
          direction *
              std::min(
                  maxVelocity,
                  std::min(
                      increasing_speed(leftStartPoint,
                                       LeftMotor.rotation(rotationUnits::rev)),
                      decreasing_speed(leftEndPoint, LeftMotor.rotation(
                                                         rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      LeftMotor.stop(hold);
    }

    if (direction *
            (LeftmotorBack.rotation(rotationUnits::rev) - leftStartPoint1) <
        direction * wheelRevs) {
      LeftmotorBack.setVelocity(
          direction *
              std::min(maxVelocity,
                       std::min(increasing_speed(
                                    leftStartPoint1,
                                    LeftmotorBack.rotation(rotationUnits::rev)),
                                decreasing_speed(leftEndPoint1,
                                                 LeftmotorBack.rotation(
                                                     rotationUnits::rev)))),
          vex::velocityUnits::pct);
    } else {
      LeftmotorBack.stop(hold);
    }
  }
  LeftMotor.stop(hold);
  LeftmotorBack.stop(hold);
  RightMotor.stop(hold);
  RightMotorBack.stop(hold);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Left / Right functions with motion pofiling */
/*-----------------------------------------------------------------------------*/
// R - 1164,1121.6, 1375 = 1220.2
// R1 - 1160.8, 1129.2, 1376 = 1220
// L - 1155.6, 1172, 1078.4 = 1135.53
// L1 - 1167.6, 1121.6, 1052.8 = 1114

void turnLeft(double degree, int velocity) {
  double ticksPerTurn = 1172.4325;
  double ticks = degree * ticksPerTurn / 360;
  double degreesToRotate = ticks;

  LeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftMotor.setReversed(true);
  RightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setReversed(true);
  RightMotorBack.setVelocity(velocity, vex::velocityUnits::pct);

  LeftMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  LeftmotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
}

void turnRight(double degree, int velocity) {
  double ticksPerTurn = 1172.4325;
  double ticks = degree * ticksPerTurn / 360;
  double degreesToRotate = ticks;

  LeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftMotor.setReversed(false);
  RightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotor.setReversed(false);
  LeftmotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setReversed(false);
  RightMotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotorBack.setReversed(false);

  LeftMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  LeftmotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Simple left / right functions with out motion pofiling */
/*-----------------------------------------------------------------------------*/

void Right(int velocity) {

  LeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotorBack.setVelocity(velocity, vex::velocityUnits::pct);

  LeftMotor.spin(vex::directionType::fwd);
  RightMotor.spin(vex::directionType::rev);
  LeftmotorBack.spin(vex::directionType::fwd);
  RightMotorBack.spin(vex::directionType::rev);
}
void Left(int velocity) {

  LeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotorBack.setVelocity(velocity, vex::velocityUnits::pct);

  LeftMotor.spin(vex::directionType::rev);
  RightMotor.spin(vex::directionType::fwd);
  LeftmotorBack.spin(vex::directionType::rev);
  RightMotorBack.spin(vex::directionType::fwd);
}

/*-----------------------------------------------------------------------------*/
/** @brief     Simple forward / backward functions with out motion pofiling */
/*-----------------------------------------------------------------------------*/

void moveForwardSimple(float distance, int velocity) {
  double wheelDiameterCM = 3.25;
  double travelTargetCM =
      distance; // this is the distance it goes which is set as a variable
  double circumfrence = wheelDiameterCM * 3.141592;
  double degreesToRotate = (360 * travelTargetCM) / circumfrence;

  LeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftMotor.setReversed(false);
  RightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotor.setReversed(true);
  LeftmotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setReversed(false);
  RightMotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotorBack.setReversed(true);

  LeftMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  LeftmotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
}

void moveBackwardSimple(int distance, int velocity) {
  double wheelDiameterCM = 3.25;
  double travelTargetCM =
      distance; // this is the distance it goes which is set as a variable
  double circumfrence = wheelDiameterCM * 3.141592;
  double degreesToRotate = (360 * travelTargetCM) / circumfrence;

  LeftMotor.setVelocity(velocity, vex::velocityUnits::pct);
  LeftMotor.setReversed(true);
  RightMotor.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotor.setReversed(false);
  LeftmotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  LeftmotorBack.setReversed(true);
  RightMotorBack.setVelocity(velocity, vex::velocityUnits::pct);
  RightMotorBack.setReversed(false);

  LeftMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotor.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  LeftmotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, false);
  RightMotorBack.rotateFor(degreesToRotate, vex::rotationUnits::deg, true);
}
/*-----------------------------------------------------------------------------*/
/** @brief     Functions used to bring 4bar / tilter to certain locations */
/*-----------------------------------------------------------------------------*/
void move4bf(int speed, float rev) {
  Bar.setVelocity(speed, vex::velocityUnits::pct);
  Bar.rotateTo(rev, vex::rotationUnits::rev);
}

void move4bb(int speed, float rev) {
  Bar.setVelocity(speed, vex::velocityUnits::pct);
  Bar.rotateTo(rev, vex::rotationUnits::rev);
}

void move4bForwardSimple(int speed) {
  Bar.setVelocity(speed, vex::velocityUnits::pct);
  Bar.spin(vex::directionType::rev);
}

void move4bBackSimple(int speed) {
  Bar.setVelocity(speed, vex::velocityUnits::pct);
  Bar.spin(vex::directionType::fwd);
}

int pivotCounter = 0;
int pivotTask() {
  while (1) {
    if (Controller1.ButtonUp.pressing()) {
      while (Controller1.ButtonUp.pressing()) {
        task::sleep(10);
      }
      switch (pivotCounter) {
      case 0: // Tilted
        while (Bar.rotation(rev) > 3) {
          Bar.spin(fwd, 100, pct);
        }
        while (Bar.rotation(rev) > 2) {
          Bar.spin(fwd, 30, pct);
        }
        Bar.stop(brakeType::coast);
        pivotCounter++;
        break;
      case 1: // Vertical
        while (Bar.rotation(rev) < 1) {
          Bar.spin(directionType::rev, 100, pct);
        }
        Bar.stop(brakeType::coast);
        pivotCounter = 0;
        break;
      }
    }
    task::sleep(50);
  }
  return (0);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Task to have the Intake spining constently */
/*-----------------------------------------------------------------------------*/

int myIntake() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  }
}

int myStopIntake() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  }
}

int myIntake1() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  }
}

int myStopIntake1() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  }
}

int myIntake2() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  }
}

int myStopIntake2() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  }
}

int myIntake3() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  }
}

int myIntake4() {
  while (1) {
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    IntakeLeft.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief     task used to bring the tilting mech up */
/*-----------------------------------------------------------------------------*/
bool deploy = true;

int barFoward() {
  while (1 == 1) {
    Bar.setVelocity(100, vex::velocityUnits::pct);
    Bar.rotateTo(4.5, vex::rotationUnits::rev);
  }
}
/*-----------------------------------------------------------------------------*/
/** @brief      task used to bring the titing mech down */
/*-----------------------------------------------------------------------------*/
bool down = true;

int barBackwards() {
  while (down == 1) {
    Bar.setVelocity(100, vex::velocityUnits::pct);
    Bar.rotateFor(1, vex::rotationUnits::rev);
    down = false;
  }

  while (down == 0) {
    Bar.stop();
  }
  return 1;
}
/*-----------------------------------------------------------------------------*/
/** @brief      Turn Bot towards object */
/*-----------------------------------------------------------------------------*/
void ObjectLooker(int sigNumber, int speed) {
  VisionCamera.setBrightness(18);
  VisionCamera.setSignature(mysigs[sigNumber]);
  int centerFOV = 158;
  while (!linedUp) {
    VisionCamera.takeSnapshot(mysigs[sigNumber]);
    if (VisionCamera.objectCount > 0) {
      if (VisionCamera.largestObject.centerX < centerFOV - 3) {
        Left(speed);
      }
      if (centerFOV - 100 < VisionCamera.largestObject.centerX >
          centerFOV + 3) {
        Right(speed);
      }
    } else {
      linedUp = true;
      LeftMotor.stop(); // Stop the left motor.
      RightMotor.stop();
      LeftmotorBack.stop(); // Stop the left motor.
      RightMotorBack.stop();
    }
  }
}
/*-----------------------------------------------------------------------------*/
/** @brief      Go toward set color */
/*-----------------------------------------------------------------------------*/
void goTo(int sigNumber, int velocity) {
  VisionCamera.setBrightness(18);
  VisionCamera.setSignature(mysigs[sigNumber]);

  while (!reached) {
    VisionCamera.takeSnapshot(mysigs[sigNumber]);
    if (VisionCamera.objectCount > 0) {
      if (VisionCamera.largestObject.height < 58 &&
          VisionCamera.largestObject.height > 4) {
        moveForwardSimple(100, velocity);
      } else {
        reached = false;
        LeftMotor.stop(); // Stop the left motor.
        RightMotor.stop();
        LeftmotorBack.stop(); // Stop the left motor.
        RightMotorBack.stop();
      }
    }
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      IntakeArm Functions */
/*-----------------------------------------------------------------------------*/

int moveArm() {
  double kp = 0.05;
  while (1 == 1) {
    towerArm.setVelocity(80, vex::velocityUnits::pct);
    towerArm.rotateTo(1, vex::rotationUnits::rev);
    double error = (1 - towerArm.rotation(rev)) * kp;
    towerArm.setVelocity(50, vex::velocityUnits::pct);
    if (error > 0) {
      towerArm.spin(fwd);
    }

    else if (error < 0) {
      towerArm.spin(directionType::rev);
    }

    else {
      towerArm.stop(brakeType::hold);
    }
  }
  return 1;
}

int moveArmDown() {
  while (1 == 1) {
    towerArm.setVelocity(100, vex::velocityUnits::pct);
    towerArm.rotateTo(0, vex::rotationUnits::rev);
    towerArm.stop(hold);
  }
  return 1;
}

int moveArmHigh() {
  double kp = 0.05;
  while (1 == 1) {
    towerArm.setVelocity(100, vex::velocityUnits::pct);
    towerArm.rotateTo(1.4, vex::rotationUnits::rev);
    double error = (1.4 - towerArm.rotation(rev)) * kp;
    towerArm.setVelocity(50, vex::velocityUnits::pct);
    if (error > 0) {
      towerArm.spin(fwd);
    }

    else if (error < 0) {
      towerArm.spin(directionType::rev);
    }

    else {
      towerArm.stop(brakeType::hold);
    }
  }
  return 1;
}
int moveArmDownHigh() {
  while (1 == 1) {
    towerArm.setVelocity(100, vex::velocityUnits::pct);
    towerArm.rotateTo(0, vex::rotationUnits::rev);
    towerArm.stop();
  }
  return 1;
}

/*-----------------------------------------------------------------------------*/
/** @brief     Auton Selector */
/*-----------------------------------------------------------------------------*/

int autonomousSelection = -1;

typedef struct _button {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char *label;
} button;

// Button array definitions for each software button. The purpose of each button
// data structure is defined above.  The array size can be extended, so you can
// have as many buttons as you wish as long as it fits.

button buttons[] = {{30, 30, 60, 60, false, 0xE00000, 0x0000E0, "Ally"},
                    //{150, 30, 60, 60, false, 0x303030, 0xD0D0D0, ""},
                    //{270, 30, 60, 60, false, 0x303030, 0xF700FF, ""},
                    {390, 30, 60, 60, false, 0x303030, 0xDDDD00, "Back"},
                    {30, 150, 60, 60, false, 0x404040, 0xC0C0C0, "ReRun"},
                    // {150, 150, 60, 60, false, 0x404040, 0xC0C0C0, ""},
                    //{270, 150, 60, 60, false, 0x404040, 0xC0C0C0, ""},
                    {390, 150, 60, 60, false, 0x404040, 0xC0C0C0, "Skill"}};

// forward ref
void displayButtonControls(int index, bool pressed);

/*-----------------------------------------------------------------------------*/
/** @brief      Check if touch is inside button */
/*-----------------------------------------------------------------------------*/
int findButton(int16_t xpos, int16_t ypos) {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    button *pButton = &buttons[index];
    if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width))
      continue;

    if (ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height))
      continue;

    return (index);
  }
  return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Init button states */
/*-----------------------------------------------------------------------------*/
void initButtons() {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    buttons[index].state = false;
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been touched */
/*-----------------------------------------------------------------------------*/
void userTouchCallbackPressed() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    displayButtonControls(index, true);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been (un)touched */
/*-----------------------------------------------------------------------------*/
void userTouchCallbackReleased() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    // clear all buttons to false, ie. unselected
    //      initButtons();

    // now set this one as true
    if (buttons[index].state == true) {
      buttons[index].state = false;
    } else {
      buttons[index].state = true;
    }

    // save as auton selection
    autonomousSelection = index;

    displayButtonControls(index, false);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Draw all buttons */
/*-----------------------------------------------------------------------------*/
void displayButtonControls(int index, bool pressed) {
  vex::color c;
  Brain.Screen.setPenColor(vex::color(0xe0e0e0));

  for (int i = 0; i < sizeof(buttons) / sizeof(button); i++) {

    if (buttons[i].state)
      c = buttons[i].onColor;
    else
      c = buttons[i].offColor;

    Brain.Screen.setFillColor(c);

    // button fill
    if (i == index && pressed == true) {
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height, c);
    } else
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                                 buttons[i].width, buttons[i].height);

    // outline
    Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
                               buttons[i].width, buttons[i].height,
                               vex::color::transparent);

    // draw label
    if (buttons[i].label != NULL)
      Brain.Screen.printAt(buttons[i].xpos + 8,
                           buttons[i].ypos + buttons[i].height - 8,
                           buttons[i].label);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Reset Encoder Functions */
/*-----------------------------------------------------------------------------*/

void resetEncoders() {
  RightMotor.setRotation(0, rev);
  RightMotorBack.setRotation(0, rev);
  LeftMotor.setRotation(0, rev);
  LeftmotorBack.setRotation(0, rev);
}

void runRerun() {
  bool checkForPlaybackEnd = false;
  if (!recordedLeftMotor.isPlayback()) {
    recordedLeftMotor.disableRecordingOrPlayback();
    recordedLeftMotorBack.disableRecordingOrPlayback();
    recordedRightMotor.disableRecordingOrPlayback();
    recordedRightMotorBack.disableRecordingOrPlayback();
    recordedArmMotor.disableRecordingOrPlayback();
    recorderIntakeLeft.disableRecordingOrPlayback();
    recorderIntakeRight.disableRecordingOrPlayback();
    recorderTilter.disableRecordingOrPlayback();
    recordedLeftMotor.enablePlayback();
    recordedLeftMotorBack.enablePlayback();
    recordedRightMotor.enablePlayback();
    recordedRightMotorBack.enablePlayback();
    recordedArmMotor.enablePlayback();
    recorderIntakeLeft.enablePlayback();
    recorderIntakeRight.enablePlayback();
    recorderTilter.enablePlayback();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Playback is now running ");
    checkForPlaybackEnd = true;
  } else {
    recordedLeftMotor.disableRecordingOrPlayback();
    recordedLeftMotorBack.disableRecordingOrPlayback();
    recordedRightMotor.disableRecordingOrPlayback();
    recordedRightMotorBack.disableRecordingOrPlayback();
    recordedArmMotor.disableRecordingOrPlayback();
    recorderIntakeLeft.disableRecordingOrPlayback();
    recorderIntakeRight.disableRecordingOrPlayback();
    recorderTilter.disableRecordingOrPlayback();
    Controller1.Screen.clearLine(1);
    checkForPlaybackEnd = false;
  }
}
/*-----------------------------------------------------------------------------*/
/** @brief      Preauton Functions */
/*-----------------------------------------------------------------------------*/
void preAuton() {
  displayButtonControls(0, true);

  Brain.Screen.pressed(userTouchCallbackPressed);
  Brain.Screen.released(userTouchCallbackReleased);
  rainbowlogo();

  // make nice background
  /* Brain.Screen.setFillColor(vex::color(0x404040));
  Brain.Screen.setPenColor(vex::color(0x404040));
  Brain.Screen.drawRectangle(0, 0, 480, 120);
  Brain.Screen.setFillColor(vex::color(0x808080));
  Brain.Screen.setPenColor(vex::color(0x808080));
  Brain.Screen.drawRectangle(0, 120, 480, 120);*/
  resetEncoders();
}
/*-----------------------------------------------------------------------------*/
/** @brief      RedBack Auton */
/*-----------------------------------------------------------------------------*/
void auton() {
  /*bool allianceBlue = buttons[0].state;
  bool startTileFar = buttons[1].state;
  bool reRun = buttons[2].state;
  bool skillsRun = buttons[3].state; */

  /*if(!allianceBlue && !startTileFar && skillsRun && !reRun){
  vex::task t(myIntake);
  moveForward(65, 50);
  task::sleep(500);
  resetEncoders();
  moveForward(35, 50);
  task::sleep(200);
  turnRight(28, 30);
  moveForward(18.8, 80);
  vex::task::stop(t);
  resetEncoders();
  move4bf(100, 3.9);
  task::sleep(500);
  moveForward(-5, 30);
  }

  if(!allianceBlue && !startTileFar && skillsRun && reRun){
  runRerun();
  }

  if(!allianceBlue && startTileFar && !skillsRun){
  vex::task t(myIntake);
  moveForward(60, 60);
  turnLeft(30, 30);
  moveForwardSimple(2, 20);
  moveBackward(-60, 100);
  turnRight(180, 30);
  vex::task::stop(myIntake);
  moveForward(6, 50);
  move4bf(50, 1);
  moveBackwardSimple(20, 40);
  }

  if(!allianceBlue && !startTileFar && !skillsRun){
  vex::task t(myIntake);
  moveForward(60, 60);
  turnLeft(30, 30);
  moveForwardSimple(2, 20);
  moveBackward(-60, 100);
  turnRight(180, 30);
  vex::task::stop(myIntake);
  moveForward(6, 50);
  move4bf(50, 1);
  moveBackwardSimple(20, 40);
  }*/

  /*vex::task t(myIntake);
  moveForward(50.8, 60);
  task::sleep(400);
  turnLeft(90, 30);
  moveForward(20, 70);
  turnLeft(90, 30);
  moveForward(44.5, 90);
  turnRight(90, 30);
  moveForward(8.6, 30);   // red front auton
  vex::task r(myStopIntake);
  move4bf(30, 1);
  task::sleep(700);
  moveBackwardSimple(20, 40);*/

  /*vex::task t(myIntake);
  moveForward(50, 65);
  turnLeft(15, 30);
  moveForward(3, 40);
  task::sleep(300);
  moveBackward(-40, 100);
  turnRight(158, 30);
  vex::task r(myStopIntake);  //red Back auton
  moveForward(6, 70);
  move4bf(35, 0.934);
  task::sleep(1000);
  moveForward(-10, 50);*/

  vex::task t(myIntake);
  moveForward(50, 65);
  turnLeft(15, 30);
  moveForward(3.5, 40);
  task::sleep(300);
  moveBackward(-38.5, 90);
  turnRight(153, 30);
  vex::task r(myStopIntake);
  moveForward(4.8, 50);
  move4bf(38, 0.95);
  task::sleep(1000);
  moveForward(-10, 50);
  move4bb(100, 0);
  task::stop(t);
  task::stop(r);
  vex::task q(myIntake1);
  moveForward(-9, 50);
  turnLeft(142, 30);
  moveForwardSimple(-25.5, 60);
  RightMotor.setRotation(0, rev);
  RightMotorBack.setRotation(0, rev);
  LeftMotor.setRotation(0, rev);
  LeftmotorBack.setRotation(0, rev);
  task::sleep(300);
  moveForward(118, 95);
  turnRight(60, 30);
  moveForward(13, 80);
  task::stop(q);
  Intake.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
  task::sleep(200);
  vex::task z(myStopIntake1); /// Skills
  move4bf(25, 0.934);
  task::sleep(1000);
  moveForward(-26, 80);
  move4bb(80, 0);
  task::stop(q);
  task::stop(z);
  vex::task s(myIntake2);
  turnRight(125, 30);
  moveForwardSimple(-25, 80);
  RightMotor.setRotation(0, rev);
  RightMotorBack.setRotation(0, rev);
  LeftMotor.setRotation(0, rev);
  LeftmotorBack.setRotation(0, rev);
  moveForward(50, 80);
  task::stop(s);
  Intake.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
  task::sleep(700);
  Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  moveForward(-5, 50);
  move4bf(100, 0.33);
  towerArm.setVelocity(80, vex::velocityUnits::pct);
  towerArm.rotateTo(1, vex::rotationUnits::rev);
  Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  task::sleep(400);
  Intake.stop();
  IntakeLeft.stop();
  towerArm.rotateTo(0, vex::rotationUnits::rev);
  move4bb(100, 0);
  vex::task j(myIntake3);
  turnLeft(60, 30);
  moveForward(18.5, 70);
  task::sleep(500);
  moveForward(-15, 70);
  turnLeft(59, 30);
  moveForward(13, 70);
  task::stop(j);
  Intake.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
  task::sleep(700);
  Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  move4bf(100, 0.33);
  towerArm.rotateTo(1, vex::rotationUnits::rev);
  Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  task::sleep(400);
  Intake.stop();
  IntakeLeft.stop();
  towerArm.rotateTo(0, vex::rotationUnits::rev);
  move4bb(100, 0);
  vex::task a(myIntake4);
  turnLeft(119, 30);
  moveForward(43, 80);
  task::sleep(500);
  moveForward(-3, 60);
  task::stop(a);
  turnLeft(22, 30);
  Intake.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
  task::sleep(700);
  Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  move4bf(100, 0.30);
  towerArm.rotateTo(1.25, vex::rotationUnits::rev);
  Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  IntakeLeft.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);

  /*vex::task t(myIntake);
  moveForward(50, 65);
  turnRight(15, 30);
  moveForward(3, 40);
  task::sleep(300);
  moveBackward(-40, 100);
  turnLeft(158, 30);
  vex::task r(myStopIntake);  //Blue Back auton
  moveForward(6, 70);
  move4bf(35, 0.934);
  task::sleep(1000);
  moveForward(-10, 50);*/

  /*vex::task t(myIntake);
  moveForward(50.8, 60);
  task::sleep(400);
  turnRight(90, 30);  // Blue Front auton
  moveForward(20, 70);
  turnRight(90, 30);
  moveForward(44.5, 90);*/
}
