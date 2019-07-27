#include "vex.h"
#include <list>
#include <algorithm>
#include <fstream>
#include <iostream>
//#include "opControl.h"

vex::competition Competition;
vex::brain Brain;
vex::brain::sdcard sdcard;
vex::controller Controller1 = vex::controller();

int intakeSpeedPCT = 100;
int intakeHalfSpeedPCT = 25;

using namespace vex; 

class velocityRecordedMotor : public vex::motor {
    private:
    
    // This structure is used to store information on a single change of the
    // motor's velocity target. It encodes when the change was made, the
    // velocity (including direction), and what units the velocity was in
    struct dataslice {
        uint32_t timestamp;
        double normalizedVelocity;
        velocityUnits velocityType;
        
        dataslice() {
            
        }
        
        dataslice(uint32_t time, double vel, velocityUnits units) {
            timestamp = time;
            normalizedVelocity = vel;
            velocityType = units;
        }
    };
    
    // The data is stored in a doubly linked list
    std::list<dataslice> data;
    
    // This iterator is used to keep track of where in the recording the 
    // playback is currently at
    std::list<dataslice>::iterator playbackMarker;
    
    // This points to a string containing the save/load filename
    const char* logFileName;
    
    // These help keep track of timings. lastRecordingTime is used to make sure
    // that data isn't being saved too fast. operationBeginTime keeps track of
    // when playback or recording started to provide the proper offset.
    uint32_t lastRecordingTime;
    uint32_t operationBeginTime;
    
    // These help keep track of what the velocityRecordedMotor is doing and
    // should be doing.
    enum recordingState {none, recording, playback};
    recordingState myState = none;
    
    public:
    
    // The constructor for a velocityRecordedMotor. Delegates much of its task
    // to a constructor of the superclass, vex::motor.
    velocityRecordedMotor (int32_t index, vex::gearSetting gears, bool reversed, const char* logFileName)
      : vex::motor (index,gears,reversed) {
        this->logFileName = logFileName;
        lastRecordingTime = vex::timer::system();
    }
    
    void spin (vex::directionType dir, double velocity, vex::velocityUnits units) {
        uint32_t invocationTime = timer::system();
        if (myState == recording && invocationTime > lastRecordingTime) {
            vex::motor::spin(dir,velocity,units);
            if (dir == directionType::rev) velocity = -velocity;
            data.emplace_back(invocationTime - operationBeginTime,velocity,units);
            lastRecordingTime = invocationTime;
        } else if (myState == playback) {
            if (playbackMarker == data.end()) {
                stop(brakeType::hold);
            } else if (invocationTime >= operationBeginTime + playbackMarker->timestamp) {
                vex::motor::spin(vex::directionType::fwd, playbackMarker->normalizedVelocity, playbackMarker->velocityType);
                playbackMarker++;
            }
        } else if (myState == none) {
            vex::motor::spin(dir,velocity,units);
        }
    }
    
    void spin (directionType dir, double velocity, percentUnits units) {
        spin(dir,velocity,velocityUnits::pct);
    }
    
    void enableRecording() {
        lastRecordingTime = operationBeginTime = timer::system();
        data.clear();
        data.emplace_back(0,0,velocityUnits::pct);
        myState = recording;
    }
    
    bool isRecording() {
        return myState == recording;
    }
    
    bool isPlayback() {
        return myState == playback;
    }
    
    bool donePlayback() {
        return playbackMarker == data.end();
    }
    
    bool isIdle() {
        return myState == none;
    }
    
    void disableRecording() {
        if (myState == recording) {
            myState = none;
        }
    }
    
    void enablePlayback() {
        playbackMarker = data.begin();
        operationBeginTime = vex::timer::system();
        myState = playback;
    }
    
    void disablePlayback() {
        if (myState == playback) {
            myState = none;
        }
    }
    
    void disableRecordingOrPlayback () {
        myState = none;
    }
    
    void saveRecording() {
        disableRecording();
        std::ofstream outputFile(logFileName, std::ofstream::out | std::ofstream::trunc | std::ofstream::binary);
        for (std::list<dataslice>::iterator it = data.begin(); it != data.end(); ++it) {
            outputFile.write((char*)&(it->timestamp),sizeof(dataslice::timestamp));
            outputFile.write((char*)&(it->normalizedVelocity),sizeof(dataslice::normalizedVelocity));
            outputFile.write((char*)&(it->velocityType),sizeof(dataslice::velocityType));
            outputFile.flush();
        }
        outputFile.close();
    }
    
    void loadRecording () {
        disableRecording();
        data.clear();
        std::ifstream inputFile(logFileName, std::ifstream::in | std::ifstream::binary);
        while (!inputFile.eof()) {
            dataslice nextInput;
            inputFile.read((char*)&(nextInput.timestamp),sizeof(dataslice::timestamp));
            inputFile.read((char*)&(nextInput.normalizedVelocity),sizeof(dataslice::normalizedVelocity));
            inputFile.read((char*)&(nextInput.velocityType),sizeof(dataslice::velocityType));
            data.push_back(nextInput);
        }
    }
    
    ~velocityRecordedMotor() {
        
    }
};


velocityRecordedMotor recordedLeftMotor(vex::PORT20, vex::gearSetting::ratio18_1,false, "motorLeftVlog.dat");
velocityRecordedMotor recordedRightMotor(vex::PORT11, vex::gearSetting::ratio18_1, true, "motorRightVlog.dat");
velocityRecordedMotor recordedLeftMotorBack(vex::PORT19, vex::gearSetting::ratio18_1, false, "motorLeftBackVlog.dat");
velocityRecordedMotor recordedRightMotorBack(vex::PORT1, vex::gearSetting::ratio18_1, true, "motorRightBackVlog.dat");
velocityRecordedMotor recordedArmMotor(vex::PORT4, vex::gearSetting::ratio36_1, true, "motorArmVlog.dat");
velocityRecordedMotor recorderIntakeLeft(vex::PORT3, vex::gearSetting::ratio36_1, false, "motorIntakeLeftVlog.dat");
velocityRecordedMotor recorderIntakeRight(vex::PORT12, vex::gearSetting::ratio18_1, true, "motorIntakeRightVlog.dat");
velocityRecordedMotor recorderTilter(vex::PORT5, vex::gearSetting::ratio36_1, true, "motorTilterVlog.dat");

int recordFunction(void) {

    bool AlastPressed = false;
    bool BlastPressed = false;
    bool XlastPressed = false;
    bool YlastPressed = false;
    bool checkForPlaybackEnd = false;
    while (true) {
        if (!AlastPressed && Controller1.ButtonA.pressing()) {
            if (!recordedLeftMotor.isRecording()) {
                recordedLeftMotor.disableRecordingOrPlayback();
                recordedLeftMotorBack.disableRecordingOrPlayback();
                recordedRightMotor.disableRecordingOrPlayback();
                recordedRightMotorBack.disableRecordingOrPlayback();
                recorderIntakeLeft.disableRecordingOrPlayback();
                recorderIntakeRight.disableRecordingOrPlayback();
                recorderTilter.disableRecordingOrPlayback();
                recordedArmMotor.disableRecordingOrPlayback();
                recordedLeftMotor.enableRecording();
                recordedLeftMotorBack.enableRecording();
                recordedRightMotor.enableRecording();
                recordedRightMotorBack.enableRecording();
                recorderTilter.enableRecording();
                recorderIntakeLeft.enableRecording();
                recorderIntakeRight.enableRecording();
                recordedArmMotor.enableRecording();
                Controller1.Screen.setCursor(1,1);
                Controller1.Screen.print("Recording...         ");
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
            }
            AlastPressed = true;
        } else if (!BlastPressed && Controller1.ButtonB.pressing()) {
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
                Controller1.Screen.setCursor(1,1);
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
            BlastPressed = true;
        } else if (!XlastPressed && Controller1.ButtonX.pressing()) {
            recordedLeftMotor.disableRecordingOrPlayback();
            recordedLeftMotorBack.disableRecordingOrPlayback();
            recordedRightMotor.disableRecordingOrPlayback();
            recordedRightMotorBack.disableRecordingOrPlayback();
            recordedArmMotor.disableRecordingOrPlayback();
            recorderIntakeLeft.disableRecordingOrPlayback();
            recorderIntakeRight.disableRecordingOrPlayback();
            recorderTilter.disableRecordingOrPlayback();
            Controller1.Screen.setCursor(1,1);
            uint32_t startLoadTime = timer::system();
            Controller1.Screen.print("Saving...           ");
            recordedLeftMotor.stop();
            recordedLeftMotorBack.stop();
            recordedRightMotor.stop();
            recordedRightMotorBack.stop();
            recordedArmMotor.stop();
            recorderIntakeLeft.stop();
            recorderIntakeRight.stop();
            recorderTilter.stop();
            recordedLeftMotor.saveRecording();
            recordedLeftMotorBack.saveRecording();
            recordedRightMotor.saveRecording();
            recordedRightMotorBack.saveRecording();
            recordedArmMotor.saveRecording();
            recorderIntakeLeft.saveRecording();
            recorderIntakeRight.saveRecording();
            recorderTilter.saveRecording();
            while (timer::system() < startLoadTime + 1000); 
            Controller1.Screen.clearLine(1);
            XlastPressed = true;
        } else if (!YlastPressed && Controller1.ButtonY.pressing()) {
            recordedLeftMotor.disableRecordingOrPlayback();
            recordedLeftMotorBack.disableRecordingOrPlayback();
            recordedRightMotor.disableRecordingOrPlayback();
            recordedRightMotorBack.disableRecordingOrPlayback();
            recordedArmMotor.disableRecordingOrPlayback();
            recorderIntakeLeft.disableRecordingOrPlayback();
            recorderIntakeRight.disableRecordingOrPlayback();
            recorderTilter.disableRecordingOrPlayback();
            Controller1.Screen.setCursor(1,1);
            uint32_t startLoadTime = timer::system();
            Controller1.Screen.print("Loading...           ");
            recordedLeftMotor.stop();
            recordedLeftMotorBack.stop();
            recordedRightMotor.stop();
            recordedRightMotorBack.stop();
            recordedArmMotor.stop();
            recorderIntakeLeft.stop();
            recorderIntakeRight.stop();
            recorderTilter.stop();
            recordedLeftMotor.loadRecording();
            recordedLeftMotorBack.loadRecording();
            recordedRightMotor.loadRecording();
            recordedRightMotorBack.loadRecording();
            recordedArmMotor.loadRecording();
            recorderIntakeLeft.loadRecording();
            recorderIntakeRight.loadRecording();
            recorderTilter.loadRecording();
            while (timer::system() < startLoadTime + 1000);
            Controller1.Screen.clearLine(1);
            YlastPressed = true;
        }
        AlastPressed = AlastPressed && Controller1.ButtonA.pressing();
        BlastPressed = BlastPressed && Controller1.ButtonB.pressing();
        XlastPressed = XlastPressed && Controller1.ButtonX.pressing();
        YlastPressed = YlastPressed && Controller1.ButtonY.pressing();
        
        recordedLeftMotor.spin(vex::directionType::fwd,(Controller1.Axis2.position() + Controller1.Axis1.position()),vex::velocityUnits::pct);
        recordedLeftMotorBack.spin(vex::directionType::fwd,(Controller1.Axis2.position() + Controller1.Axis1.position()),vex::velocityUnits::pct);
        recordedRightMotor.spin(vex::directionType::fwd,(Controller1.Axis2.position() - Controller1.Axis1.position()),vex::velocityUnits::pct);
        recordedRightMotorBack.spin(vex::directionType::fwd,(Controller1.Axis2.position() - Controller1.Axis1.position()),vex::velocityUnits::pct);


  if (Controller1.ButtonR1.pressing() || BlastPressed == true) {

    recorderIntakeRight.spin(vex::directionType::fwd, 100,
                vex::velocityUnits::pct);
    recorderIntakeLeft.spin(vex::directionType::fwd, 100,
                    vex::velocityUnits::pct);
  } else if (Controller1.ButtonR2.pressing() || BlastPressed == true) {

    recorderIntakeRight.spin(vex::directionType::rev, 100,
                vex::velocityUnits::pct);
    recorderIntakeLeft.spin(vex::directionType::rev, 100,
                    vex::velocityUnits::pct);
  } else{
    recorderIntakeRight.spin(vex::directionType::rev, 0,
                vex::velocityUnits::pct);
    recorderIntakeLeft.spin(vex::directionType::rev, 0,
                    vex::velocityUnits::pct);
  }

  /*-----------------------------------------------------------------------------*/
  /** @brief     Switch Functions Between 2bar control and 4bar control */
  /*-----------------------------------------------------------------------------*/
   recorderTilter.spin(vex::directionType::fwd, (Controller1.Axis3.position()),vex::velocityUnits::pct);

    if (Controller1.ButtonL1.pressing() || BlastPressed == true) {

    recordedArmMotor.spin(vex::directionType::fwd, 100,
                vex::velocityUnits::pct);
  } else if (Controller1.ButtonL2.pressing() || BlastPressed == true) {

    recordedArmMotor.spin(vex::directionType::rev, 100,
                vex::velocityUnits::pct);
  } else{
    recordedArmMotor.spin(vex::directionType::rev, 10,
                vex::velocityUnits::pct);
  }
  /*-----------------------------------------------------------------------------*/
  /** @brief     Macros to help the driver */
  /*-----------------------------------------------------------------------------*/

        
        if (checkForPlaybackEnd && recordedLeftMotor.donePlayback() && recordedLeftMotorBack.donePlayback() && recordedRightMotor.donePlayback() && recordedRightMotorBack.donePlayback() && recordedArmMotor.donePlayback() && recorderIntakeLeft.donePlayback() && recorderIntakeRight.donePlayback() && recorderTilter.donePlayback()) {
            checkForPlaybackEnd = false;
            Controller1.Screen.setCursor(1,1);
            Controller1.Screen.clearLine(1);
            Controller1.Screen.print("Done playback");
            recordedLeftMotor.disablePlayback();
            recordedLeftMotorBack.disablePlayback();
            recordedRightMotor.disablePlayback();
            recordedRightMotorBack.disablePlayback();
            recordedArmMotor.disablePlayback();
            recorderIntakeLeft.disablePlayback();
            recorderIntakeRight.disablePlayback();
            recorderTilter.disablePlayback();
        }
        
        task::sleep(30);
    }
}
/*Button A: start recording (deletes any recording in memory!)

Button B: begin playback

Button X: saves recording to SD card (overwrites previously saved recording!)

Button Y: loads recording from SD card (deletes any recording in memory!)*/