/*  
 * 
 * Copyright © 2023 DTU, Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */


#pragma once

#include <thread>
#include "utime.h"

/**
 * Class to allow manual control using a Ligitech gamepad
 */
class MJoy
{
public:
  /** setup and request data */
  void setup();
  /**
   * regular update tick */
  void run();
  /**
   * terminate */
  void terminate();
  /**
   * are we running in manual override mode,
   * else manual using the gamepad */
  bool manualMode()
  {
    return manual;
  }

public:
  /** update count upfdated when new values */
  int updateCnt = 0;
  UTime updTime;

public:
  /** is mission overwritten by manual override */
//   bool manOverride = true;
  /** Control forward velocity in m/s */
  float velocity = 0.0;
  /** Control turnrate (rad/s) */
  float turnValue = 0.0;
  /** servo position */
  float servoPosition = 0.0;
  float yawVelocity = 0;
  float boomVelocity = 0;
  float armVelocity = 0;
  float gripperVelocity = 0;

private:
  /// private stuff
  static void runObj(MJoy * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  void toLog();
  std::thread * th1;
  bool toConsole = false;
  FILE * logfileD = nullptr; // drive
  FILE * logfileC = nullptr; // crane
  //
  // device
  bool manual = false;
  // drive values
  bool drive_control = true;    // bool calculate crane control values
  int buttonFast;// on gamepad
  int axisVel;   // on gamepad
  int axisTurn;  // on gamepad
  int axisServo; // on gamepad
  int servo = 1;
  float servoScale = 100;
  //
  float slowFactor; // when not using fast button
  void joyControlDrive();
  void joyControlServo();
  /// are we running in fast mode = 1.0, otherwise a bit slower with this factor
  float velScale, turnScale;
  float maxVel, maxTurn;
  bool isFast;
  //
  // crane values
  // bool crane_control = true;    // bool calculate crane control values
  // // boom is the lower actuator on crane
  // float boom_vel_limit = 0.01;  // m/s
  // int boom_axis = 4;
  // float boom_scale; // from joy value to stick value
  // // arm is the upper actuator
  // float arm_vel_limit = 0.01;  // m/s
  // bool arm_axis = 4;
  // float arm_scale;
  // // yaw is the rotation actuator
  // float yaw_vel_limit = 1.0;  // rad/s
  // int yaw_axis = 4;
  // float yaw_scale;
  // // gripper
  // int gripper_vel_limit = 1000;  // us/s
  // int gripper_axis = 4;
  // float gripper_scale;
  // void joyControlCrane();
  //
};

/**
 * Make this visible to the rest of the software */
extern MJoy joy;

