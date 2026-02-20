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


#ifndef SJOYLOGITECH_H
#define SJOYLOGITECH_H

#include <thread>
#include "utime.h"

/**
 * Class to allow manual control using a Ligitech gamepad
 */
class SJoyLogitech
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

  int getAxis(int axis)
  {
    if (axis >= 0 and axis < number_of_axes)
      return joyValues.axis[axis];
    else
      return 0;
  }

  bool getButton(int button)
  {
    if (button >= 0 and button < number_of_buttons)
      return joyValues.button[button];
    else
      return 0;
  }

public:
  /** update count upfdated when new values */
  int updateCnt = 0;
  UTime updTime;

public:
  /** is mission overwritten by manual override */
//   bool manOverride = true;
  /** Control forward velocity in m/s */
  // float velocity = 0.0;
  // /** Control turnrate (rad/s) */
  // float turnValue = 0.0;
  // /** servo position */
  // float servoPosition = 0.0;
  /** is device available */
  bool joyRunning = false;
  //Structure to hold joystick values
  struct jVal {
    bool button[16] = {0};
    int axis[16] = {0};
  };
  struct jVal joyValues;
  int number_of_axes = 8, number_of_buttons = 11;

private:
  /// private stuff
  static void runObj(SJoyLogitech * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  void toLog();
  std::thread * th1;
  bool toConsole = false;
  FILE * logfile = nullptr;
  //
  // device
  // int buttonFast;// on gamepad
  // int axisVel;   // on gamepad
  // int axisTurn;  // on gamepad
  // int axisServo; // on gamepad
  // int servoToControl;  // servo to control [1..5]
  // float slowFactor; // when not using fast button
  /**
   * Open joustick device,
   * \returns false if device not found */
  bool initJoy();
  std::string deviceName = "unknown";
  /**
   * Get fresh data from joystick
   * \return false if device disappeared of received other than event data */
  bool getNewJsData();
  void joyControl();
  //
  std::string joyDevice = "/dev/input/js0";
  int jDev = -1;  ///File descriptors
  /// are we running in fast mode = 1.0, otherwise a bit slower with this factor
  // float velScale, turnScale, servoScale;
  // float maxVel, maxTurn;
  // bool isFast;
  // button update - ensure is included in log
  bool gotButton = false;
};

/**
 * Make this visible to the rest of the software */
extern SJoyLogitech joyLogi;

#endif
