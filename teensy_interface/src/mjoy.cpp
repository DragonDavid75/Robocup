/*  
 * 
 * Copyright © 2022 DTU, 
 * Author:
 * Christian Andersen jcan@dtu.dk
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

#include <sys/ioctl.h>
#include <signal.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "sjoylogitech.h"
#include "uservice.h"
#include "cmixer.h"
#include "cservo.h"
#include "mjoy.h"

// #define JS_EVENT_BUTTON         0x01    /* button pressed/released */
// #define JS_EVENT_AXIS           0x02    /* joystick moved */
// #define JS_EVENT_INIT           0x80    /* initial state of device */

// start is green button
// NB! mode switch in front of gamepad MUST be in "X" position
// #define BUTTON_GREEN   0
// // red button is 1
// #define BUTTON_RED   1
// // blue button is 2
// #define BUTTON_BLUE   2
// // image is yellow button (3)
// // #define BUTTON_IMAGE   3
// #define BUTTON_YELLOW   3
// // left front button LB is 4
// #define BUTTON_LB    4
// // right front button RB is 5 (fast)
// #define BUTTON_RB    5
// #define BUTTON_FAST  BUTTON_RB
// //
// // 3 upper-center buttons
// // button labeled 'back' is 6
#define BUTTON_BACK 6
#define BUTTON_MANUAL BUTTON_BACK
// // button labeled 'start'
#define BUTTON_START 7
#define BUTTON_AUTO BUTTON_START
// // Center big 'Logitech' button is 8
// #define BUTTON_LOGITECH 8
// axis 0 is left hand left-right axis
// axis 1 is left hand up-down axis
// axis 2 is left front speeder
// axis 3 is right hand left-right
// axis 4 is right hand up-down
// axis 5 is right front speeder
// axis 6 id digital left-write
// axis 7 is digital up-down

// create value
MJoy joy;


void MJoy::setup()
{ // ensure default values
  if (not ini.has("Joy_use"))
  { // no data yet, so generate some default values
    ini["Joy_use"]["log"] = "true";
    ini["Joy_use"]["print"] = "false";
    /// limit in m/s (velocity) and radian/sec (turnrate) and us/sec (servo rate) for max axis
    ini["Joy_use"]["drive_control"] = "true";    // bool - calculate drive values
    ini["Joy_use"]["vel_limit"] = "0.8";    // m/s
    ini["Joy_use"]["vel_axis"] = "4";  // right knob up-down
    ini["Joy_use"]["turn_limit"] = "10.0";  // rad/m or rad/s
    ini["Joy_use"]["turn_axis"] = "3";  // right knob left-right
    ini["Joy_use"]["Button_fast"] = "5"; // RB
    ini["Joy_use"]["slow_factor"] = "0.3";
    ini["Joy_use"]["axis_servo"] = "1"; // left knob up-down
    ini["Joy_use"]["servo"] = "1"; // which servo to use (1..3)
    ini["Joy_use"]["servoScale"] = "10"; // influence servo reaction speed (integrator)
    //
  }
  // drive control
  drive_control = ini["Joy_use"]["drive_control"] == "true";
  buttonFast = strtol(ini["Joy_use"]["Button_fast"].c_str(), nullptr, 10);
  axisVel = strtol(ini["Joy_use"]["vel_axis"].c_str(), nullptr, 10);
  axisTurn = strtol(ini["Joy_use"]["turn_axis"].c_str(), nullptr, 10);
  axisServo = strtol(ini["Joy_use"]["axis_Servo"].c_str(), nullptr, 10);
  servo = strtol(ini["Joy_use"]["servo"].c_str(), nullptr, 10);
  servoScale = strtod(ini["Joy_use"]["servoScale"].c_str(), nullptr);
  slowFactor = strtod(ini["Joy_use"]["slow_factor"].c_str(), nullptr);
  // max velocity in m/sec and rad/sec
  maxVel = strtof(ini["Joy_use"]["vel_limit"].c_str(), nullptr);
  maxTurn = strtof(ini["Joy_use"]["turn_limit"].c_str(), nullptr);
  // convertion factors
  velScale = maxVel/32000;
  turnScale = maxTurn/32000;
  //
  //
  // start read thread
  toConsole = ini["Joy_use"]["print"] == "true";
  if (ini["Joy_use"]["log"] == "true" and logfileD == nullptr)
  { // open logfile
    if (drive_control)
    {
      std::string fn = service.logPath + "log_joy_drive.txt";
      logfileD = fopen(fn.c_str(), "w");
      fprintf(logfileD, "%% Manual drive control\n");
      fprintf(logfileD, "%% Button fast %d\n", buttonFast);
      fprintf(logfileD, "%% Axis vel %d\n", axisVel);
      fprintf(logfileD, "%% Axis turn %d\n", axisTurn);
      fprintf(logfileD, "%% Axis servo %d\n", axisServo);
      fprintf(logfileD, "%% Slow factor %g\n", slowFactor);
      fprintf(logfileD, "%% Max velocity (m/s) %g\n", maxVel);
      fprintf(logfileD, "%% Max turnrate (rad/s) %g\n", maxTurn);
      fprintf(logfileD, "%% 1 \tTime (sec)\n");
      fprintf(logfileD, "%% 2 \tManual mode (else automatic)\n");
      fprintf(logfileD, "%% 3 \tLinear velocity (m/s)\n");
      fprintf(logfileD, "%% 4 \tTurn curvature value (rad/m)\n");
      fprintf(logfileD, "%% 5 \tYaw velocity (rad/s) - only id yaw control accept from drive joypad\n");
    }
  }
  // start listen thread
  if (th1 == nullptr)
    th1 = new std::thread(runObj, this);
}

void MJoy::terminate()
{
  // printf("# joy terminate\n");
  if (th1 != nullptr)
    th1->join();
  if (logfileD != nullptr)
  {
    fclose(logfileD);
    logfileD = nullptr;
  }
  if (logfileC != nullptr)
  {
    fclose(logfileC);
    logfileC = nullptr;
  }
}

void MJoy::run()
{
  UTime t;
  t.now();
  sleep(3);
  // bool automaticModeOld = false;
  int lastUpdate = joyLogi.updateCnt;
  while (not service.stop)
  { // handling gamepad events
    // Device is present
    if (joyLogi.updateCnt != lastUpdate)
    { //Detect manual override toggling
      lastUpdate = joyLogi.updateCnt;
      // mode change
      if (manual and joyLogi.getButton(BUTTON_AUTO) == 1)
      {
        printf("# SJoy:: shift to auto mode (button 'start' pressed)\n");
        manual = false;
      }
      if (not manual and joyLogi.getButton(BUTTON_MANUAL) == 1)
      {
        printf("# SJoy:: shift to manual mode (button 'back' pressed)\n");
        manual = true;
      }
      //
      updTime.now();
      if (manual)
      { // we are in manual mode, so
        // generate robot control from gamepad
        if (drive_control)
        {
          isFast = joyLogi.getButton(buttonFast) == 1;
          joyControlDrive();
        }
        joyControlServo();
        // if (crane_control)
        // {
        //   joyControlCrane();
        // }
        updateCnt++;
      }
      toLog();
    }
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  // printf("# Joy run finished\n");
}


void MJoy::joyControlDrive()
{ // we are in manual override mode
  // so do what is needed
  float fastScale;
  if (isFast)
    fastScale = 1.0;
  else
    fastScale = slowFactor;
  // velocity
  if (abs(joyLogi.getAxis(axisVel)) > 500)
  { // velocity is valid
    float velVector;
    velVector = joyLogi.getAxis(axisVel);
    velocity = velVector * fastScale * velScale;
  }
  else
    velocity = 0.0;
  // turn (Turn value is curvature (for Scorpi))
  if (abs(joyLogi.getAxis(axisTurn)) > 500)
  { // velocity is valid
    turnValue = -float(joyLogi.getAxis(axisTurn)) * turnScale;
  }
  else
    turnValue = 0.0;
  //
  // allow yaw control from drive game-pad
  // if (abs(joyLogi.getAxis(yaw_axis)) > 500)
  // { // value is valid
  //   yawVelocity = joyLogi.getAxis(yaw_axis) * yaw_scale;
  // }
  // else
  //   yawVelocity = 0.0;
}

void MJoy::joyControlServo()
{
  if (abs(joyLogi.getAxis(axisServo)) > 500)
  { // servo activated is valid
    int val = joyLogi.getAxis(axisServo);
    servoPosition += val / 32000.0 * servoScale;
    //
    if (servoPosition > 1024)
      servoPosition = 1024;
    else if (servoPosition < -1024)
      servoPosition = -1024;
    // make string to control servo
    const int MSL = 100;
    char s[MSL];
    if (abs(servoPosition) == 1024)
      // disable servo
      snprintf(s, MSL, "servo %d 10000 0\n", servo);
    else
      // valid position, set servo
      snprintf(s, MSL, "servo %d %d 0\n", servo, int(servoPosition));
    teensy[0].send(s);
  }
}


// void MJoy::joyControlCrane()
// {
//   if (abs(joyLogi.getAxis(yaw_axis)) > 500)
//   { // value is valid
//     yawVelocity = joyLogi.getAxis(yaw_axis) * yaw_scale;
//   }
//   else
//     yawVelocity = 0.0;
//   //
//   if (abs(joyLogi.getAxis(boom_axis)) > 500)
//   { // value is valid
//     boomVelocity = joyLogi.getAxis(boom_axis) * boom_scale;
//   }
//   else
//     boomVelocity = 0.0;
//   // printf("# Joy::crane: axis %d %d, scale %g, yaw vel %g\n", yaw_axis, joyLogi.getAxis(yaw_axis), yaw_scale, yawVelocity);
//   // printf("# Joy::crane: axis %d %d, scale %g, boom vel %g\n", boom_axis, joyLogi.getAxis(boom_axis), boom_scale, boomVelocity);
//   //
//   if (abs(joyLogi.getAxis(arm_axis)) > 500)
//   { // value is valid
//     armVelocity = joyLogi.getAxis(arm_axis) * arm_scale;
//   }
//   else
//     armVelocity = 0.0;
//   //
//   if (abs(joyLogi.getAxis(gripper_axis)) > 500)
//   { // value is valid
//     gripperVelocity = joyLogi.getAxis(gripper_axis) * gripper_scale;
//   }
//   else
//     gripperVelocity = 0.0;
//   printf("# Joy::crane: axis %d %d, scale %g, gripper vel %g\n", gripper_axis, joyLogi.getAxis(gripper_axis), gripper_scale, gripperVelocity);
// }


void MJoy::toLog()
{
  if (not service.stop)
  {
    if (drive_control)
    {
      if (logfileD != nullptr and not service.stop_logging)
      { // save all axis and buttons
        fprintf(logfileD, "%lu.%04ld %d %g %g %g %g\n", updTime.getSec(), updTime.getMicrosec()/100,
                manual, velocity, turnValue, servoPosition, yawVelocity
        );
      }
      if (toConsole)
      { // save all axis and buttons
        printf("# Joy Drive: %lu.%04ld %d %g %g %g %g\n", updTime.getSec(), updTime.getMicrosec()/100,
                manual, velocity, turnValue, servoPosition, yawVelocity
        );
      }
    }
    // if (crane_control)
    // {
    //   if (logfileC != nullptr)
    //   { // save all axis and buttons
    //     fprintf(logfileC, "%lu.%04ld %d %g %g %g %g\n", updTime.getSec(), updTime.getMicrosec()/100,
    //             manual, yawVelocity, boomVelocity, armVelocity, gripperVelocity
    //     );
    //   }
    //   if (toConsole)
    //   { // save all axis and buttons
    //     printf("# Joy Crane: %lu.%04ld %d %g %g %g %g\n", updTime.getSec(), updTime.getMicrosec()/100,
    //             manual, yawVelocity, boomVelocity, armVelocity, gripperVelocity
    //     );
    //   }
    // }
  }
}

