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

#include <string>
#include <string.h>
#include <thread>
#include <math.h>
#include "sencoder.h"
#include "cmixer.h"
#include "cmotor.h"
#include "steensy.h"
#include "uservice.h"
// #include "csteering.h"
#include "mjoy.h"
#include "mvelocity.h"
#include <stdlib.h>

// create value
CMixer mixer;


/// mixer class combines drive orders to desired wheel velocity
void CMixer::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("mixer"))
  { // no data yet, so generate some default values
    ini["mixer"]["use"] = "true";
    ini["mixer"]["log"] = "true";
    ini["mixer"]["print"] = "false";
    // motor drive - format "Teensy M1 M2"
    ini["mixer"]["driveLeft"] =  "0 0 -1";
    ini["mixer"]["driveRight"] = "0 1 -1";
    /// motor turn - format Teensy M1 M2 (M2 not used for turning)
    // ini["mixer"]["turnLeft"] =   "1 3 -1";
    // ini["mixer"]["turnRight"] =  "1 4 -1";
    // wheel base
    ini["mixer"]["wheelbase"] = "0.22";           // distance between driving wheels (m)
    ini["mixer"]["motor_gear"] = "18";            // drive motor gear
    ini["mixer"]["drive_gear"] = "1"; // any gear after the motor gear
    ini["mixer"]["wheel_radius"] = "0.075";       // wheel radius (m)
  }
  // get values from ini-file
  //
  wheelbase = strtof(ini["mixer"]["wheelbase"].c_str(), nullptr);
//   turnrateControl = ini["heading"]["enabled"] == "true";
  // wheelbase must not be zero or negative
  if (wheelbase < 0.005)
    wheelbase = 0.34;
  motorGear = strtof(ini["mixer"]["motor_gear"].c_str(), nullptr);
  if (motorGear < 5)
    motorGear = 10;
  driveGear = strtof(ini["mixer"]["drive_gear"].c_str(), nullptr);
  if (driveGear < 1)
    driveGear = 3.43;
  wheelRadius = strtof(ini["mixer"]["wheel_radius"].c_str(), nullptr);
  if (wheelRadius < 0.01)
    wheelRadius = 0.1;
  // motor connection
  const char * p1 = ini["mixer"]["driveLeft"].c_str();
  for (int i = 0; i < 3; i++)
    driveMotorLeft[i] = strtol(p1, (char **)&p1, 10);
  p1 = ini["mixer"]["driveRight"].c_str();
  for (int i = 0; i < 3; i++)
    driveMotorRight[i] = strtol(p1, (char **)&p1, 10);
  // p1 = ini["mixer"]["turnLeft"].c_str();
  // for (int i = 0; i < 3; i++)
  //   turnMotorLeft[i] = strtol(p1, (char **)&p1, 10);
  // p1 = ini["mixer"]["turnRight"].c_str();
  // for (int i = 0; i < 3; i++)
  //   turnMotorRight[i] = strtol(p1, (char **)&p1, 10);
  bool allIdsOK = limitMotorValues(driveMotorLeft, NUM_TEENSY_MAX, 2);
  allIdsOK &= limitMotorValues(driveMotorRight, NUM_TEENSY_MAX, 2);
  // allIdsOK &= limitMotorValues(turnMotorLeft, NUM_TEENSY_MAX, 4);
  // allIdsOK &= limitMotorValues(turnMotorRight, NUM_TEENSY_MAX, 4);
  if (not allIdsOK)
  { // modify ini-file for next time
    printf("# CMixer:: some motor references in robot.ini were out legal values (set to zero or -1)\n");
    ini["mixer"]["driveLeft"] =  to_string(driveMotorLeft[0]) + " " +  to_string(driveMotorLeft[1]) + " " +  to_string(driveMotorLeft[2]);
    ini["mixer"]["driveRight"] = to_string(driveMotorRight[0]) + " " + to_string(driveMotorRight[1]) + " " + to_string(driveMotorRight[2]);
    // ini["mixer"]["turnLeft"] =   to_string(turnMotorLeft[0]) + " " +   to_string(turnMotorLeft[1]) + " " +   to_string(turnMotorLeft[2]);
    // ini["mixer"]["turnRight"] =  to_string(turnMotorRight[0]) + " " +  to_string(turnMotorRight[1]) + " " +  to_string(turnMotorRight[2]);
  }
  //
  if (ini["mixer"]["use"] == "true")
  { // Mixer to drive robot should be active
    toConsole = ini["mixer"]["print"] == "true";
    if (ini["mixer"]["log"] == "true" and logfile == nullptr)
    { // open logfile
      std::string fn = service.logPath + "log_mixer.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% Mixer logfile\n");
      fprintf(logfile, "%% Wheel base used in calculation: %g m\n", wheelbase);
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tcontrol source 0 = this, 1 = manuel (gamepad), 2.. = MQTT\n");
      fprintf(logfile, "%% 3 \tmanual override mode (0= automatic, 1=manuel mode (gamepad))\n");
      fprintf(logfile, "%% 4 \tLinear velocity (m/s)\n");
      fprintf(logfile, "%% 5 \tCurvature (rad/m)\n");
      fprintf(logfile, "%% 6 \tDesired left wheel velocity (rad/s)\n");
      fprintf(logfile, "%% 7 \tDesired right wheel velocity (rad/s)\n");
      // fprintf(logfile, "%% 7 \tDesired left turn-motor velocity (rad/s)\n");
      // fprintf(logfile, "%% 8 \tDesired right turn-motor velocity (rad/s)\n");
    }
    if (th1 == nullptr)
      th1 = new std::thread(runObj, this);
  }
}

bool CMixer::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "rc", 2) == 0)
  {
    p1 += 2;
    // printf("# CMixer::decode: %lu.%04ld topic=%s, params %s\n", msgTime.getSec(), msgTime.getMicrosec()/100, msg, p1);
    if (strlen(p1) < 3)
      return false;
    rcTime = msgTime;
    // rcSource = strtol(p1, (char**)&p1, 10);
    rcSource = 2;
    float vel = strtof(p1, (char**)&p1);
    float rot = strtof(p1, (char**)&p1);
    if (rcSource > 1)
    {
      setVelocity(vel, rot);
      // notify users of a new update
      updateCnt++;
      // save to log - saved by run() loop
      // if (updateCnt > 2)
      //   toLog();
    }
  }
  else
    used = false;
  return used;
}


void CMixer::run()
{
  bool upd;
  // bool updTurnMotors;
  int loop = 0;
  while (not service.stop)
  {
    loop++;
    upd = autoUpdate;
    autoUpdate = false;
    // updTurnMotors = false;
    manualOverride = joy.manualMode();
    if (joy.updateCnt != oldUpdCntManual and manualOverride)
    { // new manual values
      oldUpdCntManual = joy.updateCnt;
      upd = true;
    }
    if (upd or updateTime.getTimePassed() > 0.5)
    { // use the new data
      updateTime.now();
      if (manualOverride)
      {  // source is manual control
        linVel = joy.velocity;
        turnrate = joy.turnValue;
        rcSource = 0;
        // if (loop % 50 == 0)
        // printf("# CMixer, manualOverride, vel=%f (rad/s), curvature=%f (rad/m)\n", linVel, turnrate);
      }
      else
      { // source is MQTT
        linVel = desiredLinVel;
        turnrate = desiredCurvature;
        // if (loop % 50 == 0)
        //  printf("# CMixer::run: auto, vel=%f (rad/s), curvature=%f (rad/m)\n", linVel, turnrate);
      }
      // mix to motor velocities
      // use actual steering curvature or turnrate (rad/m or rad/s) to adapt driving wheel speed.
      // velDif = wheelbase * curvature * linVel;
      velDif = wheelbase * turnrate;
      // adjust each wheel with half difference
      // positive curvature (CCV) makes the right
      // wheel turn faster forward
      v1 = linVel + velDif/2; // right wheel (m/s)
      v0 = v1 - velDif;       // left wheel (m/s)
      //
      // convert from drive speed (m/s) to motor velocity (rad/s)
      if (false)
      { // robobot is using wheel velocity
        v0 *= motorGear * driveGear / wheelRadius;
        v1 *= motorGear * driveGear / wheelRadius;
      }
      //
      // tell drive motor controllers
      // driveMotorXXXXX[0] is teensy number
      // driveMotorXXXXX[1] is motor on that number (0..3)
      // driveMotorXXXXX[2] is additional motor or -1 for none
      //
      if (shouldWheelsBeRunning())
      {
        // if (motor[0].inRelax() and updateTime.getSec() > 0)
        //   printf("# CMixer:: %lu.%04ld got out of relax (linvel=%g, turnrate=%g)\n",
        //          updateTime.getSec(), updateTime.getMicrosec()/100, linVel, turnrate);
        motor[0].setRelax(false);
      }
      else if (not mvel[0].areMotorsRunning())
      { // motors has stopped, so relax
        // if (not motor[0].inRelax())
        //   printf("# CMixer:: %lu.%04ld got into relax (linvel=%g, turnrate=%g)\n",
        //          updateTime.getSec(), updateTime.getMicrosec()/100, linVel, turnrate);
        motor[0].setRelax(true);
      }
      // left motor(s)
      motor[driveMotorLeft[0]].desiredVelocity[driveMotorLeft[1]] = v0;  // m1
      if (driveMotorLeft[2] >= 0)
        motor[driveMotorLeft[0]].desiredVelocity[driveMotorLeft[2]] = v0; // m2
      // right motor(s)
      motor[driveMotorRight[0]].desiredVelocity[driveMotorRight[1]] = v1; // m3
      if (driveMotorRight[2] >= 0)
        motor[driveMotorRight[0]].desiredVelocity[driveMotorRight[2]] = v1; // m4
      //
      if (updateCnt > 0)
        toLog();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}


void CMixer::setVelocity(float refLinearVelocity, float refCurvature)
{
  desiredLinVel = refLinearVelocity;
  desiredCurvature = refCurvature;
  autoUpdate = true;
  printf("# CMixer::setVelocity: vel=%.3f (m/s), turnrate=%.3f (rad/sec)\n", refLinearVelocity, refCurvature);
}


void CMixer::terminate()
{
  if (th1 != nullptr)
  {
    th1->join();
  }
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
  }
}


// void CMixer::setManualControl(bool manual)
// {
//   manualOverride = manual;
//   autoUpdate = true;
// }


void CMixer::toLog()
{
  if (service.stop)
    return;
  if (logfile != nullptr and not service.stop_logging)
  { // add to log after update
    fprintf(logfile, "%lu.%04ld %d %d %.3f %.3f %.3f %.3f %d\n",
            updateTime.getSec(), updateTime.getMicrosec()/100,
            rcSource, manualOverride, linVel, turnrate, v0, v1, updateCnt
            );
  }
  if (toConsole)
  {
    printf("%% CMixer: %lu.%04ld %d %d %.3f %.3f %.3f %.3f %d\n",
           updateTime.getSec(), updateTime.getMicrosec()/100,
           rcSource, manualOverride, linVel, turnrate, v0, v1, updateCnt
           );
  }
}


bool CMixer::limitMotorValues ( int (&idx)[3], int teensys, int motors )
{
  bool isOK = true;
  if (idx[0] < 0 or idx[0] >= teensys)
  {
    isOK = false;
    idx[0] = 0;
  }
  if (idx[1] < 0 or idx[1] >= motors)
  { // first (or only) motor for this purpose.
    isOK = false;
    idx[1] = 0;
  }
  if (idx[2] < -1 or idx[2] >= motors)
  { // optional second motor for this purpose, -1 for not used.
    isOK = false;
    idx[2] = -1;
  }
  return isOK;
}
