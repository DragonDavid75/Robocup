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


#ifndef MMIXER_H
#define MMIXER_H

#include "cmotor.h"
#include "utime.h"
// #include "cheading.h"
//#include "mvelocity.h"

using namespace std;

/**
 * The mixer translates linear and rotation reference
 * values to velocity for each motor.
 * */
class CMixer
{
public:
  /** setup and initialize parameters */
  void setup();
  /**
   * Decode messages */
  bool decode(const char* msg, UTime & msgTime);
  /**
   * thread to do updates, when new data is available */
  void run();
  /**
   * close down */
  void terminate();
  /**
   * Velocity control in automnomous mode
   * \param refLinearVelocity in meter per second
   * \param refCurvature in rad/m
   * */
   void setVelocity(float refLinearVelocity, float refCurvature);
   /**
    * are wheels commanded to run,
    * \returns true if commanded velocity is > 0 or turning */
   bool shouldWheelsBeRunning()
   {
     return fabsf(linVel) > 0.001 or fabsf(turnrate) > 0.001;
   }


public:
  /// Mixer update cnt
  int updateCnt = 0;
  UTime rcTime;
  int rcSource = 0;
  UTime updateTime;

private:
  /// private stuff
  std::thread * th1 = nullptr;
  static void runObj(CMixer * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  /**
   * Check if the index range is within limits.
   * first index [0] must be within 0..teensys,
   * next index [1] two must be within 0..motors
   * third index [2] two must be within -1..motors, -1 means no second motor
   * \retruns true if all is OK */
  bool limitMotorValues(int (&idx)[3], int teensys, int motors);
  /** autonomous preference */
  float desiredCurvature = 0;
  float desiredLinVel = 0;
  bool autoUpdate = false;
  /** update count for data sources */
  int oldUpdCntManual = -1;
  int oldUpdCntVelocity = -1;
  int oldUpdCntSteer = -1;
  // mixed motor decired velocity
  float v0, v1;
  /** log data for this module */
  void toLog();
  //
  FILE * logfile = nullptr;
  bool toConsole = false;
  /// Linear velocity (m/s)
  float linVel = 0;
  float turnrate = 0;
  // manual override mode
  bool manualOverride = false;
  //
  float wheelbase;
  float motorGear;
  float driveGear;
  float wheelRadius;
  //
  float velDif; // desired velocity difference
  //
  // motor connection
  int driveMotorLeft[3]; /// which Teensy and which motors (up to 2) to drive (-1 mean not used)
  int driveMotorRight[3]; /// which Teensy and which motors (up to 2) to drive (-1 mean not used)
  // int turnMotorLeft[3]; /// which Teensy and which motors (up to 2) to turn (-1 mean not used)
  // int turnMotorRight[3]; /// which Teensy and which motors (up to 2) to turn (-1 mean not used)
};

/**
 * Make this visible to the rest of the software */
extern CMixer mixer;

#endif
