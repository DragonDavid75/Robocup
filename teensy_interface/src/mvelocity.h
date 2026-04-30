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


#ifndef MPOSE_H
#define MPOSE_H

#include "sencoder.h"
#include "utime.h"
#include "thread"

using namespace std;

/**
 * Class that update robot based on wheel encoder update.
 * The result is odometry coordinate update
 *   x,y position
 *   h (heading)
 *   time of last encoder update (poseTime)
 *   wheel velocity (eheelVel)
 * An updateCnt is incremented at every update
 * */
class MVelocity
{
public:
  /** setup and request data */
  void setup(int teensyNumber);
  /**
   * thread to do updates, when new data is available */
  void run();
  /**
   * terminate */
  void terminate();

protected:
  int tn;
  float encTickPerRev = 64;
  float radPerTick = (2.0 * M_PI) / encTickPerRev;
  bool useTeensyVelEstimate;

public:
  bool areMotorsRunning()
  { // are motor running, in rad/s for motor before gear
    return fabsf(motorVel[0] > 0.2) or fabsf(motorVel[1] > 0.1);
  }
  /// PC time of last update
  UTime velTime;
  float sampleTime;
  //  Calculated motor velocity (rad/s on motor side)
  float motorVel[SRobot::MAX_MOTORS] = {0.0};
  // new pose is calculated count
  int updateCnt = 0;
  int oldEncUpdate = 0;
  int oldEncVelUpdate = 0;

private:
  /// private stuff
  std::string ini_section;
  std::string topicVel;

  static void runObj(MVelocity * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  /**
   * Motor velocity may need scaling - used for linear actuators, where geering is unknown.*/
  float motorScale[SRobot::MAX_MOTORS]{1.0};
  /**
   * print to console and logfile */
  void toLog();
  // support variables
  bool firstEnc = true;
  /// Debug print
  bool toConsole = false;
  /// Logfile - most details
  FILE * logfile = nullptr;
  std::thread * th1;
  // source data iteration
  int encoderUpdateCnt = 0;
};

/**
 * Make this visible to the rest of the software */
extern MVelocity mvel[NUM_TEENSY_MAX];

#endif
