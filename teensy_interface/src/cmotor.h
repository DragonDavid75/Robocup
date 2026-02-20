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


#ifndef UMOTOR_H
#define UMOTOR_H

#include <thread>

#include "sencoder.h"
#include "utime.h"
#include "upid.h"
#include "srobot.h"

/**
 * Class to do motor velocity control.
 * This class uses the encoder values as velocity measurement.
 * The desired motor velocity reference is received from
 * the mixer module.
 * */
class CMotor
{
public:
  /** setup and request data */
  void setup(int teensy_number);
  /**
   * thread to do updates, when new data is available */
  void run();
  /**
   * Decode messages from Teensy */
  bool decode(const char* msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();
  /**
   * Should motors relax (stop control loop and save power)
   * \param value, if true, then set motor voltage to 0, triggering a relax
   *               if false, then run control loop */
  void setRelax(bool value);
  /**
   * are motor in relax state */
  bool inRelax()
  {
    return relax;
  }
  /**
   * check data flow */
  void tick();


protected:
  /**
   * Save motor values for all motors */
  void toLogMv(UTime & updt);
  //
  void subscribeDataFromTeensy();
  //
public:
  float desiredVelocity[SRobot::MAX_MOTORS] = {0.0};
  // is output limited, this may be valuable for other controllers.
  bool limited[SRobot::MAX_MOTORS] = {false};
  UTime updTime; // time of last control update

private:
  /// number of this teensy
  int tn = 0;
  string ini_section;
  /// private stuff
  static void runObj(CMotor * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  void logfileLeadText(FILE * f, const char * ms);
  /**
   * PID controllers, one each motor */
  UPID pid[SRobot::MAX_MOTORS];
  //
  float sampleTime;
  // controller output
  float u[SRobot::MAX_MOTORS];
  // support variables
  FILE * logfile[SRobot::MAX_MOTORS] = {nullptr};
  FILE * logfileMv = nullptr;
  float motorVoltage[SRobot::MAX_MOTORS] = {0};
  float motorVoltageOffset[SRobot::MAX_MOTORS] = {0};
  int   motorPWM[SRobot::MAX_MOTORS] = {0};
  thread * th1;
  bool stop = false;
  int dataCnt = 0;
  int velUpdateCnt = 0;
  int relaxing = 0;
  /// should motor control relax, i.e. stop closed loop.
  bool relax = true;
  UTime relaxTime;
  float timeToRelax = 3.5; // relax if zero speed more than these seconds
  // mqtt
  std::string topicMotv;
  // subscription
  int sub_mvt = 0;
  UTime sub_mvt_t;
  int sub_mpt = 0;
  UTime sub_mpt_t;
};

/**
 * Make this visible to the rest of the software */
extern CMotor motor[NUM_TEENSY_MAX];

#endif
