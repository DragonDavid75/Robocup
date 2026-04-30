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


#ifndef SENCODER_H
#define SENCODER_H

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <sys/types.h>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include "utime.h"
#include "steensy.h"
#include "srobot.h"

using namespace std;

/**
 * Class to receive the motor encoder values.
 * */
class SEncoder
{
public:
  int tn = 0;
  /** setup and request data */
  void setup(int teensy_Number);
  /**
   * regular update tick */
  void tick();
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(const char * msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();
  /**
   * terminate */
  void flush();

public:
//   mutex dataLock; // ensure consistency
  int updatePosCnt = 0;
  int updateVelCnt = 0;
  int updatePoseCnt = 0;
  UTime encTime, encTimeLast;
  UTime encVelTime;
  UTime logTime;
  int64_t enc[SRobot::MAX_MOTORS] = {0}; /// ticks
  float vel[SRobot::MAX_MOTORS] = {0}; /// rad/s
  float pose[4]; /// x, y, h, tilt
  UTime poseTime;
private:
  std::string ini_section;
  void toLogEnc();
  void toLogPose();
  int64_t encLast[SRobot::MAX_MOTORS] = {0};
  bool firstEnc = true;
  bool encoder_reversed = true;
  bool toConsole = false;
  FILE * logfileEnc = nullptr;
  FILE * logfilePose = nullptr;
  //   std::condition_variable_any nd; // new data service
  // MQTT
  /// topic string for encoder position
  std::string topicEnc;
  /// topic string for encoder velocity - estimate from Teensy
  std::string topicEncVel;
  /// topic string for pose
  std::string topicPose;
  //
  void subscribeDataFromTeensy();
  int sub_pose = 0;
  int sub_vel = 0;
  UTime sub_pose_t;
  UTime sub_vel_t;
};

/**
 * Make this visible to the rest of the software */
extern SEncoder encoder[NUM_TEENSY_MAX];

#endif
