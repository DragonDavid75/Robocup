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

using namespace std;

/**
 * Class to receive distance or force values from Dist1 and Dist2 plugs.
 * either a force sensor (RP-C18.3-LT from DFROBOT)
 * or a distance sensor (Sharp 2Y0A21)
 *
 * */
class SDistForce
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
public:
  float distance[2]; // meter (when IR sensor is calibrated)
  float force[2]; // estimated force (N)
  int forceAD[2]; // raw AD value
  UTime updTime;
  UTime logTime;
  int updateCnt = 0;
  bool sensorOn = false;

private:
  void calculateForce();

  std::string ini_section;
  void toLogDist();
  void toLogForce();
  bool toConsole = false;
  FILE * logfileDist = nullptr;
  FILE * logfileForce = nullptr;
  //   std::condition_variable_any nd; // new data service
  // MQTT
  /// topic string for encoder position
  std::string topicDist;
  std::string topicForce;
  //
  void subscribeDataFromTeensy();
  int sub_ird = 0;
};

/**
 * Make this visible to the rest of the software */
extern SDistForce distforce[NUM_TEENSY_MAX];

