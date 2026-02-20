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

#include "sencoder.h"
#include "utime.h"

/**
 * Class is to monitor motor current.
 * */
class SCurrent
{
public:
  /** setup and request data */
  void setup(int teensy_number);
  /**
   * Decode messages from Teensy */
  bool decode(const char* msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();
  /**
   * ensure data flow */
  void tick();

protected:
  /**
   * Save motor values for all motors */
  void toLog(UTime & updt);
  void subscribeDataFromTeensy();
  //
public:
  UTime updTime; // time of last control update
  float current[5] = {0}; // index 4 is system current
  float supplyCurrent = 0; // less averaged than value in HBT
  int dataCnt = 0;

private:
  /// number of this teensy
  int tn = 0;
  string ini_section;
  /// private stuff
  void logfileLeadText(FILE * f, const char * ms);
  // support variables
  FILE * logfile = nullptr;
  bool toConsole;
  // mqtt
  std::string topicMotv;
  std::string topicCurrent;
  int sub_mca = 0;
  int sub_sca = 0;
  UTime sub_mca_t;
  UTime sub_sca_t;
};

/**
 * Make this visible to the rest of the software */
extern SCurrent current[NUM_TEENSY_MAX];

