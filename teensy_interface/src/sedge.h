/*  
 * 
 * Copyright © 2022 DTU, Christian Andersen jcan@dtu.dk
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

#include "steensy.h"

using namespace std;

/**
 * Class for general state of the robot
 * e.g. battery voltage */
class SEdge
{
public:
  int tn = 0;
  /** setup and request data */
  void setup(int teensyNumber);
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(const char * msg, UTime & msgTime);
  /**
   * runs the thread  */
  void run();
  /**
   * terminate */
  void terminate();
  /**
   * update stream check */
  void tick();

public:
  /// system time at this Teensy time
  UTime updTime;
  // line sensor values
  int ad[8];
  // line sensor normalized values
  int adn[8];

private:
  /**
   * save current state to log */
  void toLogEnc();
  void toLogNormalized();
  /// Config file entry
  std::string ini_section;
  std::string tnGroup;  // teensy group in ini-file
  /// Log flags
  bool toConsole = false;
  FILE * logfileAD = nullptr;
  FILE * logfileN = nullptr;
  /// MQTT
  std::string topic;
  //
  void subscribeDataFromTeensy();
  int sub_liv = 0;
  int sub_livn = 0;
  UTime sub_liv_t;
  UTime sub_livn_t;
};

/**
 * Make this visible to the rest of the software */
extern SEdge edge[NUM_TEENSY_MAX];

