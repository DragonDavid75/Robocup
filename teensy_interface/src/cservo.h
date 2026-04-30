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


#ifndef CSERVO_H
#define CSERVO_H

#include "utime.h"
#include "steensy.h"

using namespace std;

/**
 * Class to control the servos through the Teensy connection.
 * The servo position, as far as known by the Teensy, is also available.
 * */
class CServo
{
public:
  /** setup and request data */
  void setup(int teensyNumber);
  /**
   * Set one servo position
   * \param servo is the servo number 1,2,3,4 or 5
   * \param enabled 1=enabled, 0=disabled (works on SAVOX servos only)
   * \param position is servo position from -500 to +500
   * \param velocity is number of servo units per second (0, 1..1000) (0 = as fast as possible)
   * */
  void setServo(int servo, bool enabled, int position=0, int velocity = 0);
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(const char * msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();
  /**
   * called regularly to ensure data source */
  void tick();

public:
  int tn = 0;
  int updateCnt = false;
  UTime updTime, updTimeLast;
  static const int MAX_SERVO_CNT = 3;
  int servo_ref[MAX_SERVO_CNT];
  int servo_enabled[MAX_SERVO_CNT];
  int servo_position[MAX_SERVO_CNT];
  int servo_old_position[MAX_SERVO_CNT] = {-1,-1,-1};
  int servo_velocity[MAX_SERVO_CNT];

private:
  void subscribeDataFromTeensy();
  int sub_svo;
  //
  std::string ini_section;
  void toLog(int i);
  // debug print to console
  bool toConsole = false;
  FILE * logfile = nullptr;
};

/**
 * Make this visible to the rest of the software */
extern CServo servo[NUM_TEENSY_MAX];

#endif
