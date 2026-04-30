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
class SRobot
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
   * Set robotname and save it to a file */
  void setRobotName(const char * toName);
  /**
   * ensure data flow */
  void tick();

public:
  /// number of motor controllers on PCB
  static const int MAX_MOTORS = 2;
  /// Battery voltage with a few decimals (~2 valid decimals)
  float batteryVoltage;
  /// Teensy time since start of Teensy
  double teensyTime;
  /// robot hardware index number (serial)
  int idx = 0;
  /// robot hardware version
  int version = 0;
  /// control state
  int controlState = 0;
  /// Teensy load
  float load = 0;
  /// motor enabled state
  float supplyCurrent = 0;
  const float fullyChargedVoltage = 17.5;
  bool fullyCharged = false;
  double batteryUsedWh = 0;
  float batteryScale = 1.0;
  int shutdown_count = 0;
  /// robot hardware type
  int type = 0;
  /// system time at this Teensy time
  UTime hbtTime;
  /// mutex should be used to get consistent values
  std::mutex dataLock;
private:
  bool findIPs();
  bool updateIPlist();
  float measureCPUtemp();
  std::string exec(std::string cmd);
  std::string robotName;
  /// State monitor
  int lastIpCnt = 0;
  static const int MHL = 100;
  static const int MIPS = 7;
  char ips[MIPS][MHL] = {{'\0'}}; // list of IP strings
  //     char macs[MIPS][MHL] = {{'\0'}}; // list of MAC strings
  int ipsCnt = 0;
  //     int macCnt = 0;
  static const int MHL2 = 150;
  char ip4list[MHL2];
  char ip4listLast[MHL2];
  char maclist[MHL2];
  float cpuTemp = 0;
  std::thread * th1 = nullptr;
  bool ip_sentToTeensy = false;
  static void runObj(SRobot * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  /**
   * save current state to log */
  void toLog();
  /// Config file entry
  std::string ini_section;
  std::string tnGroup;  // teensy group in ini-file
  /// Log flags
  bool toConsole = false;
  FILE * logfile = nullptr;
  /// MQTT
  std::string topicHbt;
  UTime lastHbt;
  //
  void subscribeDataFromTeensy();
  bool setupOK = false;
};

/**
 * Make this visible to the rest of the software */
extern SRobot robot[NUM_TEENSY_MAX];

