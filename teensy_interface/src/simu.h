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


#ifndef SIMU_H
#define SIMU_H

#include "utime.h"
#include "steensy.h"

using namespace std;

/**
 * Class to receive the raw data from the IMU (MPU9250)
 * */
class SImu
{
public:
  int tn = 0;
  /** setup and request data */
  void setup(int teensyNumber);
  /**
   * regular update tick */
//   void tick();
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(const char * msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();
  /**
   * start calibration of gyro offset */
  void calibrateGyro();
  /**
   * ensure data flow */
  void tick();

public:
//   mutex dataLock; // ensure consistency
  int updateAccCnt[2] = {0}; // ACC updates
  int updateGyroCnt[2] = {0}; // gyro updates
  UTime updTimeGyro[2];
  UTime updTimeAcc[2];
  float gyro[2][3] = {{0}};
  float gyroOffset[2][3] = {{0}};
  float acc[2][3] = {{0}};
  bool inCalibration[2] = {false};

private:
  std::string ini1, ini2;
  /** save to logfile (and/or console)
   * \param accChanged if new data is from accelerometer, else it is gyro */
  void toLog(bool accChanged, int imuIdx);
  //
  FILE * logfileGyro[2] = {nullptr};
  FILE * logfileAcc[2] = {nullptr};
  bool toConsoleAcc[2] = {false};
  bool toConsoleGyro[2] = {false};
  // calibration values for calibration
  const static int calibCountMax = 200;
  int calibCount[2] = {0};
  float calibSum[2][3] = {{0}};
  // MQTT
  std::string topicAcc[2];
  std::string topicGyro[2];
  //
  void subscribeDataFromTeensy();
  int sub_gyro = 0;
  int sub_acc = 0;
  // UTime sub_gyro_t;
  // UTime sub_acc_t;
  int sub_gyro2 = 0;
  int sub_acc2 = 0;
  // UTime sub_gyro2_t;
  // UTime sub_acc2_t;
};

/**
 * Make this visible to the rest of the software */
extern SImu imu[NUM_TEENSY_MAX];

#endif
