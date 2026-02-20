/*  
 * 
 * Copyright © 2022 DTU, 
 * Author:
 * Christian Andersen jcan@dtu.dk
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

#include <string>
#include <string.h>
#include <inttypes.h>
#include "simu.h"
#include "steensy.h"
#include "uservice.h"
#include <stdlib.h>
#include "umqtt.h"
// create value
SImu imu[NUM_TEENSY_MAX];


void SImu::setup(int teensyNumber)
{ // ensure default values
  tn = teensyNumber;
  ini1 = "imu1teensy" + std::to_string(tn);
  ini2 = "imu2teensy" + std::to_string(tn);
  if (not ini.has(ini1))
  { // no data yet, so generate some default values
    // first IMU
    ini[ini1]["interval_gyro_ms"] = "12";
    ini[ini1]["interval_acc_ms"] = "12";
    ini[ini1]["gyro_offset"] = "0 0 0";
    ini[ini1]["log"] = "true";
    ini[ini1]["print_gyro"] = "false";
    ini[ini1]["print_acc"] = "false";
    // other IMU
    // ini[ini2]["use"] = "false";
    // ini[ini2]["interval_gyro_ms"] = "12";
    // ini[ini2]["interval_acc_ms"] = "12";
    // ini[ini2]["gyro_offset"] = "0 0 0";
    // ini[ini2]["log"] = "true";
    // ini[ini2]["print_gyro"] = "false";
    // ini[ini2]["print_acc"] = "false";
  }
  sub_gyro = strtol(ini[ini1]["interval_gyro_ms"].c_str(), nullptr, 10);
  sub_acc = strtol(ini[ini1]["interval_acc_ms"].c_str(), nullptr, 10);
  // use values and subscribe to source data
  // like teensy[x].send("sub pose 4\n");
  // std::string s = "sub gyro " + ini[ini1]["interval_gyro_ms"] + "\n";
  // teensy[tn].send(s.c_str());
  // s = "sub acc " + ini[ini1]["interval_acc_ms"] + "\n";
  // teensy[tn].send(s.c_str());
  // gyro offset
  const char * p1 = ini[ini1]["gyro_offset"].c_str();
  gyroOffset[0][0] = strtof(p1, (char**)&p1);
  gyroOffset[0][1] = strtof(p1, (char**)&p1);
  gyroOffset[0][2] = strtof(p1, (char**)&p1);
  // send calibration values to Teensy
  //
  toConsoleGyro[0] = ini[ini1]["print_gyro"] == "true";
  toConsoleAcc[0] = ini[ini1]["print_acc"] == "true";
  if (ini[ini1]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_t" + to_string(tn) + "_gyro_1.txt";
    logfileGyro[0] = fopen(fn.c_str(), "w");
    fprintf(logfileGyro[0], "%% Gyro logfile (IMU1)\n");
    fprintf(logfileGyro[0], "%% 1 \tTime (sec)\n");
    fprintf(logfileGyro[0], "%% 2-4 \tGyro (x,y,z)\n");
    fprintf(logfileGyro[0], "%% Gyro offset %g %g %g\n", gyroOffset[0][0], gyroOffset[0][1], gyroOffset[0][2]);
    //
    fn = service.logPath + "log_t" + to_string(tn) + "_acc_1.txt";
    logfileAcc[0] = fopen(fn.c_str(), "w");
    fprintf(logfileAcc[0], "%% Accelerometer logfile (IMU1)\n");
    fprintf(logfileAcc[0], "%% 1 \tTime (sec)\n");
    fprintf(logfileAcc[0], "%% 2-4 \tAccelerometer (x,y,z)\n");
  }
  // other IMU
  if (ini[ini2]["use"] == "true")
  {
    // s = "sub gyro2 " + ini[ini2]["interval_gyro_ms"] + "\n";
    // teensy[tn].send(s.c_str());
    // s = "sub acc2 " + ini[ini2]["interval_acc_ms"] + "\n";
    // teensy[tn].send(s.c_str());
    // gyro offset
    p1 = ini[ini2]["gyro_offset"].c_str();
    gyroOffset[1][0] = strtof(p1, (char**)&p1);
    gyroOffset[1][1] = strtof(p1, (char**)&p1);
    gyroOffset[1][2] = strtof(p1, (char**)&p1);
    toConsoleGyro[1] = ini[ini2]["print_gyro"] == "true";
    toConsoleAcc[1] = ini[ini2]["print_acc"] == "true";
    if (ini[ini2]["log"] == "true" and logfileGyro[1] == nullptr)
    { // open logfile
      std::string fn = service.logPath + "log_t" + to_string(tn) + "_gyro_2.txt";
      logfileGyro[1] = fopen(fn.c_str(), "w");
      fprintf(logfileGyro[1], "%% Gyro logfile (IMU2)\n");
      fprintf(logfileGyro[1], "%% 1 \tTime (sec)\n");
      fprintf(logfileGyro[1], "%% 2-4 \tGyro (x,y,z)\n");
      fprintf(logfileGyro[1], "%% Gyro offset %g %g %g\n", gyroOffset[1][0], gyroOffset[1][1], gyroOffset[1][2]);
      //
      fn = service.logPath + "log_t" + to_string(tn) + "_acc_2.txt";
      logfileAcc[1] = fopen(fn.c_str(), "w");
      fprintf(logfileAcc[1], "%% Accelerometer logfile (IMU2)\n");
      fprintf(logfileAcc[1], "%% 1 \tTime (sec)\n");
      fprintf(logfileAcc[1], "%% 2-4 \tAccelerometer (x,y,z)\n");
    }
  }
}


void SImu::subscribeDataFromTeensy()
{
  if (sub_gyro > 0)
  {
    std::string s = "sub gyro " + to_string(sub_gyro) + "\n";
    teensy[tn].send(s.c_str());
  }
  if (sub_acc > 0)
  {
    std::string s = "sub acc " + to_string(sub_acc) + "\n";
    teensy[tn].send(s.c_str());
  }
  if (ini[ini2]["use"] == "true")
  {
    if (sub_gyro > 0)
    {
      std::string s = "sub gyro2 " + to_string(sub_gyro) + "\n";
      teensy[tn].send(s.c_str());
    }
    if (sub_acc > 0)
    {
      std::string s = "sub acc2 " + to_string(sub_acc) + "\n";
      teensy[tn].send(s.c_str());
    }
  }
  if (sub_gyro > 0 or sub_acc > 0)
  {
    service.logMessage("# SImu: subscribed to Teensy data (gyro and acc)");
    int m = 0;
    updTimeGyro[m].now();
    updTimeAcc[m].now();
  }
}

void SImu::terminate()
{
  for (int m = 0; m < 2; m++)
  {
    if (logfileAcc[m] != nullptr)
    {
      fclose(logfileAcc[m]);
      logfileAcc[m] = nullptr;
    }
    if (logfileGyro[m] != nullptr)
    {
      fclose(logfileGyro[m]);
      logfileGyro[m] = nullptr;
    }
  }
}

bool SImu::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "acc ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    float a[3];
    for (int i = 0; i < 3; i++)
      a[i] = strtof(p1, (char**)&p1);
    // IMU 1 (pt. one only)
    int m = 0;
    updTimeAcc[m] = msgTime;
    for (int i = 0; i < 3; i++)
      acc[m][i] = a[i];
    updateAccCnt[m]++;
    // save to log
    toLog(true, m);
  }
  else if (strncmp(p1, "gyro ", 5) == 0)
  {
    if (strlen(p1) > 5)
      p1 += 5;
    else
      return false;
    // get x,y and z values
    float g[3];
    for (int i = 0; i < 3; i++)
      g[i] = strtof(p1, (char**)&p1);
    // IMU number (there is one gyro only)
    int m = 0;
    updTimeGyro[m] = msgTime;
    for (int i = 0; i < 3; i++)
    {
      gyro[m][i] = g[i] - gyroOffset[m][i];
    }
    // notify users of a new update
    updateGyroCnt[m]++;
     // save to log (if requested)
    toLog(false, m);
    //
    if (inCalibration[m])
    { // Gyro calibration can be handled ambulant
      for (int j = 0; j < 3; j++)
        calibSum[m][j] = g[j];
      calibCount[m]++;
      printf("# gyro %d, %d : %g %g %g\n", m, calibCount[m], calibSum[m][0]/float(calibCount[m]), calibSum[m][1]/float(calibCount[m]), calibSum[m][2]/float(calibCount[m]));
      if (calibCount[m] >= calibCountMax)
      {
        for (int j = 0; j < 3; j++)
          gyroOffset[m][j] = calibSum[m][j]/calibCount[m];
        // implement new values
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "%g %g %g", gyroOffset[m][0], gyroOffset[m][1], gyroOffset[m][2]);
        if (m == 0)
          ini[ini1]["gyro_offset"] = s;
        else
          ini[ini2]["gyro_offset"] = s;
        inCalibration[m] = false;
        //
        printf("# gyro %d calibration finished: %s\n", m, s);
      }
    }
  }
  else
    used = false;
  return used;
}

void SImu::toLog(bool accChanged, int imuIdx)
{
  if (service.stop)
    return;
  int m = imuIdx;
  if (accChanged)
  { // accelerometer
    if (logfileAcc[m] != nullptr and not service.stop_logging)
    {
      fprintf(logfileAcc[m],"%lu.%04ld %.4f %.4f %.4f\n", updTimeAcc[m].getSec(), updTimeAcc[m].getMicrosec()/100,
              acc[m][0], acc[m][1], acc[m][2]);
    }
    if (toConsoleAcc[m])
    {
      printf("%lu.%04ld %.4f %.4f %.4f Acc %d\n", updTimeAcc[m].getSec(), updTimeAcc[m].getMicrosec()/100,
             acc[m][0], acc[m][1], acc[m][2], imuIdx);
    }
  }
  else
  { // gyro data
    if (logfileGyro[m] != nullptr and not service.stop_logging)
    {
      fprintf(logfileGyro[m],"%lu.%04ld %.4f %.4f %.4f\n", updTimeGyro[m].getSec(), updTimeGyro[m].getMicrosec()/100,
              gyro[m][0], gyro[m][1], gyro[m][2]);
    }
    if (toConsoleGyro[m])
    {
      printf("%lu.%04ld %.4f %.4f %.4f Gyro %d\n", updTimeGyro[m].getSec(), updTimeGyro[m].getMicrosec()/100,
              gyro[m][0], gyro[m][1], gyro[m][2], imuIdx);
    }
  }
}

void SImu::calibrateGyro()
{ // calibrate both
  inCalibration[0] = true;
  inCalibration[1] = true;
}

void SImu::tick()
{
  int m = 0;
  // printf("# Imu tick, interval=%d time=%f\n", sub_gyro, updTimeGyro[m].getTimePassed());
  if ((sub_gyro > 0 and updTimeGyro[m].getTimePassed() > 2.5) or (sub_acc > 0 and updTimeAcc[m].getTimePassed() > 2.5))
  {
    subscribeDataFromTeensy();
  }
}

