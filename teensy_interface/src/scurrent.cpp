/*  
 * 
 * Copyright © 2024 DTU,
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
#include <math.h>
#include "sencoder.h"
#include "scurrent.h"
#include "steensy.h"
#include "uservice.h"
#include "mvelocity.h"
#include "cmixer.h"
#include "umqtt.h"
#include "srobot.h"

// create value
SCurrent current[NUM_TEENSY_MAX];


void SCurrent::setup(int teensy_number)
{ // ensure there is default values in ini-file
  tn = teensy_number;
  ini_section = "current_teensy_" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // motor block is OK, but control parameters are needed too
    // log motor voltage and current (from Teensy)
    ini[ini_section]["log"] = "false";
    ini[ini_section]["print"] = "false";
    //
    // update times for motor control
    ini[ini_section]["interval_ms"] = "33"; // ms
  }
  //
  // mqtt
  toConsole = ini[ini_section]["print"] == "true";
  // subscribe interval
  sub_mca = strtol(ini[ini_section]["interval_ms"].c_str(), nullptr, 10);
  // initialize logfile
  if (ini[ini_section]["log"] == "true" and logfile == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + to_string(tn) + "_motor_current.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Motor current for all motors (Teensy %d)\n", tn);
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2-5 \tVoltage current (from Teensy) 1,2,(3,4) (Amps)\n");
    fprintf(logfile, "%% 6 \tBoard current (Amps) (low pass filtered)\n");
    fprintf(logfile, "%% 7 \tBoard current (Amps) (not filtered)\n");
  }
}

void SCurrent::subscribeDataFromTeensy()
{
  if (sub_mca > 0)
  {
    string ts = "sub mca " + to_string(sub_mca) + "\n";
    teensy[tn].send(ts.c_str());
    service.logMessage("# SCurrent: subscribed to Teensy data");
    sub_mca_t.now();
  }
}


void SCurrent::terminate()
{
  if (logfile != nullptr)
    fclose(logfile);
}

bool SCurrent::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "mca ", 4) == 0)
  { // motor current
    p1 += 4;
    for (int i = 0; i < 5; i++)
      current[i] = strtof(p1, (char**)&p1);
    // save to log_encoder_pose
    toLog(msgTime);
    sub_mca_t.now();
  }
  else if (strncmp(p1, "sca ", 4) == 0)
  { // motor current
    p1 += 4;
    supplyCurrent = strtof(p1, (char**)&p1);
    // save to log_encoder_pose
    toLog(msgTime);
    sub_sca_t.now();
  }
  else
    used = false;
  return used;
}

void SCurrent::toLog(UTime & updt)
{
  if (logfile != nullptr and not service.stop_logging)
  {
    fprintf(logfile, "%lu.%04ld %.3f %.3f %.3f %.3f %.2f %.2f\n",
            updt.getSec(), updt.getMicrosec()/100,
            current[0], current[1], current[2], current[3], robot[tn].supplyCurrent, supplyCurrent);
  }
}

void SCurrent::tick()
{
  // printf("# Current tick, interval=%d time=%f\n", sub_mca, sub_mca_t.getTimePassed());
  if (sub_mca > 0 and sub_mca_t.getTimePassed() > 5)
    subscribeDataFromTeensy();
}


