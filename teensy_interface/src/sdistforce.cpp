/*  
 * 
 * Copyright © 2023 DTU,
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

/**
 * either a force sensor (RP-C18.3-LT from DFROBOT)
 * or a distance sensor (Sharp 2Y0A21)
 */

#include <string>
#include <string.h>
#include "sdistforce.h"
#include "steensy.h"
#include "uservice.h"
#include "umqtt.h"
// create value
SDistForce distforce[NUM_TEENSY_MAX];


void SDistForce::setup(int teensy_number)
{ // ensure default values
  tn = teensy_number;
  ini_section = "distforce" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // no data yet, so generate some default values
    ini[ini_section]["interval_ird_ms"] = "20";
    ini[ini_section]["log_dist"] = "false";
    ini[ini_section]["log_force"] = "true";
    ini[ini_section]["print"] = "false";
    ini[ini_section]["force"] = "true";
  }
  // reset encoder and pose
  topicDist = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/dist";
  topicForce = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/force";
  // use values and subscribe to source data
  // subscripe to ir distance that include raw AD values too
  // interval
  sub_ird = strtol(ini[ini_section]["interval_ird_ms"].c_str(), nullptr, 10);
  /// other debug feature
  toConsole = ini[ini_section]["print"] == "true";
  if (ini[ini_section]["log_dist"] == "true" and logfileDist == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_dist.txt";
    logfileDist = fopen(fn.c_str(), "w");
    fprintf(logfileDist, "%% IR distance logfile\n");
    fprintf(logfileDist, "%% 1 \tTime (sec)\n");
    fprintf(logfileDist, "%% 2-3 \tDistance sensor 1 and 2 (meter)\n");
    fprintf(logfileDist, "%% 4-5 \tRaw AD value for sensor 1 and 2\n");
    fprintf(logfileDist, "%% 6 \tSensor power on (1=on)\n");
  }
  if (ini[ini_section]["log_force"] == "true" and logfileForce == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_force.txt";
    logfileForce = fopen(fn.c_str(), "w");
    fprintf(logfileForce, "%% force (using distance sensor input) logfile\n");
    fprintf(logfileForce, "%% 1 \tTime (sec)\n");
    fprintf(logfileForce, "%% 2-3 \testimated force (1 and 2)\n");
    fprintf(logfileForce, "%% 4-5 \tRaw AD values (1 and 2)\n");
  }
}

void SDistForce::subscribeDataFromTeensy()
{
  if (sub_ird > 0)
  {
    string ts = "sub ird " + to_string(sub_ird) + "\n";
    teensy[tn].send(ts.c_str());
    service.logMessage("# SDistForce: subscribed to Teensy data");
    updTime.now();
  }
}


void SDistForce::terminate()
{
  if (logfileDist != nullptr)
  {
    fclose(logfileDist);
  }
  if (logfileForce != nullptr)
  {
    fclose(logfileForce);
  }
}

void SDistForce::calculateForce()
{ // force sensor is RP-C18.3-LT from DFROBOT
  // the resistance (in kOhm) is
  // R = 153 f^(-699) according to data sheet, f is in grams
  // log(R) = log(153) + log(f)*(-0.699)
  // log(f) = (log(R) - log(153))/-0.699
  //
  // AD measurement is from a voltage divider
  // from 5V using R + 7.5k over 15k
  // the AD has a reference of 3.3V and has 13 bit,
  // and is further averaged over 10 samples, i.e.
  // a max AD value of 81960
  // 1: V = 5 * 15/(R + 7.5 + 15)
  // 2: V = AD * 3.3 / 81960
  // 1: R = 15*5/V - 22.5
  // 1+2: R = 75*81960/(3.3*AD) - 22.5
  float r[2];
  for (int i = 0; i < 2; i++)
  { // calculate measured resistance
    r[i] = 75*81960 / (3.3 * float(forceAD[i])) - 22.5; // in kOhm
    // calculate log of force in grams
    float logF = (logf(r[i]) - logf(153.0))/-0.699;
    // convert to Newton
    force[i] = expf(logF) / 1000 * 9.82; // in Newton
  }
}


bool SDistForce::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "ir ", 3) == 0 or strncmp(p1, "ird ", 4) == 0)
  {
    p1 += 3;
    updTime = msgTime;
    /*double teensyTime = strtod(p1, (char**)&p1); */
    distance[0] = strtof(p1, (char**)&p1);
    distance[1] = strtof(p1, (char**)&p1);
    // forceAD[0] = strtoll(p1, (char**)&p1, 10);
    // forceAD[1] = strtoll(p1, (char**)&p1, 10);
    // sensorOn = strtoll(p1, (char**)&p1, 10);
    //
    if (ini[ini_section]["force"] == "true")
      calculateForce();
    // notify users of a new update
    updateCnt++;
    // save to log
    logTime = updTime;
    toLogDist();
    toLogForce();
  }
  else
    used = false;
  return used;
}


void SDistForce::toLogDist()
{
  if (not service.stop)
  {
    if (logfileDist != nullptr and not service.stop_logging)
    {
      fprintf(logfileDist,"%lu.%04ld %g %g %u %u %d\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              distance[0], distance[1], forceAD[0], forceAD[1], sensorOn);
    }
    if (toConsole and not (ini["mqtt"]["use"] == "true"))
    {
      printf("%lu.%04ld %g %g  %u %u %d\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              distance[0], distance[1], forceAD[0], forceAD[1], sensorOn);
    }
  }
}

void SDistForce::toLogForce()
{
  if (not service.stop)
  {
    if (logfileForce != nullptr and not service.stop_logging)
    {
      fprintf(logfileForce,"%lu.%04ld %g %g %u %u\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              force[0], force[1], forceAD[0], forceAD[1]);
    }
    if (toConsole and (ini["mqtt"]["use"] == "true"))
    {
      printf("%lu.%04ld %g %g %u %u\n",
              logTime.getSec(), logTime.getMicrosec()/100,
              force[0], force[1], forceAD[0], forceAD[1]);
    }
  }
}

void SDistForce::tick()
{
  if (sub_ird > 0 and updTime.getTimePassed() > 5)
    subscribeDataFromTeensy();
}
