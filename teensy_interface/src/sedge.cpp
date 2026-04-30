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
#include <math.h>

#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>
#include <unistd.h>
//#include <netpacket/packet.h>
//#include <linux/wireless.h>
//#include <sys/ioctl.h>


#include "steensy.h"
#include "sedge.h"
#include "uservice.h"
#include "umqtt.h"

// create the class with received info
SEdge edge[NUM_TEENSY_MAX];


void SEdge::setup(int teensyNumber)
{ /// subscribe to pose information
  tn = teensyNumber;
  ini_section = "edge" + std::to_string(tn);
  tnGroup = "teensy" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // no teensy group, so make one.
    ini[ini_section]["log"] = "true";
    ini[ini_section]["print"] = "false";
    ini[ini_section]["interval_liv_ms"] = "0";
    ini[ini_section]["interval_livn_ms"] = "10";
  }
  toConsole = ini[ini_section]["print"] == "true";
  // int rate = strtol(ini[ini_section]["interval_ms"].c_str(), nullptr, 10);
  // MQTT topic name
  topic = ini["mqtt"]["system"] + ini["mqtt"]["function"] + "T" + std::to_string(tn) + "/";
  // subscribe interval
  sub_liv = strtol(ini[ini_section]["interval_liv_ms"].c_str(), nullptr, 10);
  sub_livn = strtol(ini[ini_section]["interval_livn_ms"].c_str(), nullptr, 10);
  // logfile
  if (ini[ini_section]["log"] == "true" and logfileAD == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_edge_liv.txt";
    logfileAD = fopen(fn.c_str(), "w");
    fprintf(logfileAD, "%% Edge\n");
    fprintf(logfileAD, "%% 1 \tTime (sec)\n");
    fprintf(logfileAD, "%% 2-10 \tsensor AD value (0..4196), sensor 0 is left, AD=0 is no reflection\n");
  }
  if (ini[ini_section]["log"] == "true" and logfileN == nullptr)
  { // open logfile
    std::string fn = service.logPath + "log_t" + std::to_string(tn) + "_edge_livn.txt";
    logfileN = fopen(fn.c_str(), "w");
    fprintf(logfileN, "%% Edge\n");
    fprintf(logfileN, "%% 1 \tTime (sec)\n");
    fprintf(logfileN, "%% 2-10 \tsensor notmalized value (0..1000), sensor 0 is left, 0 is no reflection, 1000 is calibrated white\n");
  }
}

void SEdge::subscribeDataFromTeensy()
{
  if (sub_liv > 0)
  {
    std::string ss = "sub liv " + to_string(sub_liv);
    teensy[tn].send(ss.c_str());
  }
  if (sub_livn > 0)
  {
    std::string ss = "sub livn " + to_string(sub_livn);
    teensy[tn].send(ss.c_str());
  }
  if (sub_liv > 0 or sub_livn > 0)
  {
    service.logMessage("# SEdge: subscribed to Teensy data");
    sub_liv_t.now();
    sub_livn_t.now();
  }
}

void SEdge::terminate()
{
  if (logfileN != nullptr)
  {
    fclose(logfileN);
    logfileN = nullptr;
  }
  if (logfileAD != nullptr)
  {
    fclose(logfileAD);
    logfileAD = nullptr;
  }
}


bool SEdge::decode(const char* msg, UTime & msgTime)
{ // like: liv %ld %ld %ld %ld %ld %ld %ld %ld\r\n
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "liv ", 4) == 0)
  { // decode pose message
    // advance to first parameter
    if (strlen(p1) > 5)
      p1 += 4;
    else
      return false;
    // forward to mqtt
    updTime = msgTime;
    // get data
    for (int i = 0; i < 8; i++)
    {
      ad[i] = strtol(p1, (char **)&p1, 10);
    }
    toLogEnc();
    sub_liv_t.now();
  }
  else if (strncmp(p1, "livn ", 5) == 0)
  { // just publish - normalized values
    p1 += 5;
    // get data
    for (int i = 0; i < 8; i++)
    {
      adn[i] = strtol(p1, (char **)&p1, 10);
    }
    updTime = msgTime;
    toLogNormalized();
    sub_livn_t.now();
  }
  else
    used = false;
  return used;
}

void SEdge::toLogEnc()
{ // data is already locked
  if (service.stop)
    return;
  if (logfileAD != nullptr and not service.stop_logging)
  {
    fprintf(logfileAD, "%lu.%03ld %d %d %d %d %d %d %d %d\n",
            updTime.getSec(), updTime.getMillisec(),
            ad[0], ad[1], ad[2], ad[3], ad[4], ad[5], ad[6], ad[7]);
  }
  if (toConsole)
    printf("%lu.%03ld %d %d %d %d %d %d %d %d\n",
            updTime.getSec(), updTime.getMillisec(),
            ad[0], ad[1], ad[2], ad[3], ad[4], ad[5], ad[6], ad[7]);
}

void SEdge::toLogNormalized()
{ // data is already locked
  if (service.stop)
    return;
  if (logfileN != nullptr and not service.stop_logging)
  {
    fprintf(logfileN, "%lu.%03ld %d %d %d %d %d %d %d %d\n",
            updTime.getSec(), updTime.getMillisec(),
            adn[0], adn[1], adn[2], adn[3], adn[4], adn[5], adn[6], adn[7]);
  }
  if (toConsole)
    printf("%lu.%03ld %d %d %d %d %d %d %d %d\n",
           updTime.getSec(), updTime.getMillisec(),
           adn[0], adn[1], adn[2], adn[3], adn[4], adn[5], adn[6], adn[7]);
}

void SEdge::tick()
{
  if ((sub_liv > 0 and sub_liv_t.getTimePassed() > 3.5) or (sub_livn > 0 and sub_livn_t.getTimePassed() > 3.5))
    subscribeDataFromTeensy();
}
