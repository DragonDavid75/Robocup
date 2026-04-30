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

#include <string>
#include <string.h>
#include "cservo.h"
#include "steensy.h"
#include "uservice.h"
// create value
CServo servo[NUM_TEENSY_MAX];


void CServo::setup(int teensyNumber)
{ // ensure default values
  tn = teensyNumber;
  ini_section = "servoTn" + std::to_string(tn);
  if (not ini.has(ini_section))
  { // no data yet, so generate some default values
    ini[ini_section]["interval_ms"] = "50";
    ini[ini_section]["log"] = "true";
    ini[ini_section]["print"] = "true";
  }
  // Subscribe interval
  sub_svo = strtol(ini[ini_section]["interval_ms"].c_str(), nullptr, 10);
  // debug print
  toConsole = ini[ini_section]["print"] == "true";
  // set servo
  if (ini[ini_section]["log"] == "true" and logfile == nullptr)
  { // open logfile for servo data from Teensy
    std::string fn = service.logPath + "log_t" + to_string(tn) + "_servo.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Servo logfile (Teensy %d)\n", tn);
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tServo number\n");
    fprintf(logfile, "%% 3 \tRequested position\n");
    fprintf(logfile, "%% 4,5,6 \tEnabled, position, velocity\n");
  }
}

void CServo::subscribeDataFromTeensy()
{
  if (sub_svo > 0)
  {
    string ts = "sub svo " + to_string(sub_svo) + "\n";
    teensy[tn].send(ts.c_str());
    service.logMessage("# CServo: subscribed to Teensy data");
    updTime.now();
  }
}

void CServo::setServo(int servo, bool enabled, int position, int velocity)
{
  const int MSL = 100;
  char s[MSL];
  // UTime t("now");
  if (servo > 0 and servo <= MAX_SERVO_CNT)
  {
    servo_ref[servo - 1] = position;
    if (enabled)
      snprintf(s, MSL, "servo %d %d %d\n", servo, position, velocity);
    else
      snprintf(s, MSL, "servo %d 10000 0\n", servo);
    teensy[tn].send(s);
    toLog(servo - 1);
  }
  else
    printf("# CServo:: unknown servo %d (range 1 to %d)\n", servo, MAX_SERVO_CNT);
}

void CServo::terminate()
{
  if (logfile != nullptr and not service.stop_logging)
  {
    fclose(logfile);
  }
}

bool CServo::decode(const char* msg, UTime & msgTime)
{ // return message from Teensy
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "svo ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    for (int i = 0; i < MAX_SERVO_CNT; i++)
    {
      servo_enabled[i] = strtol(p1, (char**)&p1, 10);
      servo_position[i] = strtol(p1, (char**)&p1, 10);
      servo_velocity[i] = strtol(p1, (char**)&p1, 10);
      toLog(i);
    }
    // notify users of a new update
    updTime = msgTime;
    updateCnt++;
  }
  else
    used = false;
  return used;
}

void CServo::toLog(int i)
{
  if (logfile != nullptr and not service.stop and not service.stop_logging)
  {
    if (servo_old_position[i] != servo_position[i])
    {
      servo_old_position[i] = servo_position[i];
      fprintf(logfile, "%lu.%03ld %d %d %d %d %d\n",
              updTime.getSec(), updTime.getMillisec(),
              i,
              servo_ref[i],
              servo_enabled[i], servo_position[i], servo_velocity[i]
              // ,
              // servo_enabled[1], servo_position[1], servo_velocity[1],
              // servo_enabled[2], servo_position[2], servo_velocity[2],
              // servo_enabled[3], servo_position[3], servo_velocity[3],
              // servo_enabled[4], servo_position[4], servo_velocity[4]
      );
    }
  }
}

void CServo::tick()
{
  if (sub_svo > 0 and updTime.getTimePassed() > 5)
    subscribeDataFromTeensy();
}
