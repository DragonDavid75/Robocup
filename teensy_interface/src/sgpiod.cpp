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
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>
#include "uservice.h"
#include "sgpiod.h"
#include "srobot.h"

// inspired from https://github.com/brgl/libgpiod/blob/master/bindings/cxx/gpiod.hpp
#include "gpiod.h"

using namespace std::chrono;

// create value
SGpiod gpio;


void SGpiod::setup()
{ // ensure default values
  if (not ini.has("gpio"))
  { // no data yet, so generate some default values
    ini["gpio"]["pins_out"] = "12=0 16=0"; // give value to output pins
    ini["gpio"]["stop_on_stop"] = "true";
    ini["gpio"]["blink_period_ms"] = "600"; // ms
    ini["gpio"]["log"] = "true";
    ini["gpio"]["print"] = "false";
    ini["gpio"]["use"] = "false";
  }
  if (chip != nullptr)
  {
    if (ini["gpio"]["use"] == "true")
      chip = gpiod_chip_open_by_name("gpiochip0");
    if (chip != nullptr)
    { // set output ports
      // set output pins as specified
      int out_pin_value[MAX_PINS] = {0}; /// default value
      const char * p1 = ini["gpio"]["pins_out"].c_str();
      while (*p1 >= ' ')
      { // set output pins and initial value
        int pin = strtol(p1, (char**)&p1, 10);
        int v = 0;
        int idx = getPinIndex(pin);
        if (idx >= 0)
        {
          while (*p1 == ' ' and *p1 != '\0') p1++;
          if (*p1 == '\0')
            break;
          if (*p1 == '=')
            v = strtol(++p1, (char**)&p1, 10);
          else
          {
            printf("# SGpiod::setup: format 'pins_out=[ P=V]*' P=pin number, V=0|1 (found:%s)\n", ini["gpio"]["pins_out"].c_str());
            break;
          }
          out_pinuse[idx] = true;
          out_pin_value[idx] = v;
        }
        else
          printf("# SGpio::setup: found bad pin number in pin_out (%d)\n", pin);
      }
      // ignore first pin (start), handled by ip_disp
      for (int i = 0; i < MAX_PINS; i++)
      { // get handle to relevant pins and set output as specified
        pins[i] = gpiod_chip_get_line(chip, pinNumber[i]);
        int err = -1;
        int loop = 0;
        if (out_pinuse[i])
        {
          err = -1;
          while (err == -1)
          {
            err = gpiod_line_request_output(pins[i], "raubase_out", 0);
            if (err == -1)
              usleep(3333);
            if (loop++ > 10)
            { // failed to rerserve GPIO
              printf("# SGpio:: *********** failed to reserve GPIO pin %d\n", pinNumber[i]);
            }
          }
          setPin(pinNumber[i], out_pin_value[i]);
        }
        else
        {
          // default is input
          err = -1;
          while (err == -1)
          {
            err = gpiod_line_request_input(pins[i], "raubase_in");
            if (err == -1)
              usleep(3333);
            else
            {
              err = gpiod_line_set_flags(pins[i], GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN);
              if (err != 0)
                printf("#SGpio:: set line pull-down failed\n");
            }
            if (loop++ > 10)
            { // failed to rerserve GPIO
              printf("# SGpio:: *********** failed to reserve GPIO pin %d\n", pinNumber[i]);
            }
          }
        }
      }
    }
    else
    {
      if (ini["gpio"]["use"] == "true")
        printf("# SGpiod::setup there is no GPIO chip found - disable function\n");
      else
        printf("# SGpiod::setup GPIO disabled (in robot.ini)\n");
    }
  }
  // logfiles
  toConsole = ini["gpio"]["print"] == "true";
  if (chip != nullptr)
  {
    if (ini["gpio"]["log"] == "true" and logfile == nullptr)
    { // open logfile
      std::string fn = service.logPath + "log_gpio.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% gpio logfile\n");
      fprintf(logfile, "%% pins_out %s\n", ini["gpio"]["pins_out"].c_str());
      fprintf(logfile, "%% 1 \tTime (sec)\n");
  //     fprintf(logfile, "%% 2 \tPin %d (start)\n", pinNumber[0]);
      fprintf(logfile, "%% 2 \tPin %2d (stop)\n", pinNumber[0]);
      fprintf(logfile, "%% 3 \tPin %d\n", pinNumber[1]);
      fprintf(logfile, "%% 4 \tPin %d\n", pinNumber[2]);
      fprintf(logfile, "%% 5 \tPin %d\n", pinNumber[3]);
      fprintf(logfile, "%% 6 \tPin %d\n", pinNumber[4]);
      fprintf(logfile, "%% 7 \tPin %d\n", pinNumber[5]);
      fprintf(logfile, "%% 8 \tPin %d\n", pinNumber[6]);
    }
    if (not service.stop and th1 == nullptr)
      // start listen to the keyboard
      th1 = new std::thread(runObj, this);
  }
}

void SGpiod::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
  try
  {
    if (chip != nullptr)
    {
      for (int i = 0; i < MAX_PINS; i++)
        gpiod_line_release(pins[i]);
    }
  }
  catch (...)
  {
    printf("#### SGPIO had a pin-release error\n");
  }
}


int SGpiod::getPinIndex(int pinNumber)
{ //   int pinNumber[MAX_PINS] = {13, 6, 12, 16, 19, 26, 21, 20};
  int result = -1;
  switch (pinNumber)
  {
//     case 13: result = 0; break;
    case  6: result = 0; break;
    case 12: result = 1; break;
    case 16: result = 2; break;
    case 19: result = 3; break;
    case 26: result = 4; break;
    case 21: result = 5; break;
    case 20: result = 6; break;
    default:
      break;
  }
  return result;
}

int SGpiod::readPin(const int pin)
{
  int idx = getPinIndex(pin);
  int val = -1;
  if (chip != nullptr)
  {
    if (idx >= 0)
      val = gpiod_line_get_value(pins[idx]);
    else
      printf("# SGpiod::readPin: pin %d is not valid, use one of: %d %d %d %d %d %d %d\n", pin,
          pinNumber[0], pinNumber[1], pinNumber[2], pinNumber[3], pinNumber[4], pinNumber[5],
          pinNumber[6]);
  }
  return val;
}

void SGpiod::setPin(const int pin, bool value)
{ // set one pin to value
  int idx = getPinIndex(pin);
  if (chip != nullptr)
  {
    if (idx >= 0 and out_pinuse[idx])
      gpiod_line_set_value(pins[idx], value);
    else
    { // not valid
      printf("# SGpiod::setPin: pin %d (idx=%d) is not set as output (in robot.ini) or invalid - call ignored\n", pin, idx);
    }
  }
}


void SGpiod::run()
{
  bool pv[MAX_PINS] = {false};
  bool changed = true;
  int loop = 0;
  bool stopSwitchPressed = false;
  auto sampleTime =  1ms;
  auto loopTime = std::chrono::steady_clock::now() + sampleTime;
  while (not service.stop and chip != nullptr)
  {
    loop++;
    changed = false;
    for (int i = 0; i < MAX_PINS; i++)
    {
      pv[i] = readPin(pinNumber[i]);
      if (loop < 100)
        // make sure we don't detect a power-on event (first 100ms)
        in_pin_value[i] = pv[i];
      else if (pv[i] != in_pin_value[i])
      {
        in_pin_value[i] = pv[i];
        changed = true;
        // debug
        // printf("# SGpio:: pin %d(%d) changed (%d)\n",
        //        i, pinNumber[i], pv[i]);
        // debug end
        if (i == 0 and
            pv[i]==1 and
            ini["gpio"]["stop_on_stop"] == "true")
        { // stop switch
          stopSwitchPressed = true;
        }
      }
    }
    if (changed or loop %20 == 0)
      toLog(pv);
    // terminate app
    if (stopSwitchPressed)
    {
      service.stopNow("stop_switch");
      stopSwitchPressed = false;
    }
    //
    std::this_thread::sleep_until(loopTime);
    loopTime += sampleTime;
  }
}

int SGpiod::wait4Pin(int pin, uint timeout_ms, int wait4Value)
{
  int value = -1;
  UTime t("now");
  while (true and chip != nullptr)
  {
    int v = readPin(pin);
    if (v == wait4Value)
    {
      value = v;
      break;
    }
    usleep(500);
    float s = t.getTimePassed();
    float w = float(timeout_ms)/1000;
    if (s > w)
      break;
  }
  return value;
}

void SGpiod::toLog(bool pv[])
{ // pv is pin-value
  if (service.stop)
    return;
  UTime t("now");
  if (logfile != nullptr and not service.stop_logging)
  {
    fprintf(logfile,"%lu.%04ld %d %d %d %d %d %d %d\n",
            t.getSec(), t.getMicrosec()/100,
            pv[0], pv[1], pv[2], pv[3], pv[4], pv[5], pv[6]);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %d %d %d %d %d %d\n",
            t.getSec(), t.getMicrosec()/100,
            pv[0], pv[1], pv[2], pv[3], pv[4], pv[5], pv[6]);
  }
}
