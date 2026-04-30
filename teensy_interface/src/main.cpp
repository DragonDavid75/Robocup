/*
 #***************************************************************************
 #*   Copyright (C) 2024 by DTU
 #*   jcan@dtu.dk
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */

// System libraries
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <ostream>  // std::cout
#include <chrono>  // Time keeping
#include <thread>
// include local files for data values and functions
#include "uservice.h"
#include "cmixer.h"
#include "sgpiod.h"
#include "utime.h"
#include "cservo.h"
#include "scurrent.h"
#include "sdistforce.h"
#include "sedge.h"
#include "sencoder.h"
#include "simu.h"
#include "umqttin.h"

void loop()
{ // turn on last LED (14) as green to show that we are ready
  teensy[0].send("leds 14 0 45 0\n", true);
  int g = 5;
  int dg = 25;
  const int MSL = 50;
  char s[MSL];
  UTime t("now");
  while (not service.stopNowRequest)
  { // no action here, action can be handled over MQTT
    usleep(50000);
    if (t.getTimePassed() > 0.2)
    { // show activity in LED 14
      t.now();
      g += dg;
      if (g > 100 or g <= abs(dg))
        dg = -dg;
      snprintf(s, MSL, "leds 14 0 %d 0\n", g);
      //printf("# Main:: sending: %s", s);
      teensy[0].send(s, true);
      //
      // check for data flow dropout
      for (int i = 0; i < NUM_TEENSY_MAX; i++)
      {
        servo[i].tick();
        current[i].tick();
        distforce[i].tick();
        edge[i].tick();
        encoder[i].tick();
        imu[i].tick();
        robot[i].tick();
        teensy[i].tick();
        motor[i].tick();
        mqttin.tick();
      }
    }
  }
  // turn off LED 14
  teensy[0].send("leds 14 0 0 0\n", true);
}


int main (int argc, char **argv)
{ // is the process running already
  int a = service.isThisProcessRunning("teensy_interfac");
  if (a == 1)
  { // only me is running, so continue.
    // prepare all modules and start data flow
    // but also handle command-line options
    service.setup(argc, argv);
    //
    if (not service.theEnd)
    { // all set to go
      loop();
    }
    // close all logfiles etc.
    service.terminate();
    printf("# ---- Teensy_interface has ended (nicely) ----\r\n");
  }
  else
    printf("# ---- Teensy_interface is running already (stop with 'pkill teensy_interfac' ----\r\n");

  exit(0);
}

