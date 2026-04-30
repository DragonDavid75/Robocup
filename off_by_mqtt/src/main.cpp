/*
 #***************************************************************************
 #*   Copyright (C) 2023 by DTU
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
#include <unistd.h>

//
// include local files for data values and functions
#include "uservice.h"
#include "umqtt.h"


int main (int argc, char **argv)
{ // prepare all modules and start data flow
  int a = service.isThisProcessRunning("off_by_mqtt");
  UTime t("now");
  if (a == 1)
  { // only me is running, so continue
    bool setupOK = service.setup(argc, argv);
    std::cout << "# Listening to MQTT)\n";
    while (setupOK and not service.stop)
    {
      usleep(50000);
      if (t.getTimePassed() > 3)
      {
        t.now();
        const int MSL = 150;
        char s[MSL];
        char s2[50];
        snprintf(s, MSL, "alive at %s", t.getDateTimeAsString(s2));
        mqtt.publish("robobot/off_by_mqtt/alive", s, t, 0);
      }
      // nothing to do but wait
    }
    printf("Main end in 1 sec\n");
    sleep(1);
    printf("Main end in 0 sec\n");
  }
  else
  {
    printf("# off_by_mqtt is already running - kill with 'pkill off_by_mqtt'\n");
  }
  return 0;
}

