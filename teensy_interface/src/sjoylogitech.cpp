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

#include <sys/ioctl.h>
#include <signal.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "sjoylogitech.h"
#include "uservice.h"
// #include "cmixer.h"
// #include "cservo.h"
#include "mjoy.h"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

// create value
SJoyLogitech joyLogi;

void SJoyLogitech::setup()
{ // ensure default values
  if (not ini.has("Joy_Logitech"))
  { // no data yet, so generate some default values
    ini["Joy_Logitech"]["log"] = "true";
    ini["Joy_Logitech"]["print"] = "false";
    ini["Joy_Logitech"]["device"] = "/dev/input/js0";
  }
  // Linux device
  joyDevice = ini["Joy_Logitech"]["device"];
  //
  joyRunning = initJoy();
  if (joyRunning)
  { // logfile
    // if joystick available, then start in manual
//     mixer.setManualOverride(true);
    // start read thread
    toConsole = ini["Joy_Logitech"]["print"] == "true";
    if (ini["Joy_Logitech"]["log"] == "true" and logfile == nullptr)
    { // open logfile
      std::string fn = service.logPath + "log_joy_logitech.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% Logitech gamepad interface logfile\n");
      fprintf(logfile, "%% Device %s\n", joyDevice.c_str());
      fprintf(logfile, "%% Device type %s\n", deviceName.c_str());
      fprintf(logfile, "%% Button count %d\n", number_of_buttons);
      fprintf(logfile, "%% Axis count %d\n", number_of_axes);
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tmanual mode (else automatic)\n");
      fprintf(logfile, "%% 3-%d \tButtons pressed\n", number_of_buttons + 2);
      fprintf(logfile, "%% %d-%d \tAxis value\n", number_of_buttons + 3, number_of_axes + number_of_buttons + 2);
    }
    // start listen thread
    if (th1 == nullptr)
      th1 = new std::thread(runObj, this);
    printf("# UJoyLogitech:: joystick found (%s on %s) NB! mode must be in 'X' position\n", deviceName.c_str(), joyDevice.c_str());
  }
}

void SJoyLogitech::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
  }
}

void SJoyLogitech::run()
{
  UTime t;
  t.now();
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  while (not service.stop and joyRunning)
  { // handling gamepad events
    // Device is present
    bool gotEvent = getNewJsData();
    if (gotEvent)
    { // set time and update count
      updTime.now();
      updateCnt++;
      //
      if (t.getTimePassed() > 0.01 or gotButton or true)
      { // don't save too fast, but all button presses
        gotButton = false;
        t.now();
        toLog();
      }
    }
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (joyRunning)
  { // close device nicely
    joyRunning = false;
    if (jDev >= 0)
      close(jDev);
    jDev = -1;
  }
}

bool SJoyLogitech::initJoy()
{
  jDev = open (joyDevice.c_str(), O_RDWR | O_NONBLOCK);
  if (jDev >= 0)
  { // Joystic device found
    //Query and print the joystick name
    char name[128] = "no device found";
    if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
      strncpy(name, "Unknown", sizeof(name));
    //
    deviceName = name;
    ini["Joy_Logitech"]["device_type"] = deviceName;
    //Query and print number of axes and buttons
    ioctl (jDev, JSIOCGAXES, &number_of_axes);
    ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
//     printf("Registrered %d axes and %d buttons on joystick\n",number_of_axes, number_of_buttons);
  }
  return jDev >= 0;
}

bool SJoyLogitech::getNewJsData()
{
  struct js_event jse;
  bool lostConnection = false;
  bool isOK = false;
  // read full struct or nothing
  int bytes = read(jDev, &jse, sizeof(jse));
  // detect errors
  if (bytes == -1)
  { // error - an error occurred while reading
    switch (errno)
    { // may be an error, or just nothing send (buffer full)
      case EAGAIN:
        //not all send - just continue
        usleep(100);
        break;
      default:
        perror("UJoy::getNewJsData (other error device error): ");
        lostConnection = true;
        break;
    }
  }
  if (lostConnection)
  {
    joyRunning = false;
    if (jDev >= 0)
    {
      close(jDev);
      jDev = -1;
    }
    printf("UJoy::run: getNewJsData close\n");
  }
  if (joyRunning and bytes > 0)
  {
    if (bytes != sizeof(jse) and bytes > 0)
    { // size error
      printf("JOY control : Unexpected byte count from joystick:%d - continues\n", bytes);
    }
    else
    { //Proper joystick package has been received
      //Joystick package parser
      isOK = true;
      jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
      switch(jse.type) {
        // changed axis position
        case JS_EVENT_AXIS:
          if (jse.number < 16) {
            joyValues.axis[jse.number] = jse.value;
          }
          break;
        // changed button state
        case JS_EVENT_BUTTON:
          if (jse.number < 16) {
            joyValues.button[jse.number] = jse.value;
            gotButton = true;
          }
          break;
        default:
          printf("UJoy::getNewJsData : got bad data (event=%d, time=%d) - ignores\n", jse.type, jse.time);
          isOK = false;
          break;
      }
    }
  }
  return isOK;
}

void SJoyLogitech::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr and not service.stop_logging)
    { // save all axis and buttons
      fprintf(logfile, "%lu.%04ld %d ", updTime.getSec(), updTime.getMicrosec()/100,
           joy.manualMode());
      for (int i = 0; i < number_of_buttons; i++)
        fprintf(logfile, " %d", joyValues.button[i]);
      fprintf(logfile, " ");
      for (int i = 0; i < number_of_axes; i++)
        fprintf(logfile, " %d", joyValues.axis[i]);
      fprintf(logfile, "\n");
    }
    if (toConsole)
    { // print to console
      printf("# JoyLogitech:: %lu.%04ld %d ", updTime.getSec(), updTime.getMicrosec()/100,
              joy.manualMode());
      for (int i = 0; i < number_of_buttons; i++)
        printf(" %d", joyValues.button[i]);
      printf(" ");
      for (int i = 0; i < number_of_axes; i++)
        printf(" %5d", joyValues.axis[i]);
      printf("\n");
    }
  }
}

