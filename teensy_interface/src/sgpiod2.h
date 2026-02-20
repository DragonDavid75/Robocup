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


#ifndef SGPIOD_H
#define SGPIOD_H

#include <gpiod.h>
#include "utime.h"

/**
 * Class to help access to GPIO pins on the Raspberry
 *
 * Requires that gpiod and libgpiod-dev are installed
 */
class SGpiod
{
public:
  /** setup and request data */
  void setup();
  /**
   * terminate */
  void terminate();
  /**
   * Read one of the available pins
   * \param pin is one of 13, 6, 12, 16, 19, 26, 21, 20
   * \returns -1 if pin not valid, else 0 or 1 from pin */
  int readPin(const int pin);
  /**
   * Set one of the available output pins pins
   * NB! must be defined as output in the robot.ini file
   * \param pin is one of 13, 6, 12, 16, 19, 26, 21, 20 */
  void setPin(const int pin, bool value);
  /**
   * Wait for pin to be high or low
   * \param pin - pin to wait for
   * \param timeout - value in ms, 0= wait forever
   * \param wait4Value 1 (default), 0 wait for pin to be low.
   * \return the pin value or -1 on timeout. */
  int wait4Pin(int pin, uint timeout_ms, int wait4Value = 1);
  /**
  * to listen to pins */
  void run();
  /**
   * Create a pointer to an output line */
  struct gpiod_line_request *
  request_output_line(unsigned int offset);
  /**
   * Create a pointer to an input line */
  struct gpiod_line_request *
  request_input_line(unsigned int offset);

protected:
  int getPinIndex(int pinNumber);

private:
  // base
  static const int MAX_PINS = 7;
  // const char *chippath = "/dev/gpiochip4";
  const char * line_user_name = "ricbot";
  struct gpiod_chip *chip = nullptr;
  // struct gpiod_line *pins[MAX_PINS] = {nullptr};
  // struct gpiod_request_config *req_cfg = NULL;
  struct gpiod_line_request *request[MAX_PINS] = {nullptr};
  // struct gpiod_line_settings *settings;
  // struct gpiod_line_config *line_cfg;  //
  // NB pin 13 (start) is not enabled here
  int pinOffset[MAX_PINS] = {6, 12, 16, 19, 26, 21, 20}; // pin number
  int in_pin_value[MAX_PINS] = {-1};
  bool out_pinuse[MAX_PINS] = {false};
  bool isOK = false;
  // logfile
  bool toConsole = false;
  FILE * logfile = nullptr;

private:
  static void runObj(SGpiod * obj)
  { // called, when thread is started
      // transfer to the class run() function.
      obj->run();
  }
  /**
   * Save pin values to log when there is a change
   * \param pv is an array of current pin values */
  void toLog(bool pv[]);
  //
  std::thread * th1;
};

/**
 * Make this visible to the rest of the software */
extern SGpiod gpio;

#endif
