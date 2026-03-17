#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
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


import time as t
from datetime import *


class SGpio:

    button = [False] * 8
    butTime =datetime.now()
    onPi = False
    gpio06 = False
    gpio12 = False
    # gpio13 = False
    gpio16 = False
    gpio19 = False
    gpio20 = False
    gpio21 = False
    gpio26 = False


    def setup(self):
      try:
        import gpiod
        self.chip = gpiod.Chip('gpiochip4')
        self.onPi = True
      except:
        print("% No GPIO (not on a Pi?) - continue without GPIO support")
        pass
      if self.onPi:
        self.gpio06 = self.chip.get_line(6)
        self.gpio12 = self.chip.get_line(12)
        # self.gpio13 = self.chip.get_line(13)
        self.gpio16 = self.chip.get_line(16)
        self.gpio19 = self.chip.get_line(19)
        self.gpio20 = self.chip.get_line(20)
        self.gpio21 = self.chip.get_line(21)
        self.gpio26 = self.chip.get_line(26)
        # 'consumer' is visible with command $ gpioinfo
        allOK = False
        allOKcnt = 0;
        while not allOK or allOKcnt > 3:
          try: # reserve all used pins
            # self.gpio13.request(consumer="robobot", # start button
            #                     type  = gpiod.LINE_REQ_DIR_IN,
            #                     flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN)
            self.gpio06.request(consumer="robobot", # stop button
                                type  = gpiod.LINE_REQ_DIR_IN,
                                flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN)
            self.gpio12.request(consumer="robobot",
                                type  = gpiod.LINE_REQ_DIR_IN,
                                flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN)
            self.gpio16.request(consumer="robobot",
                                type  = gpiod.LINE_REQ_DIR_IN,
                                flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN)
            self.gpio19.request(consumer="robobot",
                                type  = gpiod.LINE_REQ_DIR_IN,
                                flags = gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN)
            self.gpio20.request(consumer="robobot",
                                type=gpiod.LINE_REQ_DIR_OUT)
            self.gpio21.request(consumer="robobot",
                                type=gpiod.LINE_REQ_DIR_OUT)
            self.gpio26.request(consumer="robobot",
                                type=gpiod.LINE_REQ_DIR_OUT)
            allOK = True
            t.sleep(0.05)
            allOKcnt += 1
          except:
            print("% GPIO request for some pins failed");
        if not allOK:
          # reserve buttons failed
          print("% GPIO pin reservation failed - another app has reserved the pin?")
          self.gpio06.release()
          self.gpio12.release()
          # self.gpio13.release()
          self.gpio16.release()
          self.gpio19.release()
          self.gpio20.release()
          self.gpio21.release()
          self.gpio26.release()
          onPi = False
        # else:
        #   print("% GPIO all ok")
        #   while self.gpio13.get_value() == 1:
        #     t.sleep(0.05)
        #     print(f"% wait for key release {allOkcnt}")
        #     allOKcnt += 1
      pass

    # def start(self):
    #   if self.onPi:
    #     # print(f"Button 13 returns {self.gpio13.get_value()}")
    #     v = self.gpio13.get_value()
    #     if (v == 1):
    #       print(f"% Button/pin 13 (start) is pressed")
    #     return v == 1
    #   else:
    #     return False

    def test_stop_button(self):
      if self.onPi:
        # print(f"Button 06 returns {self.gpio06.get_value()}")
        v = self.gpio06.get_value()
        if (v == 1):
          print("% Button/pin 6 (stop) is pressed")
        return v == 1
      else:
        return False

    def set_value(self, line, value):
      if self.onPi:
        a_line = self.chip.get_line(line)
        if a_line.direction() == a_line.DIRECTION_OUTPUT:
          a_line.set_value(value)
        else:
          print(f"% GPIO pin {line} is input: {a_line.direction()} != {a_line.DIRECTION_OUTPUT}")
      pass

    def get_value(self, line):
      if self.onPi:
        if line == 12:
          v = self.gpio12.get_value()
        elif line == 16:
          v = self.gpio16.get_value()
        elif line == 19:
          v = self.gpio19.get_value()
        else:
          print(f"% GPIO pin {line} is not an input line")
          v = -1
        if (v == 1):
          print(f"% Button/pin {line} is pressed/high")
        return v == 1
      else:
        return False

    def print(self):
      from uservice import service
      print("% GPIO button " + str(self.accTime - service.startTime))

    def decode(self, topic, msg):
      # decode MQTT message
      # none yet (may come, if teensy_interface listens to GPIO too).
      used = False
      return used


    def terminate(self):
      if self.onPi:
        pass
      print("% GPIO terminated")
      pass

# create the data object
gpio = SGpio()

