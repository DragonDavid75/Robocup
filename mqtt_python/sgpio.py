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

gpioFound = False
try:
  import RPi.GPIO as GPIO
  GPIO.setmode(GPIO.BCM)
  list = [6, 12, 16, 19, 26, 21, 20]
  GPIO.setup(list, GPIO.IN)
  gpioFound = True
  print("% BCM GPIO found OK")
except:
  print("% No GPIO (not on a Pi?) - continue without GPIO support")
  pass


class SGpio:
    onPi = False

    def setup(self):
      if gpioFound:
        # import RPi.GPIO as GPIO
        print("% GPIO setup start")
        GPIO.setwarnings(False)
        # list = [13, 12, 16, 19, 26, 21, 20]
        GPIO.setup(6, GPIO.IN)
        print("% GPIO setup finished")
        self.onPi = True

    def test_stop_button(self):
      if self.onPi:
        # print(f"Button 06 returns {self.GPIO06.get_value()}")
        v = self.get_value(6)
        if (v == 1):
          print("% Button/pin 6 (stop) is pressed")
        return v == 1
      else:
        return False

    def set_value(self, line, value):
      if self.onPi:
        # import RPi.GPIO as GPIO
        if True: # try:
          isIn = GPIO.gpio_function(line)
          if isIn:
            GPIO.setup(line, GPIO.OUT)
          GPIO.output(line, value)
        # except:
        #   print(f"% GPIO set pin {line} can't be set (got error trying)")
      pass

    def get_value(self, line):
      if self.onPi:
        # import RPi.GPIO as GPIO
        if True: # try:
          isIn = GPIO.gpio_function(line)
          if not isIn:
            GPIO.setup(line, GPIO.IN)
          v = GPIO.input(line)
          if (v == 1):
            print(f"% Button/pin {line} is pressed (high)")
          return v == 1
        # except:
        #   print(f"% GPIO get pin {line} can't be set (got error trying)")
      else:
        return False

    # def print(self):
    #   from uservice import service
    #   print("% GPIO button " + str(self.accTime - service.startTime))

    def decode(self, topic, msg):
      # decode MQTT message
      # none yet (may come, if teensy_interface listens to GPIO too).
      used = False
      return used


    def terminate(self):
      if self.onPi:
        # import RPi.GPIO as GPIO
        GPIO.cleanup()
        pass
      print("% GPIO terminated")
      pass

# create the data object
gpio = SGpio()

