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

class SRobot:

    hbtTime =datetime.now()
    hbtInterval = 30.0
    hbtUpdCnt = 0
    hbtUpd = False
    batVolt = 1.0
    robotName = "unknown"

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from uservice import service
      loops = 0
      while not service.stop:
        # wait for essential data to arrive
        t.sleep(0.03)
        if self.hbtUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% Robot (srobot.py) data stream OK; {loops} loops.")
          break
        loops += 1
        if loops > 30:
          print(f"% Robot (srobot.py) got no HBT data after {loops} loops (stops).")
          print("% Is teensy_interface running?")
          service.stop = True
          break
        pass
      pass

    def hbtHasUpd(self):
      if hbtUpd:
        hbtUpd = False
        return True
      else:
        return False

    def print(self):
      from uservice import service
      print("% Robot hbt " + str(self.hbtTime - service.startTime) +
            f" Bat {self.batVolt:.1f} V," +
            f" {self.hbtInterval:.4f} sec, " +
            str(self.hbtUpdCnt))

    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/hbt":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.hbtTime;
            self.hbtTime = datetime.fromtimestamp(float(gg[0]))
            t1 = self.hbtTime;
            if self.hbtUpdCnt == 2:
              self.hbtInterval = (t1 -t0).total_seconds()
            else:
              self.hbtInterval = (self.hbtInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.hbtUpdCnt += 1
            self.hbtUpd = True
        elif topic == "T0/mot":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            # not decoded yet
            pass
        elif topic == "T0/dname":
          gg = msg.split(" ")
          if (len(gg) >= 2):
            self.robotName = gg[1]
            # print(f"% Got dname length {len(gg)} is: {msg}")
            pass
        elif topic == "T0/current":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            # not decoded yet
            pass
        elif topic == "T0/mca": # also current
          gg = msg.split(" ")
          if (len(gg) >= 4):
            # not decoded yet
            pass
        else:
          # print(f" srobotpy got topic '{topic}' but detect failed, used={used} - set to False")
          used = False
        return used

    def terminate(self):
        print("% Robot terminated")
        pass

# create the data object
robot = SRobot()

