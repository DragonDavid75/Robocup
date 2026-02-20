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

class SIr:

    ir = [0, 0]
    irUpdCnt = 0
    irTime = datetime.now()
    irInterval = 0

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from uservice import service
      loops = 0
      while not service.stop:
        # wait for data to arrive
        t.sleep(0.01)
        if self.irUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% IR sensor (sir.py):: got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% IR sensor (sir.py):: missing data updates after {loops} wait loops (continues).")
          break
        pass
      pass

    def print(self):
      from uservice import service
      print("% IR dist " + str(self.accTime - service.startTime) + " (" +
            str(self.ir[0]) + ", " +
            str(self.ir[1]) + ", " +
            f") {self.irInterval:.4f} sec " +
            str(self.irUpdCnt))

    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/ir" or topic == "T0/ird":
          gg = msg.split(" ")
          if (len(gg) >= 3):
            t0 = self.irTime;
            self.irTime = datetime.fromtimestamp(float(gg[0]))
            self.ir[0] = float(gg[1])
            self.ir[1] = float(gg[2])
            t1 = self.irTime;
            if self.irUpdCnt == 2:
              self.irInterval = (t1 -t0).total_seconds()
            else:
              self.irInterval = (self.irInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.irUpdCnt += 1
            # self.print()
        else:
          used = False
        return used

    def terminate(self):
        print("% IR terminated")
        pass

# create the data object
ir = SIr()

