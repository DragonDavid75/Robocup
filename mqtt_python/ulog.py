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

class ULog:
  st = 0
  f = open('logfile.txt', 'w', encoding="utf-8")

  def setup(self):
    from sedge import edge
    self.f.write("% logfile for Python side\n")
    self.f.write("% 1 time (sec)\n")
    self.f.write("% 2 Mission state\n")
    self.f.write("% 3,4,5 (x,y,h) (m,m,rad)\n")
    self.f.write("% 6 Camera frame count\n")
    self.f.write("% 7-9 Line sensor: line position, e*kp, control y (rad/s), validCnt\n")
    self.f.write(f"%     Line sensor: Kp={edge.lineKp}, tau_z={edge.lineTauZ}s, tau_p={edge.lineTauP}s\n")
    self.f.write("% 10,11 trip A (distance and heading change)\n")
    self.f.write("% 12,13 trip B (distance and heading change)\n")
    pass

  def writeRemark(self, remark = "remark"):
    from uservice import service
    if not service.stop:
      lt = t.time()
      # timestamp and remark preceded by a MATLAB comment character
      self.f.write(f"% {lt} {remark}\n")

  def writeDataString(self, data = "remark"):
    from uservice import service
    if not service.stop:
      lt = t.time()
      # timestamp and remark preceded by a MATLAB comment character
      self.f.write(f"{lt} {data}\n")

  def write(self, state = -1):
    from spose import pose
    from sir import ir
    from srobot import robot
    from scam import cam
    from sedge import edge
    from sgpio import gpio
    from scam import cam
    from uservice import service
    if not service.stop:
      lt = t.time()
      if state >= 0:
        #
        self.st = state
      # timestamp
      self.f.write(f"{lt} {self.st} ")
      # pose (x, y, h)
      self.f.write(f"{pose.pose[0]:.3f} {pose.pose[1]:.3f} {pose.pose[2]:.3f} ")
      # camera image number
      self.f.write(f"{cam.cnt} ")
      # line sensor detected position
      self.f.write(f"{edge.posLeft:.2f} {edge.posRight:.2f} {edge.u:.3f} {edge.lineY:.3f} {edge.lineValidCnt} ")
      # trip A distance and heading change
      self.f.write(f"{pose.tripA:.3f} {pose.tripAh:.3f} ")
      # trip B distance and heading change
      self.f.write(f"{pose.tripB:.4f} {pose.tripBh:.4f}\n")

  def terminate(self):
    self.f.close()
    print("% logfile closed")
    pass

# create the data object
flog = ULog()

