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
from threading import Thread
import numpy as np

class SPose:
    #
    motorVelocity = [0,0] # in radians/sec
    motorVelocityTime = datetime.now()
    motorVelocityCnt = 0
    motorVelocityInterval = 1000 # sec
    #
    wheelVelocity = [0,0] # in m/sec - if gearing and wheel radius is correct
    wheelVelocityTime = datetime.now()
    wheelVelocityCnt = 0
    wheelVelocityInterval = 1000 # sec
    #
    tripA = 0; # should not be reset
    tripAh = 0; # heading
    tripAtime = datetime.now()
    tripB = 0; # reset as needed
    tripBh = 0; # heading
    tripBtime = datetime.now()
    # robot info
    infoTime = datetime.now()
    infoCnt = 0
    tickPerRev = 68
    radiusLeft = 0.1
    radiusRight = 0.1
    gear = 19
    wheelBase = 0.1
    encoder_reversed = False
    need_data = True
    # pose (x (m),y (m),h (rad), tilt (rad - if available))
    pose = [0.0, 0.0, 0.0, 0.0]
    poseTime = datetime.now()
    poseCnt = 0
    poseInterval = 1000 # sec

    def velocity(self):
      # meters per second
      return (self.wheelVelocity[0] + self.wheelVelocity[1])/2

    def turnrate(self):
      # radians per second
      return (self.wheelVelocity[0] - self.wheelVelocity[1])/self.wheelBase

    def setup(self):
      from uservice import service
      loops = 0
      configured = False
      # get robot configuration (once)
      # data subscription is set in teensy_interface/build/robot.ini
      while not service.stop:
        # wait for data to arrive
        t.sleep(0.02)
        # wheel configuration info
        if self.infoCnt == 0 and False:
          # get configuration (once)
          service.send("robobot/cmd/T0","confi")
          pass
        elif not configured:
          # reset pose
          service.send("robobot/cmd/T0","enc0")
          ## send robot configuration
          # confw rl rr g t wb Set configuration 
          #     radius (left,right (m)), gear, encTick, wheelbase (m)
          service.send("robobot/cmd/T0","confw 0.074 0.074 19 92 0.23")
          # encoder reversed (motortest only)
          # service.send("robobot/cmd/T0/encrev","1")
          # request new configuration from Teensy
          service.send("robobot/cmd/T0","confi")
          # wait for new config message
          # self.infoCnt = 0
          configured = True
        elif self.wheelVelocityCnt == 0:
          # wait for wheel velocity (set in robot.ini)
          pass
        elif self.poseCnt == 0:
          # wait for pose data (set in robot.ini)
          pass
        else: # finished
          print(f"% Pose:: configured, and got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% Pose:: missing data updates after {loops} wait loops (continues).")
          print(f"% Pose:: got wheelVelocityCnt={self.wheelVelocityCnt}.")
          print(f"% Pose:: got motorVelocityCnt={self.motorVelocityCnt} (expected 0).")
          print(f"% Pose:: got wheelVelocityCnt={self.poseCnt}.")
          break
        pass
      pass

    def printMVel(self):
      from uservice import service
      print("% Pose motor velocity " + str(self.motorVelocityTime - service.startTime) + " (" +
            str(self.motorVelocity[0]) + ", " +
            str(self.motorVelocity[1]) + f") (rad/sec) {self.motorVelocityInterval:.4f} sec " +
            str(self.motorVelocityCnt))
    def printWVel(self):
      from uservice import service
      print("% Pose wheel velocity " + str(self.wheelVelocityTime - service.startTime) + " (" +
            str(self.wheelVelocity[0]) + ", " +
            str(self.wheelVelocity[1]) + f") (m/sec) {self.wheelVelocityInterval:.4f} sec " +
            str(self.wheelVelocityCnt))
    def printPose(self):
      from uservice import service
      print("% Pose  " + str(self.poseTime - service.startTime) + " (" +
            f"{self.pose[0]:.3f}, " +
            f"{self.pose[1]:.3f}, " +
            f"{self.pose[2]:.4f}, " +
            f"{self.pose[3]:.4f}) (m,m,rad,rad) {self.poseInterval:.4f} sec " +
            str(self.poseCnt))
    def printInfo(self):
      from uservice import service
      print(f"% SPose.py:: Robot config info {self.infoCnt} at " + str(self.motorVelocityTime - service.startTime))
      print(f"%    - Wheel radius (left,right): ({self.radiusLeft}, {self.radiusRight} m")
      print(f"%    - Encoder tick per rev: {self.tickPerRev}")
      print(f"%    - Gearing: {self.gear}:1")
      print(f"%    - Wheel base: {self.wheelBase} m")
      # reversed is for motortest only
      # print(f"%    - Encoder reversed: {self.encoder_reversed} (1 = reversed)")

    def tripAreset(self):
      self.tripA = 0
      self.tripAh = 0
      self.tripAtime = datetime.now()

    def tripBreset(self):
      self.tripB = 0
      self.tripBh = 0
      self.tripBtime = datetime.now()

    def tripAtimePassed(self):
      return (datetime.now() - self.tripAtime).total_seconds()

    def tripBtimePassed(self):
      return (datetime.now() - self.tripBtime).total_seconds()

    def decode(self, topic, msg):
        from ulog import flog
        # decode MQTT message
        used = True
        if topic == "T0/vel":
          gg = msg.split(" ")
          if (len(gg) > 3):
            t0 = self.wheelVelocityTime;
            self.wheelVelocityTime = datetime.fromtimestamp(float(gg[0]))
            # Teensy time (gg[1]) is ignored
            self.wheelVelocity[0] = float(gg[2])
            self.wheelVelocity[1] = float(gg[3])
            dt = (self.wheelVelocityTime - t0).total_seconds();
            if self.wheelVelocityCnt == 2:
              self.wheelVelocityInterval = dt;
            else:
              self.wheelVelocityInterval = (self.wheelVelocityInterval * 99 + dt) / 100
            self.wheelVelocityCnt += 1
            ds = (self.wheelVelocity[0] + self.wheelVelocity[1])*dt/2
            self.tripA += ds
            self.tripB += ds
            # self.printMVel()
        elif topic == "T0/mvel":
          gg = msg.split(" ")
          if (len(gg) > 2):
            t0 = self.motorVelocityTime
            self.motorVelocityTime = datetime.fromtimestamp(float(gg[0]))
            self.motorVelocity[0] = float(gg[1])
            self.motorVelocity[1] = float(gg[2])
            dt = (self.wheelVelocityTime - t0).total_seconds();
            if self.motorVelocityCnt == 2:
              self.motorVelocityInterval = dt
            else:
              self.motorVelocityInterval = (self.motorVelocityInterval * 99 + dt) / 100
            self.motorVelocityCnt += 1
            # self.printWVel()
        elif topic == "T0/pose":
          gg = msg.split(" ")
          if (len(gg) > 5):
            t0 = self.poseTime
            self.poseTime = datetime.fromtimestamp(float(gg[0]))
            # parameter 1 is teensy timestamp, ignored
            self.pose[0] = float(gg[2])
            self.pose[1] = float(gg[3])
            # heading
            h = float(gg[4])
            dh = h - self.pose[2]
            if (dh > np.pi):
              dh -= 2.0 * np.pi
            elif (dh < -np.pi):
              dh += 2.0 * np.pi
            self.tripBh += dh
            self.tripAh += dh
            self.pose[2] = h
            # tilt
            self.pose[3] = float(gg[5])
            # update statistics
            dt = (self.poseTime - t0).total_seconds();
            if self.poseCnt == 2:
              self.poseInterval = dt
            else:
              self.poseInterval = (self.poseInterval * 99 + dt) / 100
            self.poseCnt += 1
            # save pose to log
            if self.poseCnt % 10 == 0:
              flog.write()
            # self.printPose()
        elif topic == "T0/conf":
          # from Teensy: snprintf(s, MSL, "conf %.4f %.4f %.3f %u %.4f %.4f %d\r\n", odoWheelRadius[0], odoWheelRadius[1],
          #   gear, pulsPerRev, odoWheelBase, float(service.sampleTime_us)/1e6, motor.motorReversed
          gg = msg.split(" ")
          if (len(gg) > 7):
            self.infoTime = datetime.fromtimestamp(float(gg[0]))
            self.radiusLeft = float(gg[1])
            self.radiusRight = float(gg[2])
            self.gear = float(gg[3])
            self.tickPerRev = float(gg[4])
            self.wheelBase = float(gg[5])
            self.encoder_reversed = float(gg[7])
            self.infoCnt += 1
            # if self.infoCnt == 1:
            self.printInfo()
          else:
            print("% SPose:: got a too short configuration message '{msg}'")
        else:
          used = False
        return used

    def terminate(self):
        print("% Pose terminated")
        pass

# create the data object
pose = SPose()

