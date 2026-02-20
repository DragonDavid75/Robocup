#/***************************************************************************
#*   Copyright (C) 2025 by DTU
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

import numpy as np
import cv2 as cv
from threading import Thread
import time as t
from datetime import *

class SCam:

  cap = {} # capture device
  th = {} # thread
  savedFrame = {}
  frameTime = datetime.now()
  getFrame = True
  cnt = 0
  gray = {}
  useCam = True
  imageFailCnt = 0

  def setup(self):
    if self.useCam:
      from uservice import service
      self.cap = cv.VideoCapture(f'http://{service.host}:7123/stream.mjpg')
      if self.cap.isOpened():
        self.th = Thread(target = cam.run)
        self.th.start()
      else:
        print("% SCam:: Camera failed to open")
    else:
      print("% SCam:: Camera disabled (in scam.py)")

  def getImage(self):
    fail = False
    if not self.useCam:
      if self.imageFailCnt == 0:
        print("% SCam:: not using cam")
      fail = True
    if not fail and not self.cap.isOpened():
      if self.imageFailCnt == 0:
        print("% SCam:: could not open")
      fail = True
    if not fail:
      from uservice import service
      self.getFrame = True
      cnt = 0 # timeout
      while self.getFrame and cnt < 100 and not service.stop:
        t.sleep(0.01)
        cnt += 1
      fail = self.getFrame
    if fail:
      self.imageFailCnt += 1
      return False, self.savedFrame, self.frameTime
    else:
      self.imageFailCnt = 0
      return True, self.savedFrame, self.frameTime

  def run(self):
    from uservice import service
    # print("% camera thread running")
    cnt = 0;
    first = True
    ret = False
    while self.cap.isOpened() and not service.stop:
      if self.getFrame or first:
        try:
          ret, self.savedFrame = self.cap.read()
        except:
          ret = False
        self.frameTime = datetime.now()
        if ret:
          self.getFrame = False
          self.cnt += 1
          if first:
            first = False
            h, w, ch = self.savedFrame.shape
            print(f"% Camera available: size ({h}x{w}, {ch} channels)")
      else:
        # just discard unused images
        self.cap.read()
      #
      # if frame is read correctly return is True
      if not ret:
          print("% Failed receive frame (stream end?). Exiting ...")
          self.terminate()
      # self.gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    print("% Camera thread stopped")


  def terminate(self):
    try:
      self.th.join()
    except:
      print("% join cam failed")
      pass
    if isinstance(self.cap, cv.VideoCapture):
      self.cap.release()
    else:
      print("% Camera stream was not open")
    cv.destroyAllWindows()
    print("% Camera terminated")

# create instance of this class
cam = SCam()
