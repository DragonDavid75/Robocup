# core/vision_system.py
import threading
import time
import cv2 as cv
from scam import cam
from sedge import edge

class VisionSystem(threading.Thread):

    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True

    def run(self):
        while self.running:

            if cam.useCam:
                ok, img, imgTime = cam.getImage()

                if ok:
                    edge.paint(img)

                    with self.world.lock:
                        self.world.image = img
                        self.world.line_detected = edge.lineValidCnt > 2
                        self.world.line_left = edge.posLeft
                        self.world.line_right = edge.posRight

            time.sleep(0.03)  # ~30 Hz

    def stop(self):
        self.running = False
