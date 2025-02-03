import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


from src.Drone import Drone
from Utils.wrappers.ButtonWrapper import ButtonWrapper
from Utils.wrappers.FrameWrapper import FrameWrapper
from Utils.Utils import Utils
from Utils.wrappers.WindowWrapper import WindowWrapper


class Globals:
    def __init__(self):
        self.frame_wrapper = FrameWrapper()
        self.button_wrapper = ButtonWrapper()
        self.window_wrapper = WindowWrapper()

        self.drone = Drone(self.window_wrapper)
        self.Utils = Utils(self.window_wrapper)
