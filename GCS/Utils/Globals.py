import os
import sys


from Drone import Drone
from Utils.wrappers.ButtonWrapper import ButtonWrapper
from Utils.wrappers.FrameWrapper import FrameWrapper
from Utils.wrappers.WindowWrapper import WindowWrapper


class Globals:
    def __init__(self):
        self.frame_wrapper = FrameWrapper()
        self.Button_wrapper = ButtonWrapper()
        self.Widndow_wrapper = WindowWrapper()

        self.drone = Drone(self.Widndow_wrapper)
       