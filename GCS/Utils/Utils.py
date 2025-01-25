import os
import sys


class Utils:
    def __init__(self, window_wrapper):
        self.window_wrapper = window_wrapper


    def ask_for_input(self, file_path):

        while(file_path and not os.path.isfile(file_path)):
            print(file_path)
            self.window_wrapper.create_window("bad_path", title="Takeoff", width=400, height=200)
            file_path = self.window_wrapper.display_custom_window("bad_path", message="The path you entered does not exist. Please try again.", input_prompt="Enter Path") 

        return file_path