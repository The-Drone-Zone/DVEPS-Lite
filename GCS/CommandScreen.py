import tkinter as tk
from tkinter import ttk

from Utils.Globals import Globals


class CommandScreen:
    def __init__(self, notebook, globals: Globals):
        self.frame_wrapper = globals.frame_wrapper
        self.BUttton_wrapper = globals.Button_wrapper
        self.drone = globals.drone

        self.command_tab = self.frame_wrapper.create_frame(window=notebook)

        #major command frames
        self.video_frame     : tk.Frame = None
        self.lidar_frame     : tk.Frame = None
        self.map_frame       : tk.Frame = None
        self.commands_frame  : tk.Frame = None
        self.create_frames()

        #configure the commands frame box
        self.commands_frame.grid_rowconfigure(0, weight=1)
        self.commands_frame.grid_rowconfigure(1, weight=3)
        self.commands_frame.grid_rowconfigure(2, weight=3)
        self.commands_frame.grid_columnconfigure(0, weight=1)

        #only these frames can expand
        self.expand_btns = {
            "video_frame" : tk.Button,
            "lidar_frame" : tk.Button,
            "map_frame"   : tk.Button
        }

        #minor command buttons
        self.land_btn  : tk.Button = None
        self.hover_btn : tk.Button = None
        self.create_buttons()

        #minor command drowdowns
        self.command_dropdown = None
        self.option_menu = ["Takeoff", "Set Speed", "Set Altitude", "Set Heading"]
        self.chosen_val = tk.StringVar(self.commands_frame)
        self.chosen_val.set("Select Command")
        self.command_dropdown = tk.OptionMenu(self.commands_frame, self.chosen_val, *self.option_menu, command=self.drone.command_drone)
        self.command_dropdown.config(font=("Helvetica", 32), anchor="center")
        self.command_dropdown["menu"].config(font=("Helvetica", 32))
        self.frame_wrapper.add_to_window(self.command_dropdown, row=0, column=0, padx=10, pady=5, sticky="nsew")



    def create_frames(self):

        #grid_row/column configure ensures row or column fills avaiable space with equal weight
        self.command_tab.grid_rowconfigure(0, weight=3)
        self.command_tab.grid_rowconfigure(1, weight=2)
        self.command_tab.grid_columnconfigure(0, weight=1)
        self.command_tab.grid_columnconfigure(1, weight=1)

        #sticky param ensures that each frame fills its alloted cell, nsew = north, south, east, west
        #Create video frame
        self.video_frame = self.frame_wrapper.create_frame(window=self.command_tab, bg="lightblue")
        self.frame_wrapper.add_to_window(self.video_frame, row=0, column=0, sticky="nsew")
        
        #Create lidar frame
        self.lidar_frame = self.frame_wrapper.create_frame(window=self.command_tab, bg="lightgreen")
        self.frame_wrapper.add_to_window(self.lidar_frame, row=0, column=1, sticky="nsew")

        #Create Map frame
        self.map_frame = self.frame_wrapper.create_frame(window=self.command_tab, bg="yellow")
        self.frame_wrapper.add_to_window(self.map_frame, row=1, column=0, sticky="nsew")

        #Create Commands frame
        self.commands_frame = self.frame_wrapper.create_frame(window=self.command_tab, bg="red")
        self.frame_wrapper.add_to_window(self.commands_frame, row=1, column=1, sticky="nsew")

        
    def create_buttons(self):
        #create expand buttons

        #Create command buttons
        self.land_btn = self.BUttton_wrapper.create_button(self.commands_frame, 32,text="Land", command=self.drone.land_drone)
        self.BUttton_wrapper.add_to_window(self.land_btn, row=1, column=0, padx=10, pady=5, sticky="nsew")

        self.hover_btn = self.BUttton_wrapper.create_button(self.commands_frame, 32, text="Hover", command=self.drone.hover_drone)
        self.BUttton_wrapper.add_to_window(self.hover_btn, row=2, column=0, padx=10, pady=5, sticky="nsew")

        self.create_expand_buttons()


    def create_expand_buttons(self):
        #for loop add an expand button to each of the frames that we want to expand
        for frame_name, frame in zip(["video_frame", "lidar_frame", "map_frame"], [self.video_frame, self.lidar_frame, self.map_frame]):
            self.expand_btns[frame_name] = self.BUttton_wrapper.create_button(frame, 12, text="Expand", command=lambda f=frame: self.expand_frame(f))
            self.BUttton_wrapper.add_to_window(self.expand_btns[frame_name], row=0, column=0, padx=5, pady=5)


    def expand_frame(self, frame):
        for other_frame in [self.video_frame, self.lidar_frame, self.map_frame, self.commands_frame]:
            other_frame.grid_forget() #this hides all, frames widgets/frames do not get destroyed just hidden

        #this expand the selected frame to the entire window
        self.frame_wrapper.add_to_window(frame, row=0, column=0, rowspan=2, columnspan=2, sticky="nsew")

        #this make a new buton to go back to the original veiw. this button is added to the frame that was expand
        reset_btn = self.BUttton_wrapper.create_button(frame, 12, text="Reset", command=lambda: self.reset_frames(reset_btn))
        self.BUttton_wrapper.add_to_window(reset_btn, row=1, column=0, padx=5, pady=5)


    def reset_frames(self, this_button):
        for frame in [self.video_frame, self.lidar_frame, self.map_frame, self.commands_frame]:
            frame.grid_forget() #this hides all, frames they are not destroyed just hidden.
        
        #this readds all the frames to the window in their original positions
        self.frame_wrapper.add_to_window(self.video_frame, row=0, column=0, sticky="nsew")
        self.frame_wrapper.add_to_window(self.lidar_frame, row=0, column=1, sticky="nsew")
        self.frame_wrapper.add_to_window(self.map_frame, row=1, column=0, sticky="nsew")
        self.frame_wrapper.add_to_window(self.commands_frame, row=1, column=1, sticky="nsew")

        this_button.destroy() #button destorys itself when pressed