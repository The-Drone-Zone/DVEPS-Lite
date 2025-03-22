import cv2
import os
import sys
import tkinter as tk
from Drone import Drone
from LogWindow import LoggingWindow
from Map import Map
from PIL import Image, ImageTk

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from Utils.Globals import Globals


class CommandScreen:
    def __init__(
        self,
        notebook,
        globals: Globals,
        drone: Drone,
        logs: LoggingWindow,
        settings,
    ):
        self.frame_wrapper = globals.frame_wrapper
        self.button_wrapper = globals.button_wrapper
        self.window_wrapper = globals.window_wrapper
        self.drone = drone
        self.logs = logs
        self.settings = settings

        self.command_tab = self.frame_wrapper.create_frame(
            window=notebook, name="Command Tab"
        )

        # only these frames can expand
        self.expand_btns = {
            "video_frame": tk.Button,
            "lidar_frame": tk.Button,
            "map_frame": tk.Button,
        }

        # major command frames
        self.video_frame: tk.Frame = None
        self.video_frame_width = None
        self.video_frame_height = None

        self.lidar_frame: tk.Frame = None
        self.map_frame: tk.Frame = None
        self.commands_frame: tk.Frame = None
        self.cap = cv2.VideoCapture(2)
        self.create_frames()

        self.video_frame_width = self.video_frame.winfo_width()
        self.video_frame_height = self.video_frame.winfo_height()
        self.create_video_display()
        self.update_video()
        ###############################################################################
        ###############################################################################
        # These variables are for the Command Frame

        # configure the commands frame box
        self.commands_frame.grid_rowconfigure(0, weight=1)
        self.commands_frame.grid_rowconfigure(1, weight=3)
        self.commands_frame.grid_rowconfigure(2, weight=3)
        self.commands_frame.grid_columnconfigure(0, weight=1)

        # minor command buttons
        self.land_btn: tk.Button = None
        self.hover_btn: tk.Button = None

        # minor command drowdowns
        self.command_dropdown = None
        self.option_menu = [
            "Begin Mission",
            "Pause Mission",
            "Takeoff",
            "Arm",
            "Disarm",
        ]
        self.chosen_val = tk.StringVar(self.commands_frame)
        self.chosen_val.set("Select Command")
        self.command_dropdown = tk.OptionMenu(
            self.commands_frame,
            self.chosen_val,
            *self.option_menu,
            command=self.drone.command_drone
        )
        self.command_dropdown.config(font=("Helvetica", 20), anchor="center")
        self.command_dropdown["menu"].config(font=("Helvetica", 20))
        self.frame_wrapper.add_to_window(
            self.command_dropdown, row=0, column=0, padx=10, pady=5, sticky="nsew"
        )

        ###############################################################################
        ###############################################################################
        # These variables are for the Map Frame

        # Create Map
        self.map = Map(self.map_frame, "Commands Map", globals, logs, settings, drone)
        # Set widget location and size
        self.map.map_widget.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.upload_plan_btn: tk.Button = None
        self.clear_markers_btn: tk.Button = None

        ###############################################################################
        ###############################################################################

        self.create_buttons()

        # Drone connection display
        self.drone_connection_label = tk.Label(
            self.command_tab,
            text="Drone Not Connected",
            justify=tk.CENTER,
            font=("Helvetica", 15),
        )
        self.drone_connection_label.place(relx=0.5, rely=0.1, anchor=tk.CENTER)
        self.update_drone_connected()

    def create_frames(self):

        # grid_row/column configure ensures row or column fills available space with equal weight
        self.command_tab.grid_rowconfigure(0, weight=3)
        self.command_tab.grid_rowconfigure(1, weight=2)
        self.command_tab.grid_columnconfigure(0, weight=1)
        self.command_tab.grid_columnconfigure(1, weight=1)

        # sticky param ensures that each frame fills its alloted cell, nsew = north, south, east, west
        # Create video frame
        self.video_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="video_frame", bg="lightblue"
        )
        self.frame_wrapper.add_to_window(
            self.video_frame, row=0, column=0, sticky="nsew"
        )

        # Create lidar frame
        self.lidar_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="lidar_frame", bg="lightgreen"
        )
        self.frame_wrapper.add_to_window(
            self.lidar_frame, row=0, column=1, sticky="nsew"
        )

        # Create Map frame
        self.map_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="map_frame", bg="yellow"
        )
        self.frame_wrapper.add_to_window(self.map_frame, row=1, column=0, sticky="nsew")

        # Create Commands frame
        self.commands_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="command_frame", bg="red"
        )
        self.frame_wrapper.add_to_window(
            self.commands_frame, row=1, column=1, sticky="nsew"
        )

    def create_buttons(self):

        # Create command buttons
        self.land_btn = self.button_wrapper.create_button(
            self.commands_frame, 20, text="Land", command=self.drone.land_command
        )
        self.button_wrapper.add_to_window(
            self.land_btn, row=1, column=0, padx=10, pady=5, sticky="nsew"
        )

        self.hover_btn = self.button_wrapper.create_button(
            self.commands_frame, 20, text="Hover", command=self.drone.hover_command
        )
        self.button_wrapper.add_to_window(
            self.hover_btn, row=2, column=0, padx=10, pady=5, sticky="nsew"
        )

        # create expand buttons
        self.create_expand_buttons()

        # instead of clicking on a map a user can just upload a file of coordinates.
        self.upload_plan_btn = self.button_wrapper.create_button(
            self.map_frame, 15, text="Upload Plan", command=self.map.upload_plan
        )
        self.button_wrapper.add_centered_button(self.upload_plan_btn, y=70)

        # Create clear marks button
        self.clear_markers_btn = self.button_wrapper.create_button(
            self.map_frame, 15, text="Clear Path", command=self.map.clear_marks
        )
        self.button_wrapper.add_centered_button(self.clear_markers_btn, y=110)

    def create_expand_buttons(self):
        # for loop add an expand button to each of the frames that we want to expand
        for frame_name, frame in zip(
            ["video_frame", "lidar_frame", "map_frame"],
            [self.video_frame, self.lidar_frame, self.map_frame],
        ):
            self.expand_btns[frame_name] = self.button_wrapper.create_button(
                frame, 15, text="Expand", command=lambda f=frame: self.expand_frame(f)
            )
            self.button_wrapper.add_centered_button(self.expand_btns[frame_name])

    def expand_frame(self, frame):

        for other_frame in [
            self.video_frame,
            self.lidar_frame,
            self.map_frame,
            self.commands_frame,
        ]:
            other_frame.grid_remove()  # this hides all and remembers grid positions, not destroyed just hidden

        # Use pack since we are only showing one frame
        frame.pack(fill="both", expand=True)

        # this make a new button to go back to the original view. this button is added to the frame that was expand
        minimize_btn = self.button_wrapper.create_button(
            frame,
            15,
            text="Minimize",
            command=lambda: self.minimize_frames(minimize_btn),
        )
        self.button_wrapper.add_centered_button(minimize_btn)

        self.logs.addUserLog("User expanded a view in the command tab")

    def minimize_frames(self, this_button):
        for frame in [
            self.video_frame,
            self.lidar_frame,
            self.map_frame,
            self.commands_frame,
        ]:
            frame.pack_forget()  # this hides the expanded frame

        # Shows grid frames in original positions
        self.video_frame.grid()
        self.lidar_frame.grid()
        self.map_frame.grid()
        self.commands_frame.grid()

        this_button.destroy()  # button destroys itself when pressed

        self.logs.addUserLog("User minimized a view in the command tab")

    def update_drone_connected(self):
        if self.drone.connected and self.drone.gps_ok:
            self.drone_connection_label.config(text="Drone Ready")
        elif self.drone.connected:
            self.drone_connection_label.config(text="Drone Connected")
        else:
            self.drone_connection_label.config(text="Drone Not Connected")

    def create_video_display(self):
        self.video_label = tk.Label(self.video_frame)
        self.video_label.grid(row=0, column=0, sticky="nsew")


    def update_video(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            frame_width = self.video_frame.winfo_width()
            frame_height = self.video_frame.winfo_height()
            # print(frame_width, frame_height)
            frame = cv2.resize(frame, (frame_width, frame_height))

            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)

            self.video_label.imgtk = imgtk
            self.video_label.config(image=imgtk)

        self.video_label.after(10, self.update_video) #run every 10ms