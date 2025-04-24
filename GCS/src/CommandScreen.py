import cv2
import os
import sys
import tkinter as tk
from Drone import Drone
from LogWindow import LoggingWindow
from Map import Map
from PIL import Image, ImageTk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading

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

        # Header variables
        self.header_frame: tk.Frame = None
        self.drone_connection_container: tk.Frame = None
        self.drone_connection_label: tk.Label = None
        self.battery_container: tk.Frame = None
        self.battery_label: tk.Label = None
        self.create_header()

        # major command frames
        self.video_frame: tk.Frame = None
        self.video_frame_width = None
        self.video_frame_height = None

        self.lidar_frame: tk.Frame = None
        self.map_frame: tk.Frame = None
        self.commands_frame: tk.Frame = None
        self.cap = cv2.VideoCapture(1)
        self.create_frames()

        ### Video Frame Setup
        self.video_frame_width = self.video_frame.winfo_width()
        self.video_frame_height = self.video_frame.winfo_height()
        self.create_video_display()
        self.update_video()

        ### LiDAR Frame Setup
        # Create a Matplotlib figure
        self.lidar_fig, self.lidar_ax = plt.subplots(figsize=(0.1, 0.1), dpi=150)

        # Embed the Matplotlib plot into Tkinter canvas
        self.lidar_canvas = FigureCanvasTkAgg(self.lidar_fig, master=self.lidar_frame)
        self.lidar_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.update_lidar_plot() # For Simulating ONLY

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

        self.update_drone_connected()
        self.update_drone_battery()

    def create_header(self):
        # Header frame
        self.header_frame = tk.Frame(self.command_tab, bg="white")
        self.header_frame.grid(row=0, column=0, columnspan=2, sticky="ew")

        # Configure grid layout for the header_frame
        self.header_frame.columnconfigure(0, weight=1)
        self.header_frame.columnconfigure(1, weight=1)

        # ====== LEFT TEXT ======
        # Container frame to simulate padding/border
        self.drone_connection_container = tk.Frame(self.header_frame, bg="tomato")
        self.drone_connection_container.grid(row=0, column=0, sticky="w", padx=(0, 5))

        # Drone connection display inside red container
        self.drone_connection_label = tk.Label(
            self.drone_connection_container,
            text="Drone Not Connected",
            bg="tomato",
            anchor="w",
            font=("Helvetica", 15),
        )
        self.drone_connection_label.pack(fill="both", padx=40, pady=2)

        # ====== RIGHT TEXT ======
        # Container frame to simulate padding/border
        self.battery_container = tk.Frame(self.header_frame, bg="tomato")
        self.battery_container.grid(row=0, column=1, sticky="e", padx=(5, 0))

        # Battery % Display
        self.battery_label = tk.Label(
            self.battery_container,
            text="Battery Remaining: N/A",
            bg="tomato",
            anchor="e",
            font=("Helvetica", 15),
        )
        self.battery_label.pack(fill="both", padx=40, pady=2)

    def create_frames(self):

        # grid_row/column configure ensures row or column fills available space with equal weight
        self.command_tab.grid_rowconfigure(1, weight=3)
        self.command_tab.grid_rowconfigure(2, weight=2)
        self.command_tab.grid_columnconfigure(0, weight=1)
        self.command_tab.grid_columnconfigure(1, weight=1)

        # sticky param ensures that each frame fills its alloted cell, nsew = north, south, east, west
        # Create video frame
        self.video_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="video_frame", bg="lightblue"
        )
        self.frame_wrapper.add_to_window(
            self.video_frame, row=1, column=0, sticky="nsew"
        )

        # Create lidar frame
        self.lidar_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="lidar_frame", bg="lightgreen"
        )
        self.frame_wrapper.add_to_window(
            self.lidar_frame, row=1, column=1, sticky="nsew"
        )
        self.lidar_frame.grid_propagate(False)

        # Create Map frame
        self.map_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="map_frame", bg="yellow"
        )
        self.frame_wrapper.add_to_window(self.map_frame, row=2, column=0, sticky="nsew")

        # Create Commands frame
        self.commands_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="command_frame", bg="red"
        )
        self.frame_wrapper.add_to_window(
            self.commands_frame, row=2, column=1, sticky="nsew"
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
        if self.drone.connected and self.drone.gps_ok and self.drone.armable:
            self.drone_connection_label.config(text="Drone Ready", bg="lightgreen")
            self.drone_connection_container.config(bg="lightgreen")
        elif self.drone.connected:
            self.drone_connection_label.config(text="Drone Connected", bg="yellow")
            self.drone_connection_container.config(bg="yellow")
        else:
            self.drone_connection_label.config(text="Drone Not Connected", bg="tomato")
            self.drone_connection_container.config(bg="tomato")

    def update_drone_battery(self):
        if self.drone.battery_percent > 65:
            self.battery_label.config(text=f"Battery Remaining: {self.drone.battery_percent}%", bg="lightgreen")
            self.battery_container.config(bg="lightgreen")
        elif self.drone.battery_percent > 30:
            self.battery_label.config(text=f"Battery Remaining: {self.drone.battery_percent}%", bg="yellow")
            self.battery_container.config(bg="yellow")
        else:
            self.battery_label.config(text=f"Battery Remaining: {self.drone.battery_percent}%", bg="tomato")
            self.battery_container.config(bg="tomato")

    def create_video_display(self):
        self.video_label = tk.Label(self.video_frame)
        self.video_label.grid(row=1, column=0, sticky="nsew")


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

    # data is of type OBSTACLE_DISTANCE from px4_msgs
    def update_lidar_plot(self, data=None):
        def update():
            # Simulate incoming LiDAR data (Replace this with real data)
            angles = np.linspace(-30, 30, 72)  # 72 points for -60 to 60 degrees
            distances = np.random.uniform(20, 30, size=72)  # Random distances for simulation

            ## For actual display
            # angles = []
            # for i in range(len(data.distances)):
            #     # distances.append(data.distances[i])
            #     angles.append(i * data.increment)
            #     data.distances[i] /= 100.0 # Convert distances from cm to m

            # print(f"Angles: {angles}")
            # print(f"Distances {data.distances}")
            
            # Convert polar to cartesian coordinates
            y = distances * np.cos(np.radians(angles)) # For simulated display
            x = distances * np.sin(np.radians(angles)) # For simulated display
            # y = data.distances * np.cos(np.radians(angles)) # For actual display
            # x = data.distances * np.sin(np.radians(angles)) # For actual display
            # print(f"X: {x}")
            # print(f"Y: {y}")
            
            # Clear previous plot and plot new data
            self.lidar_ax.clear()
            self.lidar_ax.scatter([0], [0.5], color='red', s=500) # Represents drone position
            self.lidar_ax.scatter(x, y, color='blue', s=25)  # Draw lidar points

            # Set axis limits
            self.lidar_ax.set_xlim(-20, 20)
            self.lidar_ax.set_ylim(0, 40)

            # Set axis labels
            self.lidar_ax.set_xlabel("Horizontal Distance (m)")
            self.lidar_ax.set_ylabel("Vertical Distance (m)")

            # Set title
            self.lidar_ax.set_title("LiDAR Scan Visualization")

            # Set aspect ratio (optional, if you want the axes to scale equally)
            self.lidar_ax.set_aspect('equal', 'box')

            # Set axis scale (example: log scale for distance)
            self.lidar_ax.set_xscale('linear')
            self.lidar_ax.set_yscale('linear')
            
            # Redraw the canvas
            self.lidar_canvas.draw()

            # Schedule the next update (e.g., 100 ms)
            self.lidar_frame.after(10, self.update_lidar_plot)

        # Run the update task in a separate thread
        thread = threading.Thread(target=update, daemon=True)
        thread.start()

    def create_popup(self, message):
        self.window_wrapper.display_message(message)