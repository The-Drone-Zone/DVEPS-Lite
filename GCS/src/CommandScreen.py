import cv2
import os
import sys
import tkinter as tk
from tkinter import ttk
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
        self.lidar_notebook: ttk.Notebook = None
        self.lidar_plot1_frame: tk.Frame = None
        self.lidar_plot2_frame: tk.Frame = None
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
        # Variables for storing x,y positions. format = [x1, y1, x2, y2]
        self.horizontal = [None, None, None, None]
        self.diagonal1 = [None, None, None, None]
        self.diagonal2 = [None, None, None, None]
        # Create a Matplotlib figures
        self.lidar_fig, self.lidar_ax = plt.subplots(figsize=(0.1, 0.1), dpi=150)
        self.lidar_fig2, self.lidar_ax2 = plt.subplots(figsize=(0.1, 0.1), dpi=150)
        # Embed the Matplotlib plots into Tkinter canvas
        self.lidar_canvas = FigureCanvasTkAgg(self.lidar_fig, master=self.lidar_plot1_frame)
        self.lidar_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.lidar_canvas2 = FigureCanvasTkAgg(self.lidar_fig2, master=self.lidar_plot2_frame)
        self.lidar_canvas2.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # Stores if a plot is currently being drawn
        self.drawing_lidar_plot = False

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

        ##  Create lidar frames
        # Outermost frame
        self.lidar_frame = self.frame_wrapper.create_frame(
            window=self.command_tab, name="lidar_frame", bg="lightgreen"
        )
        self.frame_wrapper.add_to_window(
            self.lidar_frame, row=1, column=1, sticky="nsew"
        )
        self.lidar_frame.grid_propagate(False)
        # Create Notebook for both Lidar plots
        self.lidar_notebook = ttk.Notebook(self.lidar_frame)
        # Lidar plot 1
        self.lidar_plot1_frame = self.frame_wrapper.create_frame(
            window=self.lidar_notebook, name="lidar_plot1_frame", bg="lightgreen"
        )
        # Lidar plot 2
        self.lidar_plot2_frame = self.frame_wrapper.create_frame(
            window=self.lidar_notebook, name="lidar_plot2_frame", bg="lightgreen"
        )
        self.lidar_notebook.pack(expand=True, fill="both")
        self.lidar_notebook.add(self.lidar_plot1_frame, text="Forward Plot")
        self.lidar_notebook.add(self.lidar_plot2_frame, text="360 Plot")

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
            # For actual display
            data.increment /= 1000.0
            angles = []
            angles2 = []
            distances = []
            for i in range(len(data.distances)):
                angles.append((i * data.increment))
                angles2.append((i * data.increment) + data.angle_offset)
                distances.append(data.distances[i] / 100.0) # Convert distances from cm to m

            # print(data.angle_offset)
            # print(f"Angles: {angles2}")
            # print(f"Distances {distances}")

            ## Convert polar to cartesian coordinates (For actual display)
            # Horizontal line
            if data.angle_offset == -3:
                self.horizontal[0] = distances * np.sin(np.radians(angles))
                self.horizontal[1] = distances * np.cos(np.radians(angles))
                self.horizontal[2] = distances * np.sin(np.radians(angles2))
                self.horizontal[3] = distances * np.cos(np.radians(angles2))
            # Bottom left to top right diagonal (diagonal1)
            elif data.angle_offset == 119:
                self.diagonal1[0] = distances * np.sin(np.radians(angles))
                self.diagonal1[1] = distances * np.cos(np.radians(angles))
                self.diagonal1[2] = distances * np.sin(np.radians(angles2))
                self.diagonal1[3] = distances * np.cos(np.radians(angles2))
            # Top left to bottom right diagonal (diagonal2)
            elif data.angle_offset == 228:
                print(f"Angles: {angles2}")
                print(f"Distances {distances}")
                self.diagonal2[0] = distances * np.sin(np.radians(angles))
                self.diagonal2[1] = distances * np.cos(np.radians(angles))
                self.diagonal2[2] = distances * np.sin(np.radians(angles2))
                self.diagonal2[3] = distances * np.cos(np.radians(angles2))

            # Check if another thread is currently drawing and if this is the end of the scan (3rd message/diagonal2)
            if not self.drawing_lidar_plot and data.angle_offset == 228:
                self.drawing_lidar_plot = True
                ## Clear previous plot and plot new data
                # Plot 1
                self.lidar_ax.clear()
                self.lidar_ax.scatter([0], [0], color='orange', s=500) # Represents drone position
                self.lidar_ax.scatter(self.horizontal[0], self.horizontal[1], color='blue', s=25)  # Draw horizontal lidar points
                self.lidar_ax.scatter(self.diagonal1[0], self.diagonal1[1], color='red', s=25)  # Draw diagonal1 lidar points
                self.lidar_ax.scatter(self.diagonal2[0], self.diagonal2[1], color='green', s=25)  # Draw diagonal2 lidar points
                # Plot 2
                self.lidar_ax2.clear()
                self.lidar_ax2.scatter([0], [0], color='orange', s=500) # Represents drone position
                self.lidar_ax2.scatter(self.horizontal[2], self.horizontal[3], color='blue', s=25)  # Draw horizontal lidar points
                self.lidar_ax2.scatter(self.diagonal1[2], self.diagonal1[3], color='red', s=25)  # Draw diagonal1 lidar points
                self.lidar_ax2.scatter(self.diagonal2[2], self.diagonal2[3], color='green', s=25)  # Draw diagonal2 lidar points

                ## Set axis limits
                # Plot 1
                self.lidar_ax.set_xlim(-1, 1)
                self.lidar_ax.set_ylim(0, 30)
                # Plot 2
                self.lidar_ax2.set_xlim(-30, 30)
                self.lidar_ax2.set_ylim(-30, 30)

                ## Set axis labels
                # Plot 1
                self.lidar_ax.set_xlabel("Horizontal Distance (m)")
                self.lidar_ax.set_ylabel("Vertical Distance (m)")
                # Plot 2
                self.lidar_ax2.set_xlabel("Horizontal Distance (m)")
                self.lidar_ax2.set_ylabel("Vertical Distance (m)")

                # Set titles
                self.lidar_ax.set_title("LiDAR Scan Visualization")
                self.lidar_ax2.set_title("360 LiDAR Scan Visualization")

                ## Set axis scale (example: log scale for distance)
                # Plot 1
                self.lidar_ax.set_xscale('linear')
                self.lidar_ax.set_yscale('linear')
                # Plot 2
                self.lidar_ax2.set_xscale('linear')
                self.lidar_ax2.set_yscale('linear')
                
                # Redraw the canvas
                self.lidar_canvas.draw()
                self.lidar_canvas2.draw()

                # Open plot to be redrawn
                self.drawing_lidar_plot = False

        # Run the update task in a separate thread
        thread = threading.Thread(target=update, daemon=True)
        thread.start()

    def create_popup(self, message):
        self.window_wrapper.display_message(message)

    def tab_selected(self, event):  # TODO DELETE LATER
        self.lidar_notebook = event.widget
        tab_id = self.lidar_notebook.select()
        tab_text = self.lidar_notebook.tab(tab_id, "text")
