import tkinter as tk
from Utils.Globals import Globals
from LogWindow import LoggingWindow
from Utils.connected import is_connected
from tkintermapview import TkinterMapView
import os
from Utils.loadOfflineTiles import loadOfflineTiles
import threading
import yaml
from Utils.Utils import get_root_dir
from Map import Map


class Settings:
    def __init__(self, notebook, globals: Globals, logs: LoggingWindow):
        # Get Settings YAML file Path
        self.settings_file = os.path.join(get_root_dir(), "settings.yml")

        # Load Data from YAML
        with open(self.settings_file, "r") as file:
            config = yaml.safe_load(file)

        ## Default Settings ##
        self.flight_height = config["flight_height"]
        self.flight_speed = config["flight_speed"]
        self.map_start_position = (
            config["map_start_position"]["latitude"],
            config["map_start_position"]["longitude"],
        )

        # Initialize globals
        self.globals = globals
        self.frame_wrapper = globals.frame_wrapper
        self.button_wrapper = globals.button_wrapper
        self.window_wrapper = globals.window_wrapper

        # Setup Logs
        self.logs = logs

        # Setup Maps
        self.position_map = None
        self.offline_map = None

        # Create main frame
        self.settings_tab = self.frame_wrapper.create_frame(
            window=notebook, name="Settings Tab", bg="lightblue"
        )

        # Add Title
        title = tk.Label(
            self.settings_tab,
            text="Settings",
            justify=tk.CENTER,
            font=("Helvetica", 30, "bold"),
        )
        title.pack(pady=10)

        self.createFlightHeightSection()
        self.createFlightSpeedSection()
        self.createMapPositionSection()
        self.createOfflineMapSection()

    def createFlightHeightSection(self):
        # Create Frame for this section
        height_frame = self.frame_wrapper.create_frame(
            window=self.settings_tab, name="Flight Height (m)", bg="lightblue"
        )
        height_frame.pack(pady=10)
        # Create Label
        height_label = tk.Label(
            height_frame,
            text="Flight Height",
            justify=tk.CENTER,
            font=("Helvetica", 15),
            bg="lightblue",
        )
        height_label.pack()
        # Create User Input
        height = tk.StringVar()
        height.set(str(self.flight_height))
        height_input = tk.Entry(
            height_frame, textvariable=height, font=("Helvetica", 15)
        )
        height_input.pack()
        # Create Save Button
        save_btn = self.button_wrapper.create_button(
            height_frame,
            12,
            text="Save",
            command=lambda: self.updateFlightHeight(height),
        )
        save_btn.pack(pady=5)

    def createFlightSpeedSection(self):
        # Create Frame for this section
        speed_frame = self.frame_wrapper.create_frame(
            window=self.settings_tab, name="Flight Speed (m/s)", bg="lightblue"
        )
        speed_frame.pack(pady=10)
        # Create Label
        speed_label = tk.Label(
            speed_frame,
            text="Flight Speed",
            justify=tk.CENTER,
            font=("Helvetica", 15),
            bg="lightblue",
        )
        speed_label.pack()
        # Create User Input
        speed = tk.StringVar()
        speed.set(str(self.flight_speed))
        speed_input = tk.Entry(speed_frame, textvariable=speed, font=("Helvetica", 15))
        speed_input.pack()
        # Create Save Button
        save_btn = self.button_wrapper.create_button(
            speed_frame, 12, text="Save", command=lambda: self.updateFlightSpeed(speed)
        )
        save_btn.pack(pady=5)

    def createMapPositionSection(self):
        # Create Frame for this section
        position_frame = self.frame_wrapper.create_frame(
            window=self.settings_tab, name="Map Position Setting", bg="lightblue"
        )
        position_frame.pack(pady=10)
        # Create Label
        position_label = tk.Label(
            position_frame,
            text="Starting Map Position",
            justify=tk.CENTER,
            font=("Helvetica", 15),
            bg="lightblue",
        )
        position_label.pack()

        # Initialize Map
        self.position_map = Map(
            position_frame,
            "Starting Position Settings Map",
            self.globals,
            self.logs,
            self,
        )

        # Place in frame
        self.position_map.map_widget.pack()

        # Create Save Button
        save_btn = self.button_wrapper.create_button(
            position_frame, 12, text="Save", command=self.updateMapPosition
        )
        save_btn.pack(pady=5)

    def createOfflineMapSection(self):
        # Create Frame for this section
        offline_map_frame = self.frame_wrapper.create_frame(
            window=self.settings_tab, name="Offline Map Setting", bg="lightblue"
        )
        offline_map_frame.pack(pady=10)
        # Create Label
        height_label = tk.Label(
            offline_map_frame,
            text="Download Offline Map",
            justify=tk.CENTER,
            font=("Helvetica", 15),
            bg="lightblue",
        )
        height_label.pack()

        if is_connected():
            # Initialize map widget
            self.offline_map = Map(
                offline_map_frame,
                "Download Offline Tiles Map",
                self.globals,
                self.logs,
                self,
            )
            self.offline_map.map_widget.pack()

            # Label to display download progress
            progress = tk.Label(offline_map_frame, text="", bg="lightblue")

            # Create Save Button
            save_btn = self.button_wrapper.create_button(
                offline_map_frame,
                12,
                text="Save",
                command=lambda: self.updateOfflineMap(progress),
            )

            save_btn.pack(pady=5)
            progress.pack()
        else:
            text = "You must be connected to the internet to use this feature"
            height_label = tk.Label(
                offline_map_frame, text=text, justify=tk.CENTER, font=("Helvetica", 15)
            )
            height_label.pack()

    def updateFlightHeight(self, height: tk.StringVar):
        logText = ""
        try:
            self.flight_height = float(height.get())
            self.save_settings()
            logText = f"User updated flight height to: {self.flight_height} m"
        except Exception:
            logText = f"Error: User attempted flight height to non decimal(float) value: {height.get()}"
            height.set(str(self.flight_height))

        self.logs.addUserLog(logText)

    def updateFlightSpeed(self, speed: tk.StringVar):
        logText = ""
        try:
            self.flight_speed = float(speed.get())
            self.save_settings()
            logText = f"User updated flight speed to: {self.flight_speed} m"
        except Exception:
            logText = f"Error: User attempted flight speed to non decimal(float) value: {speed.get()}"
            speed.set(str(self.flight_speed))

        self.logs.addUserLog(logText)

    def updateMapPosition(self):
        logText = ""
        if len(self.position_map.markers) > 0:
            self.map_start_position = self.position_map.markers[0].position
            self.save_settings()
            self.position_map.clear_marks()
            logText = (
                f"User updated starting map position to: {self.map_start_position}"
            )
        else:
            logText = f"Error: User attempted to set starting map position to None"

        self.logs.addUserLog(logText)

    def updateOfflineMap(self, progress: tk.Label):
        def download_task():
            logText = ""
            if len(self.offline_map.markers) == 2:
                top_left_map = self.offline_map.markers[0].position
                bottom_right_map = self.offline_map.markers[1].position
                logText = f"User began downloading new offline map with corners at {top_left_map} and {bottom_right_map}"
                self.logs.addUserLog(logText)

                # Update the GUI safely
                progress.after(0, lambda: progress.config(text="Downloading..."))

                # Simulate the downloading process
                loadOfflineTiles(top_left_map, bottom_right_map)

                # Update the GUI safely after the download is complete
                progress.after(0, lambda: progress.config(text="Download Complete"))

                logText = f"User completed download of offline map with corners at {top_left_map} and {bottom_right_map}"
            else:
                logText = f"Error: User attempted to download new offline map using invalid coordinates"
                # Update the GUI safely with an error
                progress.after(0, lambda: progress.config(text=logText))

            self.logs.addUserLog(logText)

        # Run the download task in a separate thread
        thread = threading.Thread(target=download_task, daemon=True)
        thread.start()

    def save_settings(self):
        data = {
            "flight_height": self.flight_height,
            "flight_speed": self.flight_speed,
            "map_start_position": {
                "latitude": self.map_start_position[0],
                "longitude": self.map_start_position[1],
            },
        }

        with open(self.settings_file, "w") as file:
            yaml.dump(data, file)
