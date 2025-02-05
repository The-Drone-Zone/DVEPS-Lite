import tkinter as tk
from Utils.Globals import Globals
from LogWindow import LoggingWindow
from Utils.connected import is_connected
from tkintermapview import TkinterMapView
import os

class Settings:
    def __init__(self, notebook, globals: Globals, logs: LoggingWindow):
        ## Default Settings ##
        self.flight_height = 1
        self.flight_speed = 5
        self.map_start_position = (32.2319, -110.9535) # U of A Mall
        # For offline map download
        self.top_left_map = (32.2339879, -110.9566328)
        self.bottom_right_map = (32.2308002, -110.9502170)

        # Initialize globals
        self.frame_wrapper = globals.frame_wrapper
        self.button_wrapper = globals.button_wrapper
        self.window_wrapper = globals.window_wrapper

        # Setup Logs
        self.logs = logs

        # Setup for map widgets
        self.map_widget1 = None
        self.marker = None
        self.map_widget2 = None
        self.markers = []

        # Create main frame
        self.settings_tab = self.frame_wrapper.create_frame(
            window=notebook, name="Settings Tab", bg="lightblue"
        )

        # Add Title
        title = tk.Label(
            self.settings_tab,
            text="Settings",
            justify=tk.CENTER,
            font=("Helvetica", 30, "bold")
        )
        title.pack(pady=10)

        self.flightHeight()
        self.flightSpeed()
        self.mapPosition()
        self.offlineMap()

    def flightHeight(self):
        # Create Frame for this section
        height_frame = self.frame_wrapper.create_frame(window=self.settings_tab, name="Flight Height")
        height_frame.pack(pady=10)
        # Create Label
        height_label = tk.Label(height_frame, text="Flight Height", justify=tk.CENTER, font=("Helvetica", 15))
        height_label.pack()
        # Create User Input
        height = tk.StringVar()
        height.set(str(self.flight_height))
        height_input = tk.Entry(height_frame, textvariable=height, font=("Helvetica", 15))
        height_input.pack()
        # Create Save Button
        save_btn = tk.Button(height_frame, 
            text="Save",
            font=("Helvetica", 12),
            command=lambda: self.updateFlightHeight(height)
        )
        save_btn.pack()

    def flightSpeed(self):
        # Create Frame for this section
        speed_frame = self.frame_wrapper.create_frame(window=self.settings_tab, name="Flight Speed")
        speed_frame.pack(pady=10)
        # Create Label
        speed_label = tk.Label(speed_frame, text="Flight Speed", justify=tk.CENTER, font=("Helvetica", 15))
        speed_label.pack()
        # Create User Input
        speed = tk.StringVar()
        speed.set(str(self.flight_speed))
        speed_input = tk.Entry(speed_frame, textvariable=speed, font=("Helvetica", 15))
        speed_input.pack()
        # Create Save Button
        save_btn = tk.Button(speed_frame, 
            text="Save",
            font=("Helvetica", 12),
            command=lambda: self.updateFlightSpeed(speed)
        )
        save_btn.pack()

    def mapPosition(self):
        # Create Frame for this section
        position_frame = self.frame_wrapper.create_frame(window=self.settings_tab, name="Map Position Setting")
        position_frame.pack(pady=10)
        # Create Label
        position_label = tk.Label(position_frame, text="Starting Map Position", justify=tk.CENTER, font=("Helvetica", 15))
        position_label.pack()

        # Initialize map widget
        if is_connected():
            print("Using Map with Internet for Map Position Settings")
            self.map_widget1 = TkinterMapView(position_frame, corner_radius=0)
        else:
            print("Using Offline Map for Map Position Settings")
            script_directory = os.path.dirname(
                os.path.abspath(__file__)
            )  # Current script directory
            parent_directory = os.path.dirname(script_directory)  # Move one folder back
            database_path = os.path.join(
                parent_directory, "offline_tiles.db"
            )  # DB in parent dir
            self.map_widget1 = TkinterMapView(
                position_frame,
                corner_radius=0,
                use_database_only=True,
                max_zoom=22,
                database_path=database_path,
            )

        # Place in frame
        self.map_widget1.pack()

        # Set Starting Location
        self.map_widget1.set_position(
            self.map_start_position[0], self.map_start_position[1]
        )

        # Add Create Marker Event
        self.map_widget1.add_right_click_menu_command(
            label="Add Marker", command=self.add_marker_event1, pass_coords=True
        )

        # Create Save Button
        save_btn = tk.Button(position_frame, 
            text="Save",
            font=("Helvetica", 12),
            command=self.updateMapPosition
        )
        save_btn.pack()

    def offlineMap(self):
        # Create Frame for this section
        offline_map_frame = self.frame_wrapper.create_frame(window=self.settings_tab, name="Offline Map Setting")
        offline_map_frame.pack(pady=10)
        # Create Label
        height_label = tk.Label(offline_map_frame, text="Offline Map Area", justify=tk.CENTER, font=("Helvetica", 15))
        height_label.pack()

        # Initialize map widget
        if is_connected():
            print("Using Map with Internet for Map Position Settings")
            self.map_widget2 = TkinterMapView(offline_map_frame, corner_radius=0)
        else:
            text = "You must be connected to the internet to use this feature"
            height_label = tk.Label(offline_map_frame, text=text, justify=tk.CENTER, font=("Helvetica", 15))
            height_label.pack()

        # Place in frame
        self.map_widget2.pack()

        # Set Starting Location
        self.map_widget2.set_position(
            self.map_start_position[0], self.map_start_position[1]
        )

        # Add Create Marker Event
        self.map_widget2.add_right_click_menu_command(
            label="Add Marker", command=self.add_marker_event2, pass_coords=True
        )

        # Create Save Button
        save_btn = tk.Button(offline_map_frame, 
            text="Save",
            font=("Helvetica", 12),
            command=self.updateOfflineMap
        )
        save_btn.pack()

    def updateFlightHeight(self, height: tk.StringVar):
        logText = ""
        try:
            self.flight_height = float(height.get())
            logText = f"User updated flight height to: {self.flight_height} m"
        except Exception:
            logText = f"Error: User attempted flight height to non decimal(float) value: {height.get()}"
            height.set(str(self.flight_height))

        print(logText)
        self.logs.addUserLog(logText)

    def updateFlightSpeed(self, speed: tk.StringVar):
        logText = ""
        try:
            self.flight_height = float(speed.get())
            logText = f"User updated flight speed to: {self.flight_height} m"
        except Exception:
            logText = f"Error: User attempted flight speed to non decimal(float) value: {speed.get()}"
            speed.set(str(self.flight_speed))

        print(logText)
        self.logs.addUserLog(logText)

    def updateMapPosition(self):
        logText = ""
        if self.marker != None:
            self.map_start_position = self.marker.position
            logText = f"User updated starting map position to: {self.map_start_position}"
        else:
            logText = f"Error: User attempted to set starting map position to None"

        print(logText)
        self.logs.addUserLog(logText)

    def updateOfflineMap(self):
        logText = ""
        if len(self.markers) == 2:
            self.top_left_map = self.markers[0].position
            self.bottom_right_map = self.markers[1].position
            logText = f"User began downloading new offline map with corners at {self.top_left_map} and {self.bottom_right_map}"
            ## ADD DOWNLOAD STUFF HERE ##
        else:
            logText = f"Error: User attempted to download new offline map using invalid coordinates"

        print(logText)
        self.logs.addUserLog(logText)

    def add_marker_event1(self, coords):
        print("Added marker in settings:", coords)
        if self.marker != None:
            self.marker.delete()
        self.marker = self.map_widget1.set_marker(
            coords[0], coords[1]
        )
        self.logs.addUserLog(
            "User added a marker on the map in the starting map position settings at: "
            + str(coords[0])
            + ", "
            + str(coords[1])
        )

    def add_marker_event2(self, coords):
        print("Added marker in settings:", coords)
        while len(self.markers) > 1:
            self.markers[0].delete()
            self.markers.pop(0)
        new_marker = self.map_widget2.set_marker(
            coords[0], coords[1]
        )
        self.markers.append(new_marker)
        self.logs.addUserLog(
            "User added a marker on the map in the offline map settings at: "
            + str(coords[0])
            + ", "
            + str(coords[1])
        )