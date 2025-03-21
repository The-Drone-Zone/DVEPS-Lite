import tkinter as tk
from tkintermapview import TkinterMapView
import os
import csv
from Utils.connected import is_connected
from LogWindow import LoggingWindow
from Utils.Utils import get_root_dir


class Map:
    def __init__(
        self, map_frame, map_name, globals, logs: LoggingWindow, settings, drone=None
    ):
        # Initialize global variables
        self.globals = globals
        self.window_wrapper = globals.window_wrapper
        self.logs = logs
        self.settings = settings
        self.drone = drone

        self.map_name = map_name

        # Initialize map widget
        if is_connected():
            self.logs.addUserLog(f"{self.map_name}: Using Map with Internet")
            self.map_widget = TkinterMapView(map_frame, corner_radius=0)
        else:
            self.logs.addUserLog(f"{self.map_name}: Using Offline Map")
            # Get DB directory
            database_path = os.path.join(get_root_dir(), "offline_tiles.db")
            self.map_widget = TkinterMapView(
                map_frame,
                corner_radius=0,
                use_database_only=True,
                max_zoom=22,
                database_path=database_path,
            )
            # self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga")

        # Set Starting Location
        self.map_widget.set_position(
            self.settings.map_start_position[0], self.settings.map_start_position[1]
        )

        # Track markers
        self.drone_marker = None
        self.markers = []
        self.marker_positions = []
        self.path = None

        # Add Create Marker Event
        self.map_widget.add_right_click_menu_command(
            label="Add Marker", command=self.add_marker_event, pass_coords=True
        )

        if self.map_name == "Commands Map":
            self.add_marker_event([self.drone.latitude, self.drone.longitude])
            self.update_drone_marker([self.drone.latitude, self.drone.longitude]) # Keep for possible later use, inaccurate with simulation

    def add_marker_event(self, coords):
        if self.map_name == "Commands Map":
            new_marker = self.map_widget.set_marker(
                coords[0], coords[1], text=str(len(self.markers))
            )
            self.markers.append(new_marker)
            self.marker_positions.append(new_marker.position)
            # set a path
            if len(self.markers) > 1:
                self.path = self.map_widget.set_path(self.marker_positions)
            self.drone.add_mission_item(coords[0], coords[1])
        elif self.map_name == "Starting Position Settings Map":
            # Remove previously placed marker
            while len(self.markers) > 0:
                self.markers[0].delete()
                self.markers.pop(0)
            # Place new marker
            new_marker = self.map_widget.set_marker(coords[0], coords[1])
            self.markers.append(new_marker)
        elif self.map_name == "Download Offline Tiles Map":
            # Remove marker if 2 or more are already placed
            while len(self.markers) > 1:
                self.markers[0].delete()
                self.markers.pop(0)
            # Place new marker
            new_marker = self.map_widget.set_marker(coords[0], coords[1])
            self.markers.append(new_marker)
        # Add marker placement to log
        self.logs.addUserLog(
            f"{self.map_name}: User added a marker on the map at: "
            + str(coords[0])
            + ", "
            + str(coords[1])
        )

    def update_drone_marker(self, coords):
        if self.map_name == "Commands Map":
            if self.drone_marker:
                self.drone_marker.delete()
            self.drone_marker = self.map_widget.set_marker(
                coords[0], coords[1], text="Drone"
            )

    def upload_plan(self):
        file_path = self.window_wrapper.display_input_box(
            "Enter the path to the plan file you want to upload:", title="Upload Plan"
        )

        if file_path and not os.path.isfile(file_path):
            self.window_wrapper.display_message("The file path you entered does not exist.")
        elif file_path:
            try:
                # Open and read the file line by line
                with open(file_path, mode="r") as file:
                    reader = csv.reader(file)
                    next(reader)  # Skip the header row

                    for row in reader:
                        latitude, longitude = row
                        self.add_marker_event([float(latitude), float(longitude)])
                        self.logs.addUserLog(
                            f"{self.map_name}: User added marker at ({latitude}, {longitude}) by uploading a custom flight path file."
                        )
                self.logs.addUserLog(
                    f"{self.map_name}: User uploaded a custom flight path file."
                )
            except Exception as e:
                self.window_wrapper.display_message("Error: An invalid flight path file was uploaded.")
                self.logs.addUserLog(
                    f"{self.map_name}: Error: User attempted to upload an invalid flight path file: {e}"
                )
                self.clear_marks()

    def clear_marks(self):
        for marker in self.markers:
            marker.delete()

        self.map_widget.delete_all_path()
        self.markers = []
        self.marker_positions = []
        self.path = None
        self.drone.clear_mission_items()

        if self.map_name == "Commands Map":
            self.add_marker_event([self.drone.latitude, self.drone.longitude])
            self.update_drone_marker([self.drone.latitude, self.drone.longitude]) # Keep for possible later use, inaccurate with simulation

        self.logs.addUserLog(f"{self.map_name}: User removed all marks on the map")
