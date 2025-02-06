import tkinter as tk
from tkintermapview import TkinterMapView
import os
from Utils.connected import is_connected
from LogWindow import LoggingWindow
from Utils.Utils import get_root_dir


class Map:
    def __init__(self, map_frame, map_name, globals, logs: LoggingWindow, settings):
        # Initialize global variables
        self.globals = globals
        self.window_wrapper = globals.window_wrapper
        self.logs = logs
        self.settings = settings

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
        self.markers = []
        self.marker_positions = []
        self.path = None

        # Add Create Marker Event
        self.map_widget.add_right_click_menu_command(
            label="Add Marker", command=self.add_marker_event, pass_coords=True
        )

    def add_marker_event(self, coords):
        if self.map_name == "Commands Map":
            new_marker = self.map_widget.set_marker(
                coords[0], coords[1], text=str(len(self.markers) + 1)
            )
            self.markers.append(new_marker)
            self.marker_positions.append(new_marker.position)
            # set a path
            if len(self.markers) > 1:
                self.path = self.map_widget.set_path(self.marker_positions)
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

    def upload_plan(self):
        file_path = self.window_wrapper.display_input_box(
            "Enter the path to the plan file you want to upload:", title="Upload Plan"
        )

        if file_path and not os.path.isfile(file_path):
            file_path = self.globals.Utils.ask_for_input(file_path)

        with open(file_path, "r") as file:
            content = file.read()

        self.logs.addUserLog(
            f"{self.map_name}: User uploaded a custom flight path path"
        )

    def clear_marks(self):
        for marker in self.markers:
            marker.delete()

        self.map_widget.delete_all_path()
        self.markers = []
        self.marker_positions = []
        self.path = None

        self.logs.addUserLog(f"{self.map_name}: User removed all marks on the map")
