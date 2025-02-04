import tkinter as tk
from tkintermapview import TkinterMapView
import os
from Utils.connected import is_connected
from LogWindow import LoggingWindow


class Map:
    def __init__(self, map_frame, globals, logs: LoggingWindow):
        # Initialize global variables
        self.globals = globals
        self.window_wrapper = globals.window_wrapper
        self.logs = logs

        # Initialize map widget
        if is_connected():
            print("Using Map with Internet")
            self.map_widget = TkinterMapView(map_frame, corner_radius=0)
        else:
            print("Using Offline Map")
            script_directory = os.path.dirname(
                os.path.abspath(__file__)
            )  # Current script directory
            parent_directory = os.path.dirname(script_directory)  # Move one folder back
            database_path = os.path.join(
                parent_directory, "offline_tiles.db"
            )  # DB in parent dir
            self.map_widget = TkinterMapView(
                map_frame,
                corner_radius=0,
                use_database_only=True,
                max_zoom=22,
                database_path=database_path,
            )
            # self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga")

        # Set widget location and size
        self.map_widget.place(relx=0, rely=0, relwidth=1, relheight=1)

        # Set Starting Location
        self.map_widget.set_position(
            32.2319, -110.9535
        )  # University of Arizona, Tucson, AZ

        # Track markers
        self.markers = []
        self.marker_positions = []
        self.path = None

        # Add Create Marker Event
        self.map_widget.add_right_click_menu_command(
            label="Add Marker", command=self.add_marker_event, pass_coords=True
        )

    def add_marker_event(self, coords):
        print("Add marker:", coords)
        new_marker = self.map_widget.set_marker(
            coords[0], coords[1], text=str(len(self.markers) + 1)
        )
        self.markers.append(new_marker)
        self.marker_positions.append(new_marker.position)
        # set a path
        self.path = self.map_widget.set_path(self.marker_positions)
        self.logs.addUserLog("User added a marker on the map at: " + str(coords[0]) + ", " + str(coords[1]))

    def upload_plan(self):
        file_path = self.window_wrapper.display_input_box(
            "Enter the path to the plan file you want to upload:", title="Upload Plan"
        )

        if file_path and not os.path.isfile(file_path):
            file_path = self.globals.Utils.ask_for_input(file_path)

        with open(file_path, "r") as file:
            content = file.read()

        print(content)
        self.logs.addUserLog("User uploaded a custom flight path path")

    def clear_marks(self):
        for marker in self.markers:
            marker.delete()

        self.map_widget.delete_all_path()
        self.markers = []
        self.marker_positions = []
        self.path = None

        self.logs.addUserLog("User removed all marks on the map")
