import os
import sys
import tkinter as tk
from tkinter import ttk

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from Drone import Drone
from LogWindow import LoggingWindow
from CommandScreen import CommandScreen
from Settings import Settings
from Utils.Globals import Globals


class Initialize:
    def __init__(self):
        self.root = tk.Tk()
        self.notebook = ttk.Notebook(self.root)

        self.globals = Globals()
        self.logs_tab = LoggingWindow(self.notebook, self.globals)
        self.settings = Settings(self.notebook, self.globals, self.logs_tab)
        self.drone = Drone(self.globals.window_wrapper, self.logs_tab, self.settings)
        self.command_tab = CommandScreen(
            self.notebook, self.globals, self.drone, self.logs_tab, self.settings
        )
        self.drone.command_tab = self.command_tab

        self.screen_width = self.root.winfo_screenwidth()
        self.screen_height = self.root.winfo_screenheight()

        self.root.title("DVEPS-Lite Telemetry")
        self.root.geometry(
            f"{self.screen_width}x{self.screen_height}+0+0"
        )  # Fullscreen with toolbar shown

        self.notebook.pack(expand=True, fill="both")
        self.notebook.add(self.command_tab.command_tab, text="Command")
        self.notebook.add(self.logs_tab.logs_tab, text="Logs")
        self.notebook.add(self.settings.settings_tab, text="Settings")
        self.notebook.bind(
            "<<NotebookTabChanged>>", self.tab_selected
        )  # TODO DELETE LATER

    def tab_selected(self, event):  # TODO DELETE LATER
        notebook = event.widget
        tab_id = notebook.select()
        tab_text = notebook.tab(tab_id, "text")
        self.logs_tab.addUserLog(f"User switched to the {tab_text} tab")
