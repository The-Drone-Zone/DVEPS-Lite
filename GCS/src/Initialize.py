import os
import sys
import tkinter as tk
from tkinter import ttk

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from CommandScreen import CommandScreen
from Utils.Globals import Globals
from Utils.Log import LoggingWindow


class Initialize:
    def __init__(self):
        self.root = tk.Tk()
        self.notebook = ttk.Notebook(self.root)

        self.Gobals = Globals()
        self.command_tab = CommandScreen(self.notebook, self.Gobals)
        self.logs_tab = LoggingWindow(self.notebook)

        self.screen_width = self.root.winfo_screenwidth()
        self.screen_height = self.root.winfo_screenheight()

        self.root.title("DVEPS-Lite Telemetry")
        self.root.geometry(f"{self.screen_width}x{self.screen_height}+0+0") # Fullscreen with toolbar shown


        self.notebook.pack(expand=True, fill='both')
        self.notebook.add(self.command_tab.command_tab, text="Command")
        self.notebook.add(self.logs_tab.logs_tab, text="Logs")
        self.notebook.bind("<<NotebookTabChanged>>", tab_selected) # TODO DELETE LATER

def tab_selected(event): # TODO DELETE LATER
    notebook = event.widget
    tab_id = notebook.select()
    tab_text = notebook.tab(tab_id, 'text')
    print(f"Selected Tab Text: {tab_text}") 