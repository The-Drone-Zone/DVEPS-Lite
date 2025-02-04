from tkinter import Scrollbar
from tkinter import Text
import tkinter as tk
from Utils.Globals import Globals


class LoggingWindow:
    def __init__(self, notebook, globals: Globals):
        # Initialize globals
        self.frame_wrapper = globals.frame_wrapper
        self.button_wrapper = globals.button_wrapper
        self.window_wrapper = globals.window_wrapper

        # Create main frame
        self.logs_tab = self.frame_wrapper.create_frame(
            window=notebook, name="Logs Tab"
        )

        # Configure the separate logs areas
        self.logs_tab.grid_rowconfigure(0, weight=1)
        self.logs_tab.grid_rowconfigure(1, weight=1)
        self.logs_tab.grid_columnconfigure(0, weight=1)

        # Create Log Areas
        self.drone_logs_text = self.createDroneLogs()
        self.user_logs_text = self.createUserLogs()

    def createDroneLogs(self):
        # Create Drone logs frame
        drone_logs_frame = self.frame_wrapper.create_frame(
            window=self.logs_tab, name="Drone Logs", bg="lightblue", padx=50
        )
        self.frame_wrapper.add_to_window(
            drone_logs_frame, row=0, column=0, sticky="nsew"
        )

        # Add Title
        title = tk.Label(drone_logs_frame, text="Drone Logs", justify=tk.CENTER, bg="lightblue", font=("Helvetica", 20, "bold"))
        title.pack()

        # Setup Scrollbar
        drone_logs_scrollbar = Scrollbar(drone_logs_frame)
        drone_logs_scrollbar.pack(side = "right", fill = "y")

        # Setup text widget
        drone_logs_text = Text(drone_logs_frame, yscrollcommand = drone_logs_scrollbar.set, padx=50, pady=10)
        
        # Insert text into the text widget (DELETE ONCE REAL DATA)
        for i in range(40):
            drone_logs_text.insert(tk.END, "DRONE TEXT DRONE COMMAND DRONES STUFF LOGS TELEM RANDOOOO\n")
        drone_logs_text.see('end')

        # Disable Text Editing
        drone_logs_text.config(state="disabled")

        # attach Text widget to root window at top
        drone_logs_text.pack(side=tk.LEFT, fill="both", expand=True)

        return drone_logs_text

    def createUserLogs(self):
        # Create User logs frame
        user_logs_frame = self.frame_wrapper.create_frame(
            window=self.logs_tab, name="User Logs", bg="yellow", padx=50
        )
        self.frame_wrapper.add_to_window(
            user_logs_frame, row=1, column=0, sticky="nsew"
        )

        # Add Title
        title = tk.Label(user_logs_frame, text="User Logs", justify=tk.CENTER, bg="yellow", font=("Helvetica", 20, "bold"))
        title.pack()

        # Setup Scrollbar
        user_logs_scrollbar = Scrollbar(user_logs_frame)
        user_logs_scrollbar.pack(side = "right", fill = "y")

        # Setup text widget
        user_logs_text = Text(user_logs_frame, yscrollcommand = user_logs_scrollbar.set, padx=50, pady=10)

        # Disable Text Editing
        user_logs_text.config(state="disabled")

        # attach Text widget to root window at top
        user_logs_text.pack(side=tk.LEFT, fill="both", expand=True)

        return user_logs_text
    
    def addDroneLog(self, text):
        # Enable Text Editing
        self.drone_logs_text.config(state="normal")
        # Insert New Log Text
        self.drone_logs_text.insert(tk.END, text + "\n")
        self.drone_logs_text.see('end')
        # Disable Text Editing
        self.drone_logs_text.config(state="disabled")

    def addUserLog(self, text):
        # Enable Text Editing
        self.user_logs_text.config(state="normal")
        # Insert New Log Text
        self.user_logs_text.insert(tk.END, text + "\n")
        self.user_logs_text.see('end')
        # Disable Text Editing
        self.user_logs_text.config(state="disabled")