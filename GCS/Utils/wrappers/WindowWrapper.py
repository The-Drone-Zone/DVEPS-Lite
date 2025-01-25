import tkinter as tk
from tkinter import simpledialog, messagebox

class WindowWrapper:
    def __init__(self):
        self.windows = {}


    def create_window(self, window_id, title="Popup", width=300, height=200):
        if window_id in self.windows:
            print(f"Window with ID '{window_id}' already exists.")
            return

        window = tk.Toplevel()
        window.title(title)
        window.geometry(f"{width}x{height}")
        window.protocol("WM_DELETE_WINDOW", lambda: self.close_window(window_id, window))
        self.windows[window_id] = window


    def display_message(self, message, title="Message"):
        messagebox.showinfo(title, message)


    def display_input_box(self, prompt, title="Input"):
        return simpledialog.askstring(title, prompt)


    def destroy_window(self, window_id):
        if window_id not in self.windows:
            print(f"Window with ID '{window_id}' does not exist.")
            return

        self.windows[window_id].destroy()
        del self.windows[window_id]


    def close_window(self, window_id, window):
        window.destroy()
        if window_id in self.windows:
            del self.windows[window_id]


    #message and a input box all in one
    def display_custom_window(self, window_id, message=None, input_prompt=None):

        if window_id not in self.windows:
            print(f"Window with ID '{window_id}' does not exist.")
            return

        value: str = "nothing yet"
        input_value = tk.StringVar()

        window = self.windows[window_id]

        if message:
            label = tk.Label(window, text=message)
            label.pack(pady=10)

        if input_prompt:
            entry = tk.Entry(window, textvariable=input_value)
            entry.pack(pady=10)

            def on_submit():
                nonlocal value
                value = input_value.get()
                print(f"Input received: {value}, Window ID: {window_id}")
                entry.delete(0, tk.END)
                self.windows.pop(window_id, None)
                window.destroy()

            submit_button = tk.Button(window, text="Submit", command=on_submit)
            submit_button.pack(pady=10)

        window.wait_window() #do not delete: this makes sure the window is closed before returning
        return value
