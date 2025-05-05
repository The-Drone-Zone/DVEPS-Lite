from tkinter import ttk
import tkinter as tk


class ButtonWrapper:
    def __init__(self):
        self.window = None
        self.style = ttk.Style()

    def create_button(self, window, text_size=20, **kwargs):
        button = tk.Button(
            window, font=("Helvetica", text_size), anchor="center", bg="#e1e3de", **kwargs
        )
        self.add_hover_effect(button)

        return button

    def add_to_window(self, button, row=0, column=0, **grid_options):
        """
        Add a button to the window at the specified grid position.
        :param button: The button to add.
        :param row: The row position in the grid (default is 0).
        :param column: The column position in the grid (default is 0).
        :param grid_options: Additional grid options (e.g., padx, pady, sticky).
        """
        button.grid(row=row, column=column, **grid_options)

    def add_centered_button(self, button, y=30):
        """
        Add a button to the frame, centered within it.
        :param button: The button to be placed.
        :param y: screen units in y-direction to be placed.
        """
        button.place(relx=0.5, y=y, anchor=tk.CENTER)
        button.lift()
        return button
    
    def add_hover_effect(self, button):
        button.bind("<Enter>", lambda e: button.config(bg="#babdb7"))
        button.bind("<Leave>", lambda e: button.config(bg="#e1e3de"))
