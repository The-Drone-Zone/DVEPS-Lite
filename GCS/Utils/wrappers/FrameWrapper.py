import tkinter as tk


class FrameWrapper:
    def __init__(self):
        self.window = None

    def create_frame(self, window, **kwargs):
        frame = tk.Frame(window, **kwargs)
        frame.grid_propagate(False) 
        return frame

    def add_to_window(self, frame, row=0, column=0, **grid_options):
        """
        Add a frame to the window at the specified grid position.
        :param frame: The frame to add.
        :param row: The row position in the grid (default is 0).
        :param column: The column position in the grid (default is 0).
        :param grid_options: Additional grid options (e.g., padx, pady, sticky).
        """
        frame.grid(row=row, column=column, **grid_options)