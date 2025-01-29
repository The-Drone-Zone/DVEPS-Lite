from tkinter import ttk


class ButtonWrapper:
    def __init__(self):
        self.window = None
        self.style = ttk.Style()

    def create_button(self, window, text_size, **kwargs):
        self.style.configure(
            "Custom.TButton",
            font=("Helvetica", text_size),
            padding=(0, 0),
            anchor="center",
        )
        button = ttk.Button(window, style="Custom.TButton", **kwargs)
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
