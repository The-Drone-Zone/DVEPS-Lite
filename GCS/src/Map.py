from collections import deque
import os
from PIL import Image, ImageTk
import tkinter as tk

class Map:
    def __init__(self, window_id, globals):
        # self.resize_active = True
        self.globals = globals
        self.window_wrapper = globals.Widndow_wrapper

        self.window_id = window_id
        self.image = Image.open("GCS/images/Map.jpg")
        self.original_image = self.image.copy()
        self.img_height = self.image.height
        self.img_width = self.image.width

        self.CONST_BUTTON_HEIGHT = 35

        self.first_open = True
        self.clear_marks_btn = None
        self.make_route_btn = None

        self.create_map()

        self.waypoint_queue = deque(maxlen=100)
#this is the example coordinates of my picture in tucson.
#32°13'17.0"N 110°57'33.8"W BL
#32°13'17.5"N 110°54'34.5"W BR
#32°15'28.1"N 110°54'35.0"W TR
#32°15'27.4"N 110°57'36.1"W TL
        self.top_left = (32.15274, 110.57361)
        self.top_right = (32.15281, 110.5435)
        self.bottom_left = (32.1317, 110.57338)
        self.bottom_right = (32.13175, 110.54345)


    def upload_plan(self):
        file_path = self.window_wrapper.display_input_box("Enter the path to the plan file you want to upload:", title="Upload Plan")

        if(file_path and not os.path.isfile(file_path)):
            file_path = self.globals.Utils.ask_for_input(file_path)

        with open(file_path, "r") as file:
                content = file.read()
        
        print(content)


    def add_map(self):
        file_path = self.window_wrapper.display_input_box("Enter the path to the map image you want to add:", title="Add Map")

        if(file_path and not os.path.isfile(file_path)):
            print("in if")
            file_path = self.globals.Utils.ask_for_input(file_path)

        print("skipping if")
        self.upload_map_image(file_path)


    def upload_map_image(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f"File not found: {path}")

        self.original_image = Image.open(path)
        self.image = self.original_image.copy()
        self.img_height = self.image.height
        self.img_width = self.image.width

        self.photo = ImageTk.PhotoImage(self.image)

        # Update the canvas dimensions and image
        if self.window_id in self.window_wrapper.windows:
            window = self.window_wrapper.windows[self.window_id]

            if window.winfo_exists():
                window.geometry(f"{self.img_width}x{self.img_height}")
                self.canvas.config(width=self.img_width, height=self.img_height)
                self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

        else:
            self.create_map()

        self.window_wrapper.display_message("Map image uploaded successfully.\n Click on the \"Open Map\" button to view the map.")


    def open_map(self):
        if self.window_id in self.window_wrapper.windows:
            window = self.window_wrapper.windows[self.window_id]
            window.deiconify()
            window.update_idletasks()
            total_height = window.winfo_height()
            if self.first_open:
                window.geometry(f'{self.img_width}x{total_height + 3*self.CONST_BUTTON_HEIGHT}')
                self.first_open = False


    def hide_map(self):
        if self.window_id in self.window_wrapper.windows:
            window = self.window_wrapper.windows[self.window_id]
            window.withdraw()


    def destroy_map(self):
        if self.window_id in self.window_wrapper.windows:
            window = self.window_wrapper.windows[self.window_id]
            window.destroy()


    def clear_marks(self):
        self.canvas.delete("waypoint")
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        self.waypoint_queue.clear()
        self.clear_marks_btn.pack_forget()
        self.make_route_btn.pack_forget()


    def create_map(self):
        self.window_wrapper.create_window(self.window_id, title="Map Window", width=self.img_width, height=self.img_height)
        window = self.window_wrapper.windows[self.window_id]

        self.photo = ImageTk.PhotoImage(self.image)
        self.canvas = tk.Canvas(window, width=self.img_width, height=self.img_height, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        self.canvas.bind("<Button-1>", self.on_map_click)

        hide_window_btn = self.globals.Button_wrapper.create_button(window, 20, text="Hide Map", command=self.hide_map)
        hide_window_btn.pack(side=tk.BOTTOM, fill=tk.X)

        self.clear_marks_btn = self.globals.Button_wrapper.create_button(window, 20, text="Clear Marks", command=self.clear_marks)
        self.clear_marks_btn.pack_forget()

        self.make_route_btn = self.globals.Button_wrapper.create_button(window, 20, text="Make Route", command=self.make_route)
        self.make_route_btn.pack_forget()

        self.hide_map()


    def on_map_click(self, event):
        x, y = event.x, event.y
        lat, lon = self.pixel_to_gps(x, y, self.canvas.winfo_width(), self.canvas.winfo_height(), self.top_left, self.bottom_right)
        print(f"Clicked at pixel: ({x}, {y}), GPS Coordinates: ({lat:.6f}, {lon:.6f})")
        radius = 5

        label_number = len(self.waypoint_queue) + 1
        # self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill="red", outline="black", tags="waypoint")
        self.canvas.create_text(x, y - radius - 10, text=str(label_number), fill="red", font=("Arial", 10), tags="waypoint")

        self.waypoint_queue.append((lat, lon))
        self.clear_marks_btn.pack(side=tk.BOTTOM, fill=tk.X)
        self.make_route_btn.pack(side=tk.BOTTOM, fill=tk.X)


    #ChatGPT inspired function ;) <--winky face
    #unsure about performance accuracy
    def pixel_to_gps(self, x, y, img_width, img_height, top_left, bottom_right):
        lat_top, lon_left = top_left
        lat_bottom, lon_right = bottom_right

        latitude = lat_top - ((lat_top - lat_bottom) * (y / img_height))
        longitude = lon_left + ((lon_right - lon_left) * (x / img_width))

        return latitude, longitude


    def make_route(self):
        self.window_wrapper.display_message("Route created successfully.")
        self.make_route_btn.pack_forget()
