# TODO add imports here
from Utils.Enums import DRONE_STATE
from Utils.wrappers.WindowWrapper import WindowWrapper

class Drone:
    def __init__(self, WW, logs):
        self.window_wrapper: WindowWrapper = WW
        self.altitude: int = 0
        self.speed: float = 0.0
        self.heading: int = 0
        self.drone_status: DRONE_STATE = DRONE_STATE.LANDED
        self.logs = logs

        pass

    def command_drone(self, selected_option):
        if selected_option == "Takeoff":
            print("Takeoff") 
            self.logs.addUserLog("User selected the Takeoff command")
            self.takeoff_drone()
        elif selected_option == "Set Speed":
            print("Set Speed") 
            self.logs.addUserLog("User selected the Set Speed command")
            self.set_speed()
        elif selected_option == "Set Altitude":
            print("Set Altitude")
            self.logs.addUserLog("User selected the Set Altitude command")
            self.set_altitude()
        elif selected_option == "Set Heading":
            print("Set Heading")
            self.logs.addUserLog("User selected the Set Heading command")
            self.set_heading()

    def land_drone(self):
        # TODO hook up drone and sim
        print("Landing Drone")
        self.logs.addUserLog("User selected the Land Drone command")

    def hover_drone(self):
        # TODO Hook up drone and sim
        print("Hovering Drone")
        self.logs.addUserLog("User selected the Hover Drone command")

    # drone is landed and we take off to a certain height
    def takeoff_drone(self):
        if (
            DRONE_STATE.LANDED != self.drone_status
        ):  # TODO  this probably should happen in a different way.
            return

        take_off_value: str = ""

        self.window_wrapper.create_window(
            "takeoff", title="Takeoff", width=300, height=200
        )
        take_off_value = self.window_wrapper.display_custom_window(
            "takeoff", message="Enter the desired altitude:", input_prompt="Altitude"
        )
        self.window_wrapper.destroy_window("takeoff")

        if not take_off_value:
            take_off_value = "xxx"

        self.altitude = int(take_off_value)
        print(
            f"Altitude: {take_off_value}"
        )  # TODO make a logs class so we can log user input

    def set_speed(self):
        self.window_wrapper.create_window(
            "Set Speed", title="Set Speed (m/s)", width=300, height=200
        )
        speed_value = self.window_wrapper.display_custom_window(
            "Set Speed", message="Enter the desired speed:", input_prompt="Speed"
        )
        self.window_wrapper.destroy_window("Set Speed")

        if not speed_value:
            speed_value = "xxx"

        self.speed = float(speed_value)
        print(f"Speed: {speed_value}")

    def set_altitude(self):
        self.window_wrapper.create_window(
            "Set Altitude", title="Set Altitude (m)", width=300, height=200
        )
        altitude_value = self.window_wrapper.display_custom_window(
            "Set Altitude",
            message="Enter the desired altitude:",
            input_prompt="Altitude",
        )
        self.window_wrapper.destroy_window("Set Altitude")

        if not altitude_value:
            altitude_value = "xxx"

        self.altitude = int(altitude_value)
        print(f"Altitude: {altitude_value}")

    def set_heading(self):
        self.window_wrapper.create_window(
            "Set Heading", title="Set Heading (degrees)", width=300, height=200
        )
        heading_value = self.window_wrapper.display_custom_window(
            "Set Heading", message="Enter the desired heading:", input_prompt="Heading"
        )
        self.window_wrapper.destroy_window("Set Heading")

        if not heading_value:
            heading_value = "xxx"

        self.heading = int(heading_value)
        print(f"Heading: {heading_value}")
