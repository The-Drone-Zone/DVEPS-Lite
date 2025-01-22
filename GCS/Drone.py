#TODO add imports here
from Utils.wrappers.WindowWrapper import WindowWrapper

class Drone:
    def __init__(self, WW):
        self.window_wrapper: WindowWrapper = WW
        pass

    def command_drone(self, selected_option):
        if selected_option == "Takeoff":
            print("Takeoff") #TODO make a logs class so we can log user input
            self.takeoff_drone()
        elif selected_option == "Set Speed":
            print("Set Speed") #TODO make a logs class so we can log user input
        elif selected_option == "Set Altitude":
            print("Set Altitude") #TODO make a logs class so we can log user input
        elif selected_option == "Set Heading":
            print("Set Heading") #TODO make a logs class so we can log user input


    def land_drone(self):
        #TODO hook up drone and sim
        print("Landing Drone") #TODO make a logs class so we can log user input


    def hover_drone(self):
        #TODO Hook up drone and sim
        print("Hovering Drone") #TODO make a logs class so we can log user input


    def takeoff_drone(self):
        take_off_value: str = "000"


        self.window_wrapper.create_window("takeoff", title="Takeoff", width=300, height=200)
        take_off_value = self.window_wrapper.display_custom_window("takeoff", message="Enter the desired altitude:", input_prompt="Altitude")
        self.window_wrapper.destroy_window("takeoff")

        if not take_off_value:
            take_off_value = "xxx"
        print(f"Altitude: {take_off_value}") #TODO make a logs class so we can log user input


    def set_speed(self):
        pass #stubbed for now


    def set_altitude(self):
        pass #stubbed for now


    def set_heading(self):
        pass  #stubbed for now