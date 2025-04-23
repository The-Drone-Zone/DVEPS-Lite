from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from Utils.Enums import DRONE_STATE
from Utils.wrappers.WindowWrapper import WindowWrapper
import asyncio
import os
import threading


class Drone:
    def __init__(self, WW, logs, settings):
        self.window_wrapper: WindowWrapper = WW

        self.drone = None
        self.heading: int = 0
        self.drone_status: DRONE_STATE = DRONE_STATE.LANDED
        self.mission_items = []
        self.latitude = 0
        self.longitude = 0
        self.connected = False
        self.disconnect_counter = 0 # For checking connection (2 missed/false connections = disconnection)
        self.gps_ok = False
        self.armable = False
        self.battery_percent = 0

        self.logs = logs
        self.settings = settings
        self.command_tab = None

        # Initialize separate thread for running asyncio loop
        self.loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.async_thread.start()

        asyncio.run_coroutine_threadsafe(self.setup(), self.loop)

    def run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def setup(self):
        # Connect to Drone
        # self.drone = System(mavsdk_server_address="localhost", port=50051)
        # await self.drone.connect(
        #     system_address="udp://:14550"
        # ) 

        if os.name == "nt":  # Windows

            self.drone = System(mavsdk_server_address="localhost", port=50051)
            await self.drone.connect(
                system_address="serial://COM3:57600"
            )
        elif os.name == "posix":  # Linux/macOS

            self.drone = System()
            await self.drone.connect(
                system_address="serial:///dev/ttyUSB0:57600"
            )


        # Setup Drone Configuration based on Settings
        asyncio.run_coroutine_threadsafe(self.set_takeoff_height_drone(), self.loop)
        asyncio.run_coroutine_threadsafe(self.set_speed_drone(), self.loop)

        # Start telemetry log loops
        asyncio.run_coroutine_threadsafe(self.print_status_text(), self.loop)
        asyncio.run_coroutine_threadsafe(self.print_battery(), self.loop)
        asyncio.run_coroutine_threadsafe(self.print_gps_info(), self.loop)
        asyncio.run_coroutine_threadsafe(self.get_position(), self.loop)

        # Start drone health/connection loops
        asyncio.run_coroutine_threadsafe(self.check_drone_connection(), self.loop)
        asyncio.run_coroutine_threadsafe(self.check_drone_health(), self.loop)

        print("Waiting for drone to connect...")
        self.logs.addDroneLog("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True
                self.disconnect_counter = 0
                self.logs.addDroneLog("-- Connected to drone!")
                print(f"-- Connected to drone!")
                if self.command_tab:
                    self.command_tab.update_drone_connected()
                break

        print("Waiting for drone to have a global position estimate...")
        self.logs.addDroneLog("Waiting for drone to have a global position estimate...")

    ## Telemetry Loop functions begin here ##
    async def print_status_text(self):
        async for status_text in self.drone.telemetry.status_text():
            if self.connected:
                self.logs.addDroneLog(f"Status: {status_text.type}: {status_text.text}")

    async def print_battery(self):
        async for battery in self.drone.telemetry.battery():
            if self.connected and self.gps_ok:
                self.battery_percent = battery.remaining_percent
                self.logs.addDroneLog(f"Battery: {battery.remaining_percent}%")
                if self.command_tab:
                    self.command_tab.update_drone_battery()
            await asyncio.sleep(2)

    async def print_gps_info(self):
        async for gps_info in self.drone.telemetry.gps_info():
            if self.connected and self.gps_ok:
                self.logs.addDroneLog(f"{gps_info}")
            await asyncio.sleep(2)

    async def get_position(self):
        async for position in self.drone.telemetry.position():
            if self.connected and self.gps_ok:
                self.latitude = position.latitude_deg
                self.longitude = position.longitude_deg
                self.logs.addDroneLog(
                    f"Position: latitude: {position.latitude_deg}°, longitude: {position.longitude_deg}°"
                )
                self.logs.addDroneLog(
                    f"Altitude: relative: {round(position.relative_altitude_m, 3)} m, absolute: {round(position.absolute_altitude_m, 3)} m"
                )
                if self.command_tab and self.command_tab.map: # Keep for possible later use, inaccurate with simulation
                    self.command_tab.map.update_drone_marker([self.latitude, self.longitude])
            await asyncio.sleep(2)

    async def check_drone_connection(self):
        async for state in self.drone.core.connection_state():
            print(f"Connection: {state.is_connected}")
            if state.is_connected and not self.connected:
                self.logs.addDroneLog("-- Connected to drone!")
                self.connected = True
                self.disconnect_counter = 0
                if self.command_tab:
                    self.command_tab.update_drone_connected()
            elif not state.is_connected and self.connected and self.disconnect_counter >= 2: # 3rd false connection in a row
                self.logs.addDroneLog("-- Disconnected from drone")
                self.logs.addDroneLog("Waiting for drone to connect...")
                self.connected = False
                self.disconnect_counter += 1
                if self.command_tab:
                    self.command_tab.update_drone_connected()
            elif not state.is_connected:
                self.disconnect_counter += 1

    async def check_drone_health(self):
        async for health in self.drone.telemetry.health():
            # Check GPS estimates
            print(f"global: {health.is_global_position_ok} | local: {health.is_local_position_ok}") # Keep for GPS debugging
            if health.is_global_position_ok and health.is_home_position_ok and not self.gps_ok:
                self.gps_ok = True
                self.logs.addDroneLog("-- Global position estimate OK")
                print("-- Global position estimate OK")
                if self.command_tab:
                    self.command_tab.update_drone_connected()
            elif (not health.is_global_position_ok or not health.is_home_position_ok) and self.gps_ok:
                self.gps_ok = False
                self.logs.addDroneLog("-- Global position estimate FAILED")
                self.logs.addDroneLog("Waiting for drone to have a global position estimate...")
                if self.command_tab:
                    self.command_tab.update_drone_connected()
            # Check if drone is armable (pre-flight checklist)
            if health.is_armable and not self.armable:
                self.armable = True
                self.logs.addDroneLog("-- Drone is armable and ready for flight")
                if self.command_tab:
                    self.command_tab.update_drone_connected()
            elif (not health.is_armable) and self.armable:
                self.armable = False
                self.logs.addDroneLog("-- Drone not armable, check pre-flight errors")
                if self.command_tab:
                    self.command_tab.update_drone_connected()
            await asyncio.sleep(1)

    ## Button Click Event Handlers begin here ##
    def command_drone(self, selected_option):
        if selected_option == "Begin Mission":
            self.logs.addUserLog("User selected the Begin Mission command")
            mission_plan = MissionPlan(self.mission_items)
            asyncio.run_coroutine_threadsafe(
                self.start_mission_drone(mission_plan), self.loop
            )
        elif selected_option == "Pause Mission":
            self.logs.addUserLog("User selected the Pause Mission command")
            asyncio.run_coroutine_threadsafe(self.pause_mission_drone(), self.loop)
        elif selected_option == "Takeoff":
            self.logs.addUserLog("User selected the Takeoff command")
            asyncio.run_coroutine_threadsafe(self.takeoff_drone(), self.loop)
        elif selected_option == "Arm":
            self.logs.addUserLog("User selected the Arm command")
            asyncio.run_coroutine_threadsafe(self.arm_drone(), self.loop)
        elif selected_option == "Disarm":
            self.logs.addUserLog("User selected the Disarm command")
            asyncio.run_coroutine_threadsafe(self.disarm_drone(), self.loop)

    def land_command(self):
        self.logs.addUserLog("User selected the Land Drone command")
        asyncio.run_coroutine_threadsafe(self.land_drone(), self.loop)

    def hover_command(self):
        self.logs.addUserLog("User selected the Hover Drone command")
        asyncio.run_coroutine_threadsafe(self.hover_drone(), self.loop)

    def set_takeoff_height(self):
        asyncio.run_coroutine_threadsafe(self.set_takeoff_height_drone(), self.loop)

    def set_speed(self):
        asyncio.run_coroutine_threadsafe(self.set_speed_drone(), self.loop)

    ## ASync MavLink command functions begin here ##
    async def set_takeoff_height_drone(self):
        await self.drone.action.set_takeoff_altitude(self.settings.flight_height)

    async def set_speed_drone(self):
        await self.drone.action.set_current_speed(self.settings.flight_speed)

    async def land_drone(self):
        await self.drone.action.land()

    async def hover_drone(self):
        await self.drone.action.hold()

    async def start_mission_drone(self, mission_plan):
        await self.drone.mission.clear_mission()
        self.logs.addDroneLog("-- Uploading Mission")
        await self.drone.mission.upload_mission(mission_plan)
        await self.drone.action.arm()
        self.logs.addDroneLog("-- Starting Mission")
        await self.drone.mission.start_mission()

    async def pause_mission_drone(self):
        await self.drone.mission.pause_mission()

    # drone is landed and we take off to a certain height
    async def takeoff_drone(self):
        # if (
        #     DRONE_STATE.LANDED != self.drone_status
        # ):
        #     return

        await self.drone.action.arm()
        await self.drone.action.takeoff()

    async def arm_drone(self):
        await self.drone.action.arm()

    async def disarm_drone(self):
        await self.drone.action.disarm()

    ## Mission Creation helper functions
    def add_mission_item(self, latitude, longitude):
        self.mission_items.append(
            MissionItem(
                latitude,  # latitude_deg (double) – Latitude in degrees (range: -90 to +90)
                longitude,  # longitude_deg (double) – Longitude in degrees (range: -180 to +180)
                self.settings.flight_height,  # relative_altitude_m (float) – Altitude relative to takeoff altitude in metres
                self.settings.flight_speed,  # speed_m_s (float) – Speed to use after this mission item (in metres/second)
                True,  # is_fly_through (bool) – True will make the drone fly through without stopping, while false will make the drone stop on the waypoint
                float("nan"),  # gimbal_pitch_deg (float) – Gimbal pitch (in degrees)
                float("nan"),  # gimbal_yaw_deg (float) – Gimbal yaw (in degrees)
                MissionItem.CameraAction.NONE,  # camera_action (CameraAction) – Camera action to trigger at this mission item
                float("nan"),  # loiter_time_s (float) – Loiter time (in seconds)
                float(
                    "nan"
                ),  # camera_photo_interval_s (double) – Camera photo interval to use after this mission item (in seconds)
                float(
                    "nan"
                ),  # acceptance_radius_m (float) – Radius for completing a mission item (in metres)
                float(
                    "nan"
                ),  # yaw_deg (float) – Absolute yaw angle (in degrees) camera_photo_distance_m (float) – Camera photo distance to use after this mission item (in meters)
                float(
                    "nan"
                ),  # camera_photo_distance_m (float) – Camera photo distance to use after this mission item (in meters)
                MissionItem.VehicleAction.NONE,
            )  # vehicle_action (VehicleAction) – Vehicle action to trigger at this mission item.
        )

    def clear_mission_items(self):
        self.mission_items = []
