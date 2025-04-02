import asyncio
from mavsdk import System
from pymavlink import mavutil


async def run_mavsdk():

    drone = System(mavsdk_server_address="localhost", port=50051)
    # await drone.connect(system_address="udp://:14550")
    await drone.connect(system_address="serial://COM3:57600")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # print("Waiting for drone to have a global position estimate...")
    # async for health in drone.telemetry.health():
    #     if health.is_global_position_ok and health.is_home_position_ok:
    #         print("-- Global position estimate OK")
    #         break

    # print("-- Arming")
    # await drone.action.arm()

    # print("-- Taking off")
    # await drone.action.takeoff()

    await asyncio.sleep(10)

    # print("-- Landing")
    # await drone.action.land()

    status_text_task.cancel()

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


def setup_mavlink_connection():
    # Connect pymavlink to MAVSDK Server's UDP output
    print("above master")
    # master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master = mavutil.mavlink_connection("COM3", baud=57600)

    # Wait for a heartbeat to confirm connection
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Connected to MAVLink via MAVSDK Server!")

    return master

def req_mavlink_msg(connection):
    # Define command_long_encode message to send MAV_CMD_REQUEST_MESSAGE command
    # param1: MAVLINK_MSG_ID_OBSTACLE_DISTANCE (message to stream)
    # param2: 1000000 (Stream interval in microseconds)
    message = connection.mav.command_long_encode(
            connection.target_system,  # Target system ID
            connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # ID of command to send
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE,  # param1: Message ID to be streamed
            1000000, # param2: Interval in microseconds
            0,       # param3 (unused)
            0,       # param4 (unused)
            0,       # param5 (unused)
            0,       # param5 (unused)
            0        # param6 (unused)
            )

    # Send the COMMAND_LONG
    connection.mav.send(message)

    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = connection.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")
    
def set_mavlink_msg_interval(connection):
    # Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
    # param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
    # param2: 1000000 (Stream interval in microseconds)
    message = connection.mav.command_long_encode(
            connection.target_system,  # Target system ID
            connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,  # param1: Message ID to be streamed
            1000000, # param2: Interval in microseconds
            0,       # param3 (unused)
            0,       # param4 (unused)
            0,       # param5 (unused)
            0,       # param5 (unused)
            0        # param6 (unused)
            )

    # Send the COMMAND_LONG
    connection.mav.send(message)

    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = connection.recv_match(type='COMMAND_ACK', blocking=True)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Command accepted")
    else:
        print("Command failed")

    
def get_mavlink_msg(master):
    while True:
        msg = master.recv_match(type='OBSTACLE_DISTANCE', blocking=True)
        if msg:
            print(f"Received Obstacle Distance: {msg.distances}")
            break


if __name__ == "__main__":
    # Run the asyncio loop
    # asyncio.run(run_mavsdk())
    master = setup_mavlink_connection()
    # req_mavlink_msg(master)
    set_mavlink_msg_interval(master)
    # get_mavlink_msg(master)
