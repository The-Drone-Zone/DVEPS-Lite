import serial
import socket
import time
import os

class MsgForwarding:
    def __init__(self):
        self.stop = False

    def forwardMavLinkMsgs(self):
        # === CONFIGURE SERIAL CONNECTION ===
        if os.name == "nt":  # Windows
            SERIAL_PORT = "COM3"
        elif os.name == "posix":  # Linux/macOS
            SERIAL_PORT = "/dev/ttyUSB0"
        BAUDRATE = 57600

        # === CONFIGURE UDP FORWARDING ===
        UDP_TARGETS = [("127.0.0.1", 14550),  # MAVSDK
                    ("127.0.0.1", 14551)]  # pymavlink
        

        while not self.stop:
            try:
                # Open serial connection to Pixhawk
                ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

                # Create UDP socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                
                print(f"Forwarding MAVLink from {SERIAL_PORT} to {UDP_TARGETS}")

                while True and not self.stop:
                    data = ser.read(1024)  # Read MAVLink data from serial
                    if data:
                        for target in UDP_TARGETS:
                            sock.sendto(data, target)  # Forward to all UDP targets
            except Exception:
                print('Error forwarding MavLink Messages (Device likely not connected)')
                time.sleep(1)