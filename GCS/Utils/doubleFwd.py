import serial
import socket
import time
import os
import threading

class MsgForwarding:
    def __init__(self):
        self.stop = False
        self.udp_listen_threads = []

    def forwardMavLinkMsgs(self):
        # === CONFIGURE SERIAL CONNECTION ===
        if os.name == "nt":  # Windows
            SERIAL_PORT = "COM3"
        elif os.name == "posix":  # Linux/macOS
            SERIAL_PORT = "/dev/ttyUSB0"
        BAUDRATE = 57600

        # === CONFIGURE UDP FORWARDING ===
        # UDP_SOURCE = ("127.0.0.1", 14550)  # For Simulated MAVLink input (e.g., SITL) (USE FOR SITL SIMULATION ONLY)
        UDP_TARGETS = [("127.0.0.1", 14551),  # MAVSDK
                       ("127.0.0.1", 14552)]  # pymavlink
        UDP_LISTEN_PORTS = [14551, 14552]  # Ports to listen for outgoing messages

        try:
            # Open serial connection to Pixhawk
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) # USE FOR PIXHAWK

            # Create UDP socket for receiving MAVLink from the simulator (USE FOR SITL SIMULATION ONLY)
            # ser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # ser.bind(UDP_SOURCE)
            # ser.settimeout(1)  # Prevent blocking forever


            # Create UDP socket for sending
            udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            print(f"Forwarding MAVLink from {SERIAL_PORT} to {UDP_TARGETS}")

            # Start threads to listen for UDP messages from MAVSDK/pymavlink
            for port in UDP_LISTEN_PORTS:
                thread = threading.Thread(target=self.listen_udp_and_forward, args=(port, ser))
                thread.daemon = True
                thread.start()
                self.udp_listen_threads.append(thread)

            # Main loop: Forward serial to UDP
            while not self.stop:
                data = ser.read(1024)  # Read MAVLink data from serial (USE FOR PIXHAWK)
                # data, _ = ser.recvfrom(1024)  # Read MAVLink data from simulator (USE FOR SITL SIMULATION ONLY)
                if data:
                    for target in UDP_TARGETS:
                        udp_send_sock.sendto(data, target)  # Forward to all UDP targets

        except Exception as e:
            print(f"Error forwarding MavLink Messages (Device likely not connected): {e}")
            time.sleep(1)
        finally:
            if ser:
                ser.close()
            udp_send_sock.close()

    def listen_udp_and_forward(self, port, ser):
        """ Listens for MAVLink messages from UDP and writes them to the serial port """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", port))
        sock.settimeout(1)  # Prevent blocking forever

        print(f"Listening for MAVLink messages on UDP port {port}...")

        while not self.stop:
            try:
                data, _ = sock.recvfrom(1024)  # Receive MAVLink message
                if data:
                    ser.write(data)  # Send to serial port (Pixhawk) (USE FOR PIXHAWK)
                    # ser.sendto(data, ("127.0.0.1", 14550))  # USE FOR SITL SIMULATION ONLY
            except socket.timeout:
                continue  # Just continue if there's no data
            except Exception as e:
                print(f"Error in UDP listener {port}) (Device likely not connected): {e}")

        sock.close()