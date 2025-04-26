from Initialize import Initialize
from Utils.msgForwarding import MsgForwarding
import subprocess
import os
import serial
import time

def check_serial_port(port):
    try:
        # Try to open the serial port
        with serial.Serial(port):
            return True
    except serial.SerialException:
        return False

if __name__ == "__main__":
    mavsdk_process = None
    router_process = None
    input_port = None
    try:
        # Run mavsdk_server in the background
        if os.name == "nt":  # Windows
            print("Running on Windows")
            mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_win.exe")
            router_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavp2p_win.exe")
            # Change if using udp simulation (setup for radio)
            input_port = "serial:COM3"
        elif os.name == "posix":  # Linux/macOS
            print("Running on Linux")
            mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_linux.exe")
            router_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavp2p_linux")
            # Change if using udp simulation (setup for radio)
            input_port = "serial:/dev/ttyUSB0"
        while not router_process:
            if check_serial_port(input_port.split(':')[1]):
                router_process = subprocess.Popen([router_path, f"{input_port}:57600", "udpc:127.0.0.1:14551", "udpc:127.0.0.1:14552"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"Started router with {input_port}")
            else:
                print(f"Port {input_port} not found. Retrying...")
                time.sleep(1)  # Wait before retrying
        mavsdk_process = subprocess.Popen([mavsdk_server_path, "--url", "udp://:14551"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Start GCS Application
        app = Initialize()
        app.root.mainloop()

    # End processes on GUI termination
    finally:
        if mavsdk_process:
            mavsdk_process.terminate()
            mavsdk_process.wait()
        if router_process:
            router_process.terminate()
            router_process.wait()