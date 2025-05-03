from Initialize import Initialize
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
            # Change if using udp simulation (setup for radio)
            input_port = "serial://COM3:57600"
        elif os.name == "posix":  # Linux/macOS
            print("Running on Linux")
            mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_linux.exe")
            # Change if using udp simulation (setup for radio)
            input_port = "serial:/dev/ttyUSB0"
        mavsdk_process = subprocess.Popen([mavsdk_server_path, "--url", input_port], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Start GCS Application
        app = Initialize()
        app.root.mainloop()

    # End processes on GUI termination
    finally:
        if mavsdk_process:
            mavsdk_process.terminate()
            mavsdk_process.wait()