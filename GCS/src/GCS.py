from Initialize import Initialize
import subprocess
import os

if __name__ == "__main__":
    # Run mavsdk_server in the background
    if os.name == "nt":  # Windows
        print("Running on Windows")
        mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_win.exe")
        # Change 3rd argument if using udp simulation (setup for radio)
        process = subprocess.Popen([mavsdk_server_path, "--url", "serial://COM3:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    elif os.name == "posix":  # Linux/macOS
        print("Running on Linux")
        mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_linux.exe")
        # Change 3rd argument if using udp simulation (setup for radio)
        process = subprocess.Popen([mavsdk_server_path, "--url", "serial:///dev/ttyUSB0:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Start GCS Application
    app = Initialize()

    app.root.mainloop()