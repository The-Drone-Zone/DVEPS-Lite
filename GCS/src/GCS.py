from Initialize import Initialize
import subprocess
import os

if __name__ == "__main__":
    # Run mavsdk_server in the background
    if os.name == "nt":  # Windows
        print("Running on Windows")
        # Change 3rd argument if using udp simulation (setup for radio)
        process = subprocess.Popen(["mavsdk_server_win", "--url", "serial://COM3:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    elif os.name == "posix":  # Linux/macOS
        print("Running on Linux")
        # Change 3rd argument if using udp simulation (setup for radio)
        process = subprocess.Popen(["mavsdk_server_win", "--url", "serial://COM3:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Start GCS Application
    app = Initialize()

    app.root.mainloop()
