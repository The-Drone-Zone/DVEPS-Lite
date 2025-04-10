from Initialize import Initialize
from Utils.msgForwarding import MsgForwarding
import subprocess
import os
import threading

if __name__ == "__main__":
    mavsdk_process = None
    mavlink_router_process = None
    # forwarding = MsgForwarding() # Manual forwarding DELETE
    # forwarding_thread = threading.Thread(target=forwarding.forwardMavLinkMsgs, daemon=True) # Manual forwarding DELETE
    # forwarding_thread.start() # Manual forwarding DELETE
    try:
        # Run mavsdk_server in the background
        if os.name == "nt":  # Windows
            print("Running on Windows")
            mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_win.exe")
            # Change 3rd argument if using udp simulation (setup for radio)
            # mavsdk_process = subprocess.Popen([mavsdk_server_path, "--url", "serial://COM3:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif os.name == "posix":  # Linux/macOS
            print("Running on Linux")
            mavsdk_server_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavsdk_server_linux.exe")
            mavlink_router_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../mavlink-routerd")
            # Change 3rd argument if using udp simulation (setup for radio)
            # mavsdk_process = subprocess.Popen([mavsdk_server_path, "--url", "serial:///dev/ttyUSB0:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            mavlink_router_process = subprocess.Popen([mavlink_router_path, "-e", "127.0.0.1:14551", "-e", "127.0.0.1:14552", "serial:///dev/ttyUSB0:57600"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        mavsdk_process = subprocess.Popen([mavsdk_server_path, "--url", "udp://:14551"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Start GCS Application
        app = Initialize()

        app.root.mainloop()
    finally:
        if mavsdk_process:
            mavsdk_process.terminate()
            mavsdk_process.wait()
        if mavlink_router_process:
            mavlink_router_process.terminate()
            mavlink_router_process.wait()
        # forwarding.stop = True # Manual forwarding DELETE