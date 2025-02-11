Black Formatter Usage (Windows)
python -m black GCS

Gazebo Sim on Jetson Info
- go to "/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink"
- add -p to mavlink start command
    - should then be "mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -p"
- command to start sitl "make px4_sitl gz_x500"

Running MavSDK_server
- Run with command: "mavsdk_server --url udp://:14550"

Connecting to drone via MavSDK scripts:
- first line "drone = System(mavsdk_server_address='localhost', port=50051)"
- second line "await drone.connect(system_address="udp://<JETSON IP>:14550")"
- IP at Aidan's house: 192.168.50.180

GCS w/ SITL Testing
- Default starting location for SITL is: 47.397971399999996, 8.5461638
    - Use this as starting location on GCS for testing mission commands