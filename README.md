## Fast-Calib2

**Data collection**

```bash
source install/setup.bash

# Launch LiDAR driver
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# Launch camera driver (replace 7 with your camera index)
ros2 launch hik_camera_ros2_driver hik_camera_launch.py camera_index:=1

# Record bag
ros2 bag record /perception/sensors/hik_camera_1/image /livox/lidar
```

Edit `src/fast_calib2/config/qr_params.yaml` (camera intrinsics, ROI, `bag_path`, etc.), then run:

```bash
source install/setup.bash

# Single-scene calibration
ros2 launch fast_calib calib.launch.py
LD_LIBRARY_PATH=/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH ros2 launch fast_calib calib.launch.py

# Multi-scene joint calibration (collect at least three scenes first)
ros2 launch fast_calib multi_calib.launch.py
```