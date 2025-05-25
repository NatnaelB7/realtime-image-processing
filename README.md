# Realtime Image Processing (ROS 2)

This project demonstrates a ROS 2-based image processing pipeline using two custom packages: one written in Python and one in C++. The system captures webcam images, overlays a timestamp, and processes them to detect and recolor red regions in real-time.



## Packages

### 1. `orange` (Python)
**Executable:** `webcam_driver`

- Captures live webcam feed using OpenCV.
- Overlays the current system time (in seconds) on each frame.
- Publishes images to the topic `/webcam/image_raw`.

### 2. `red` (C++)
**Executable:** `color_filter`

- Subscribes to `/webcam/image_raw`.
- Detects **red** regions in the image.
- Replaces the red regions with **blue**.
- Displays:
  - Original image
  - Red regions (in red color)
  - Modified image with red replaced by blue



## Installation & Build

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-url>
   ```


2. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
   

## Running the System

Use the provided launch file to start both nodes:

```bash
ros2 launch orange webcam_color_filter.launch.py
```

To directly view the published webcam stream:

```bash
ros2 run image_view image_view --ros-args -r image:=/webcam/image_raw
```


## Dependencies

* ROS 2 (tested with Humble)
* OpenCV
* `cv_bridge`
* Python 3
* C++


## Notes

* The Python package name is based on a favorite fruit (`orange`).
* The C++ package name is based on a favorite color (`red`).
* Ensure your camera is available and not used by other applications.


## Author

Natnael Takele
[nbtakele@aggies.ncat.edu](mailto:nbtakele@aggies.ncat.edu)

