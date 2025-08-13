# NewLine-TurtleBot3

# TurtleBot3 ROS2 Navigation & SLAM Project

This repository contains the implementation for TurtleBot3 autonomous navigation with SLAM (Simultaneous Localization and Mapping) capabilities using ROS2.

## Prerequisites

- ROS2 (Humble/Galactic)
- TurtleBot3 packages
- Cartographer for SLAM
- Navigation2 stack
- v4l2_camera for video streaming
- Python 3

## Project Structure

```
turtlebot3_ws/
└── src/
    └── turtlebot3/
        ├── my_turtlebot3/
        │   └── my_turtlebot3/
        │       ├── detect.py
        │       ├── navV2.py
        │       └── navV3.py
        └── myturtlebot3_servor/
            └── servor.py
```

## Setup

### Environment Configuration
```bash
export TURTLEBOT3_MODEL=burger
```

### Map Files
- Main navigation map: `mapRunV4.yaml` (located in workspace)
- SLAM maps: stored in `$HOME/maps/`

## Usage Instructions

### 1. SLAM (Mapping)

**SSH Terminal (Robot):**
```bash
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

**Remote PC:**
```bash
# Terminal 1 - Launch Cartographer
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Terminal 2 - Manual Control
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 3 - Save Map (when mapping is complete)
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 2. Waypoint Creation

**SSH Terminal (Robot):**
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Remote PC:**
```bash
# Launch Navigation with existing map
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/maps/hotelmap3.yaml

# Monitor waypoints
ros2 topic echo /waypoints
```

### 3. Full Autonomous Navigation (with Object Detection)

**SSH Terminal (Robot):**
```bash
# Terminal 1 - Robot Base
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2 - Server
cd turtlebot3_ws/src/turtlebot3/myturtlebot3_servor/
python3 servor.py

# Terminal 3 - Camera
sudo chmod a+rw /dev/video0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

**Remote PC:**
```bash
# Terminal 1 - Navigation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/mapRunV4.yaml

# Terminal 2 - Object Detection
cd /home/jakgritb/turtlebot3_ws/src/turtlebot3/my_turtlebot3/my_turtlebot3/
python3 detect.py

# Terminal 3 - Navigation Controller
cd /home/jakgritb/turtlebot3_ws/src/turtlebot3/my_turtlebot3/my_turtlebot3/
python3 navV2.py
```

### 4. Autonomous Navigation (without Object Detection)

**SSH Terminal (Robot):**
```bash
# Terminal 1 - Robot Base
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2 - Server
cd turtlebot3_ws/src/turtlebot3/myturtlebot3_servor/
python3 servor.py
```

**Remote PC:**
```bash
# Terminal 1 - Navigation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/mapRunV4.yaml

# Terminal 2 - Navigation Controller
cd /home/jakgritb/turtlebot3_ws/src/turtlebot3/my_turtlebot3/my_turtlebot3/
python3 navV3.py
```

## Key Components

### Python Scripts
- **`servor.py`**: Server component running on robot
- **`detect.py`**: Object detection module
- **`navV2.py`**: Navigation controller with object detection
- **`navV3.py`**: Navigation controller without object detection

### ROS2 Topics
- `/waypoints`: Waypoint monitoring
- `/cmd_vel`: Velocity commands
- Camera topics via v4l2_camera

## Notes

- Map file `mapRunV4.yaml` was moved from `$HOME/` to workspace directory
- Camera permissions must be set before launching: `sudo chmod a+rw /dev/video0`
- Ensure proper network connection between SSH (robot) and Remote PC
- Use `teleop_keyboard` for manual control during mapping phase

## Troubleshooting

1. **Network Issues**: Verify SSH connection and ROS_DOMAIN_ID settings
2. **Camera Access**: Check camera permissions and device availability
3. **Map Loading**: Ensure map file paths are correct
4. **Navigation Failures**: Verify map quality and initial pose estimation

## Dependencies Installation

```bash
# TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*

# Cartographer
sudo apt install ros-humble-cartographer*

# Navigation2
sudo apt install ros-humble-navigation2*

# Camera
sudo apt install ros-humble-v4l2-camera
```
