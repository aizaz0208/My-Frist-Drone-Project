# drone_ws

Clean ROS 2 Jazzy workspace for the PX4 v1.17 + Gazebo Harmonic X500 project.

## Workspace structure

```text
drone_ws/
├── src/
│   └── my_first_drone/
│       ├── launch/
│       │   └── drone.launch.py
│       ├── my_first_drone/
│       │   ├── __init__.py
│       │   └── my_first_drone.py
│       ├── resource/
│       │   └── my_first_drone
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── README.md
```

## What the one Python file does

`my_first_drone.py` contains both:
- keyboard control
- ROS 2 command publishing
- MAVLink connection to PX4 on UDP 14540
- arm/disarm
- AUTO TAKEOFF
- land
- RTL
- horizontal movement
- vertical movement
- yaw
- continuous Offboard velocity setpoints

## Launch

After placing this workspace at `~/drone_ws` and building it:

```bash
cd ~/drone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/drone_ws/install/setup.bash
ros2 launch my_first_drone drone.launch.py
```

The launch file starts:
- PX4 v1.17 + Gazebo X500 using `~/PX4-Gazebo`
- QGroundControl from `~/Downloads/QGroundControl-x86_64.AppImage`
- the combined keyboard/MAVLink controller in a new terminal

## Keyboard

- SPACE = Arm / Disarm
- T = Takeoff
- X = Emergency Land
- L = Land
- R = Return to Launch
- W / S = Up / Down
- Arrow keys = Forward / Backward / Left / Right
- A / D = Yaw left / right

## Notes

This workspace expects:
- Ubuntu 24.04
- ROS 2 Jazzy
- PX4 v1.17 with Gazebo Harmonic
- Gazebo X500 already working from `~/PX4-Gazebo`
- QGroundControl AppImage at the path above
- `pymavlink` available to Python 3
