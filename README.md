# VierBot — Autonomous SLAM & Navigation Mobile Robot

An autonomous differential-drive robot powered by **ROS 2**, **Slam Toolbox**, **Nav2**, **Raspberry Pi 5**, and **ESP32** microcontroller.

---

## 1. System Architecture

```
[ Remote PC / Workstation (RViz2) ]
         ▲
         │ (WiFi SSH)
         ▼
[ Raspberry Pi 5 (Ubuntu + ROS 2) ]
  ├── slam_toolbox (2D Online Async SLAM)
  ├── Nav2 (NavFn Planner + DWB Controller)
  ├── robot_state_publisher (URDF & TF Tree)
  ├── sllidar_ros2 (RPLidar C1 2D Scanner -> /scan)
  └── diffdrive_arduino (ros2_control Hardware Interface)
         │
         │ (USB-UART Serial @ 115200 Baud)
         ▼
[ ESP32 Microcontroller ]
  ├── Encoder Interrupts (JGA25-370 Dual Hall Effect)
  ├── Motor Control PWM (BTS7960 Driver)
  └── Closed-Loop Velocity Controller
```

---

## 2. Cleaned Repository Structure

```
Vierbot/
├── minibot/                     # Core robot package
│   ├── config/                  # SLAM Toolbox, Nav2, controller, and twist_mux configs
│   │   ├── controller.yaml             # ros2_control diff_drive_controller config
│   │   ├── controller_gz_sim.yaml      # Simulation controller config
│   │   ├── mapper_params_online_async.yaml # Real-time 2D SLAM Toolbox parameters
│   │   ├── mapper_params_localization.yaml # Localization-mode SLAM config
│   │   ├── nav2_params.yaml            # Complete Nav2 costmap & planner config
│   │   ├── twist_mux.yaml              # Priority multiplexer for teleop / nav2
│   │   ├── joystick_params.yaml        # Joy controller configuration
│   │   ├── minibot_config.rviz         # Hardware RViz2 setup
│   │   └── sim_config.rviz             # Simulation RViz2 setup
│   ├── description/             # Clean Xacro robot models (no dead cameras)
│   │   ├── robot.urdf.xacro            # Top-level robot model selector
│   │   ├── robot_main.xacro            # Base chassis, wheels & caster
│   │   ├── lidar.xacro                 # RPLidar C1 mounting & frame
│   │   ├── ros2_control.xacro          # Hardware interface definition
│   │   ├── gz_diff_drive_control.xacro # Gazebo physics plugin
│   │   └── inertial_macros.xacro       # Physical mass & inertia equations
│   ├── launch/                  # Modular launch files
│   │   ├── robot.launch.py             # Physical robot bringup (sensors + controllers)
│   │   ├── online_async_launch.py      # Real-time SLAM Toolbox node
│   │   ├── robot_remote_station.launch.py # Remote workstation RViz2 monitoring
│   │   ├── sim.launch.py               # Gazebo simulation world + robot spawn
│   │   ├── sim_control_station.launch.py  # Simulation RViz2 + navigation station
│   │   └── joystick_teleop.launch.py   # Joy teleop launcher
│   ├── maps/                    # Map occupancy grids (.yaml, .pgm)
│   ├── scripts/                 # Utility scripts (WASD teleop, TF helpers)
│   │   └── teleop_wasd.py
│   └── worlds/                  # Gazebo simulation worlds (playground.sdf, empty.sdf)
├── diffdrive_arduino/           # C++ ros2_control Hardware Interface plugin
│   └── hardware/                       # DiffBotSystemHardware & ArduinoComms drivers
├── ESP32MotorCytronSerial/      # Low-level ESP32 firmware
│   └── ros_arduino_bridge/             # Arduino sketch with encoder ISRs & PWM loops
├── sllidar_ros2/                # Slamtec RPLidar C1 / A-series driver
├── serial/                      # Cross-platform POSIX serial dependency
└── twist_stamper/               # Twist <-> TwistStamped message converter
```

---

## 3. Hardware Configuration & Wiring Pinout

### Physical Specifications
- **Dimensions**: $26.5\text{ cm } (L) \times 27.5\text{ cm } (W) \times 20.0\text{ cm } (H)$
- **Wheel Diameter**: $66\text{ mm}$ (Radius: $33\text{ mm}$)
- **Wheel Track (Separation)**: $220\text{ mm}$
- **Motors**: JGA25-370 12V DC Gear Motors with Hall-Effect Encoders (1920 ticks/revolution after gearbox).

### ESP32 Pin Connections
| Component | Pin Function | ESP32 GPIO |
| :--- | :--- | :--- |
| **Left Encoder** | Channel A | GPIO 18 (Interrupt) |
| **Left Encoder** | Channel B | GPIO 19 (Interrupt) |
| **Right Encoder** | Channel A | GPIO 22 (Interrupt) |
| **Right Encoder** | Channel B | GPIO 23 (Interrupt) |
| **Left Motor Driver (BTS7960)** | Forward PWM (RPWM/LPWM) | GPIO 25 |
| **Left Motor Driver (BTS7960)** | Reverse PWM (RPWM/LPWM) | GPIO 26 |
| **Right Motor Driver (BTS7960)**| Forward PWM (RPWM/LPWM) | GPIO 32 |
| **Right Motor Driver (BTS7960)**| Reverse PWM (RPWM/LPWM) | GPIO 33 |
| **USB Communication** | UART0 (RX / TX) | Default USB Serial (115200 Baud) |

---

## 4. Build & Setup Guide

### Prerequisites
Install ROS 2 (Humble / Iron / Jazzy), `ros2_control`, `slam_toolbox`, and `nav2`:
```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-twist-mux \
  ros-$ROS_DISTRO-teleop-twist-joy
```

### Build Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Set up RPLidar USB Permissions
```bash
cd ~/ros2_ws/src/Vierbot/sllidar_ros2/scripts
sudo bash create_udev_rules.sh
```

---

## 5. Running the Robot

### Option A: Physical SLAM (Real Robot)

1. **Upload ESP32 Firmware**:
   - Open `ESP32MotorCytronSerial/ros_arduino_bridge/ros_arduino_bridge.ino` in Arduino IDE / PlatformIO and flash it to the input ESP32.

2. **On the Raspberry Pi 5 (Robot)**:
   ```bash
   # Terminal 1: Bringup Robot Hardware (Controllers, State Publisher, LiDAR, Twist Mux)
   ros2 launch minibot robot.launch.py

   # Terminal 2: Start 2D SLAM Mapping
   ros2 launch minibot online_async_launch.py
   ```

3. **On the Remote PC (Workstation on same WiFi / ROS_DOMAIN_ID)**:
   ```bash
   # Terminal 1: Launch RViz2 Remote Control Station
   ros2 launch minibot robot_remote_station.launch.py

   # Terminal 2: Drive the robot to build the map
   ros2 run minibot teleop_wasd.py
   ```

4. **Save Map**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/minibot/maps/my_new_map
   ```

---

### Option B: Gazebo Simulation SLAM

1. **Launch Simulation**:
   ```bash
   # Terminal 1: Launch Gazebo World & Spawn Robot
   ros2 launch minibot sim.launch.py
   ```

2. **Launch SLAM & Simulation Station**:
   ```bash
   # Terminal 2: Start SLAM in simulation
   ros2 launch minibot online_async_launch.py use_sim_time:=true

   # Terminal 3: Launch RViz2 Sim Monitor & Teleop
   ros2 launch minibot sim_control_station.launch.py
   ```
