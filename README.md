# 🤖 Beebot - Autonomous Mobile Robot System

<div align="center">

[![MIT License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS Version](https://img.shields.io/badge/ROS-Melodic-orange.svg)](http://wiki.ros.org/melodic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-18.04-purple.svg)](https://releases.ubuntu.com/18.04/)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

*A comprehensive autonomous mobile robot (AMR) system built with ROS for navigation, SLAM, and intelligent control*

</div>

## 📖 Overview

Beebot is a sophisticated autonomous mobile robot system designed for indoor navigation and autonomous operations. The project integrates advanced robotics algorithms including SLAM (Simultaneous Localization and Mapping), path planning, obstacle avoidance, and intelligent control systems. Built on the Robot Operating System (ROS) framework, Beebot provides a complete solution for autonomous mobile robotics applications.

### 🎯 Key Features

- **Advanced Localization**: Monte Carlo Localization (MCL) with expansion resetting and advanced likelihood estimation
- **Intelligent Path Planning**: A* global planning with Dynamic Window Approach (DWA) local planning
- **Real-time Mapping**: Ray casting-based local mapping with 2D LiDAR integration
- **Multi-sensor Fusion**: IMU, LiDAR, and odometry sensor integration
- **Remote Control**: Xbox gamepad support with manual/automatic mode switching
- **MQTT Communication**: Wireless control and monitoring capabilities
- **Hardware Integration**: Arduino-based hardware control with LCD feedback
- **Robust Navigation**: Obstacle avoidance and collision detection

## 🏗️ System Architecture

The system is organized into four main components:

### 📡 **SLAM & Localization** (`/slam`)
- **Advanced Localization System (ALS)**: Robust MCL with reliability estimation
- **Expanded MCL (EMCL)**: MCL with expansion resetting capabilities
- **Ray Casting Mapping**: Real-time local map generation
- **ICP Matching**: Point cloud alignment and registration
- **Sensor Fusion**: Multi-sensor data integration

### 🎯 **Motion Planning & Control** (`/motion`)
- **A* Global Planner**: Optimal path planning with configurable heuristics
- **DWA Local Planner**: Dynamic window approach for local obstacle avoidance
- **Waypoint Management**: Automated waypoint navigation system
- **Navigation Utils**: Common utilities for AMR navigation

### 🎮 **Control Systems** (`/control`)
- **Arduino Integration**: Hardware control and feedback
- **Remote Control**: Xbox gamepad teleoperation
- **Robot Eyes**: Visual feedback system

### ⚙️ **System Integration** (`/system`)
- **Launch Configuration**: System-wide launch files and configurations
- **RViz Visualization**: Real-time system monitoring and debugging

## 🚀 Quick Start

### Prerequisites

- Ubuntu 18.04 LTS
- ROS Melodic Morenia
- Python 2.7 / 3.6+
- Required ROS packages (see [Installation](#installation))

### Installation

1. **Clone the repository**
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/phatcvo/Beebot.git
   cd ..
   catkin_make
   source devel/setup.bash
   ```

2. **Install dependencies**
   ```bash
   # Install ROS packages
   sudo apt-get update
   sudo apt-get install ros-melodic-map-server ros-melodic-amcl ros-melodic-move-base
   sudo apt-get install ros-melodic-serial ros-melodic-rosserial ros-melodic-joy
   
   # Install additional dependencies
   sudo apt-get install ros-melodic-pcl-ros ros-melodic-tf2-eigen
   ```

3. **Configure hardware permissions**
   ```bash
   sudo chmod 666 /dev/ttyACM0  # Arduino
   sudo chmod a+rw /dev/input/js0  # Gamepad
   ```

### Basic Usage

1. **Start the complete system**
   ```bash
   export TURTLEBOT3_MODEL=burger
   roslaunch system demo.launch
   ```

2. **Launch individual components**
   ```bash
   # SLAM and localization
   roslaunch als_ros mcl.launch
   
   # Motion planning
   roslaunch a_star_ros a_star.launch
   roslaunch dwa_planner local_planner.launch
   
   # Hardware control
   roslaunch arduino_serial arduino_serial.launch
   roslaunch remote_control remote_control.launch
   ```

## 🧩 Core Components

### Navigation Stack

| Component | Algorithm | Description |
|-----------|-----------|-------------|
| **Global Planner** | A* Search | Optimal path planning on occupancy grid |
| **Local Planner** | Dynamic Window Approach (DWA) | Real-time obstacle avoidance |
| **Localization** | Monte Carlo Localization (MCL) | Robust pose estimation with reliability |
| **Mapping** | Ray Casting Update | Real-time local map generation |

### Sensor Integration

- **2D LiDAR**: Primary perception sensor for mapping and localization
- **IMU**: Inertial measurement for odometry and pose estimation  
- **Encoders**: Wheel odometry for motion estimation
- **Camera**: Visual feedback and monitoring

### Control Interfaces

- **Xbox Gamepad**: Manual teleoperation with speed control
- **MQTT Protocol**: Wireless command and monitoring
- **Arduino Interface**: Hardware control and status feedback
- **RViz**: Real-time visualization and debugging

## 📊 Performance Features

### Advanced Localization (ALS)
- Sensor measurement class estimation for robust localization
- Reliability estimation using Bayesian filtering
- Misalignment recognition with Markov Random Fields
- Global localization with free-space features
- Quick re-localization via importance sampling

### Motion Planning
- Multi-layered planning architecture (global + local)
- Real-time obstacle avoidance with DWA
- Configurable cost functions and constraints
- Waypoint management and goal handling

### Hardware Integration
- Real-time Arduino communication via serial interface
- System status feedback through LCD display
- Hardware emergency stop functionality
- Multi-device support (keypad, ultrasonic sensors)

## 🔧 Configuration

### System Parameters

Key configuration files are located in:
- `/system/launch/` - Launch configurations
- `/*/config/` - Algorithm parameters
- `/*/maps/` - Map files and configurations

### Hardware Setup

1. **Arduino Connection**
   - Keypad controller: `/dev/ttyACM0`
   - Ultrasonic sensors: `/dev/ttyUSB0`

2. **Sensor Calibration**
   - LiDAR frame: `laser`
   - Base frame: `base_link`
   - IMU frame: Configure in launch files

### Network Configuration

For MQTT communication setup, see the [MQTT Configuration Guide](#mqtt-communication-setup) below.

## 🌐 MQTT Communication Setup

### Raspberry Pi Broker Setup

1. **Install Mosquitto**
   ```bash
   sudo apt install mosquitto mosquitto-clients
   sudo systemctl enable mosquitto
   sudo systemctl start mosquitto
   ```

2. **Configure Authentication**
   ```bash
   sudo nano /etc/mosquitto/mosquitto.conf
   ```
   Add:
   ```
   listener 1883
   allow_anonymous false
   password_file /etc/mosquitto/pwfile
   ```

3. **Set Password**
   ```bash
   sudo mosquitto_passwd -c /etc/mosquitto/pwfile <username>
   sudo systemctl restart mosquitto
   ```

### Client Setup

```bash
sudo apt install mosquitto-clients libmosquitto-dev

# Test connection
mosquitto_pub -h <broker_ip> -t "beebot/cmd" -m "status" -u <username> -P <password>
```

## 🎮 Control Interface

### Xbox Gamepad Controls

| Button/Axis | Function |
|-------------|----------|
| **Left Stick Y** | Forward/Backward movement |
| **Right Stick X** | Left/Right rotation |
| **A Button** | Joystick speed control mode |
| **X Button** | Cross-button speed control mode |
| **B Button** | Emergency stop |
| **Y Button** | Cancel emergency stop |
| **LB/RB** | Linear speed adjustment |
| **LT/RT** | Angular speed adjustment |
| **Back** | Auto mode |
| **Start** | Manual mode / Start controller |

### Arduino Interface

The Arduino interface provides:
- System status display on LCD
- Hardware button inputs (system, go, door)
- Real-time feedback on robot state
- Emergency stop functionality

## 🛠️ Development

### Building from Source

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Running Tests

```bash
# Test individual components
roslaunch a_star_ros test.launch
roslaunch dwa_planner demo.launch
roslaunch waypoint_manager_ros test.launch
```

### Debugging

Enable debug mode for detailed visualization:
```bash
roslaunch a_star_ros test.launch debug_mode:=true
roslaunch dwa_planner demo.launch
```

## 📝 API Documentation

Detailed API documentation for the DWA planner is available at: [https://amslabtech.github.io/dwa_planner/](https://amslabtech.github.io/dwa_planner/)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Author

**Phat C. Vo** - *Project Developer and Maintainer*
- GitHub: [@phatcvo](https://github.com/phatcvo)
- Email: phatcvo@todo.todo

## 🙏 Acknowledgments

This project builds upon several open-source robotics libraries and research:

- **Advanced Localization System (ALS)** by Naoki Akai
- **ROS Navigation Stack** by the ROS community
- **PCL (Point Cloud Library)** for 3D perception
- **OpenCV** for computer vision capabilities

### Research Citations

If you use this work in your research, please consider citing:

```bibtex
@misc{beebot2025,
  title={Beebot: Autonomous Mobile Robot System},
  author={Phat C. Vo},
  year={2025},
  publisher={GitHub},
  url={https://github.com/phatcvo/Beebot}
}
```

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

</div>
