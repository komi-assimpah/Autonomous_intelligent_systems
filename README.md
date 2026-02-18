# Autonomous Object Search System

## Project Overview
This project involves developing a mobile robot capable of exploring an environment to find a specific object (e.g., dog, cat, bottle, etc.) completely autonomously.

### The Concept
The robot combines **Autonomous Exploration** and **Artificial Intelligence**.
1.  It **explores** the room while avoiding walls.
2.  It visually **detects** its target using a camera.
3.  It **stops**, localizes the object on the map, and reports the object's coordinates in the map frame.

---

## System Architecture
The system relies on 3 ROS 2 Packages:

- **`ia_package`**: Object detection (YOLOv11) and 3D positioning in different frames (CAMERA and MAP).
- **`robot_orchestrator`**: State machine that controls the global mission.
- **`object_search_navigation`**: Exploration and obstacle avoidance algorithm.

### Mission Sequence
The diagram below illustrates the complete mission flow, from initialization to shutdown, and the interactions between the 3 packages and the environment:

![Mission Sequence](docs/architecture.png)

---

## Installation & Quick Start

> [!IMPORTANT]
> **Disk Space Required**: 
> - **Minimum 15 GB** for CPU-only installation (recommended for VMs/laptops without GPU)
> - **Minimum 20 GB** for GPU installation with CUDA support
> 
> Breakdown (CPU-only):
> - ROS 2 Humble (~2 GB)
> - Python dependencies - CPU version (~2 GB)
> - Build artifacts (~2 GB)
> - System updates and cache (~2 GB)

### Prerequisites

### 1. Prepare a Virtual Machine (If necessary)
- On Windows: VirtualBox (https://www.virtualbox.org)
- On MacOSX: UTM (https://mac.getutm.app)

### 2. Install Ubuntu 22.04 (Jammy Jellyfish)
- Download: https://releases.ubuntu.com/jammy/

! ARM image is required for Apple Silicon-based laptops,
- https://cdimage.ubuntu.com/releases/22.04/release/
- Only the server version is available… need to manually install GUI…

```bash
sudo apt update && sudo apt upgrade && sudo apt install ubuntu-desktop
```
Then reboot your computer.

### 3. Install ROS 2 Humble

#### 3.1 Add ROS 2 Repository
```bash
# Add ROS 2 GPG key and repository
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 3.2 Install ROS 2 Humble Desktop and Required Packages
```bash
# Update package list
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble Desktop (base distribution)
sudo apt install ros-humble-desktop -y

# Install project-specific packages
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-nav2-map-server \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-msgs \
  ros-humble-xacro
```

#### 3.3 Install Build Tools
Install **Colcon** (the standard ROS 2 build tool):

```bash
sudo apt install python3-colcon-common-extensions python3-argcomplete libboost-system-dev
```

Finally, install **Cyclone DDS**:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### Shell Configuration (bashrc)
To make ROS 2 accessible in every new terminal and to use CycloneDDS for robot communication, execute this once:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

source .bashrc

# Verify variables are loaded
printenv | grep -i ROS
```

Verify if any system dependencies are missing and install them:

```bash
sudo apt install build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

Verify that the ROS 2 installation works:

```bash
ros2
# usage: ros2 [-h] ...
# ros2 is an extensible command-line tool for ROS 2.
```

### 4. Install Gazebo

This project uses **Gazebo Ignition Fortress** compatible with ROS2 Humble.
```bash
sudo apt install ros-humble-ros-gz

# Verify installation
ign gazebo --version
# Should display: Gazebo Sim, version 6.x.x
```

> **Note**: Do not confuse with `gazebo` (Gazebo Classic) or `gz` (Gazebo Garden+).
> This project uses `ign gazebo` (Ignition Fortress).

Official Documentation: https://gazebosim.org/docs/fortress/install_ubuntu

---

## Simulation Installation (Gazebo)

### 1. Clone the Project
```bash
git clone https://github.com/komi-assimpah/Autonomous_intelligent_systems.git

cd Autonomous_intelligent_systems
```

### 2. Install Dependencies

```bash
# 1. Build tools and Python package management
sudo apt install python3-vcstool python3-colcon-common-extensions python3-pip -y

# 2. Fetch sub-repositories (TurtleBot3, Dynamixel...)
vcs import < dependencies.repos

# 3. Install ROS dependencies
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# 4. Install Python libs (YOLO, PyTorch, etc.)
# IMPORTANT: This step is REQUIRED and installs AI/ML dependencies
# Choose ONE of the following options:

# Option A: CPU-only (Recommended for VMs/laptops without GPU, saves ~3-4 GB)
pip3 install --user -r requirements-cpu.txt

# Option B: GPU with CUDA (Only if you have an NVIDIA GPU)
pip3 install --user -r requirements.txt
```

### 3. Build
```bash
colcon build
# ... (this may take a while) ...

source install/setup.bash
```

### 4. Configure Environment

In every new terminal, run:
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Or add these lines to your `~/.bashrc` for automatic setup:
```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/Autonomous_intelligent_systems/install/setup.bash' >> ~/.bashrc
```

### 5. Launch Simulation
Launch Gazebo, RViz, and all intelligence with the command:
```bash
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true
```

By default, it searches for a dog. To specify a target object, ensure the previous simulation is completely stopped, then launch:

```bash
# To search for a specific object
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='target_class_name'

# Examples:
# To search for a dog
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='dog'

# To search for a cat
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true target_class:='cat'
```

If the object is not supported by the search model, an error will appear in the terminal listing all supported classes.

---

## Real Robot Deployment (TurtleBot3 Burger + RealSense D435i)

> [!IMPORTANT]
> **Use the `demo-real` branch**, the `main` branch targets Gazebo simulation.
> The real-hardware code has been specifically adapted and validated on this branch.
> ```bash
> git checkout demo-real
> ```

> [!NOTE]
> The architecture is **split** for performance: the **Raspberry Pi** (on the robot) handles hardware drivers only, while the **PC** runs all intelligence (navigation, detection, mapping).
> 
> The `realsense2_camera` ROS package is **not required** on the PC — the robot streams a compressed RGB-D feed over WiFi via `mqtt_rgb_bridge`, which is decoded on the PC side.

### Architecture Overview

```
┌─────────────────────────────────┐     WiFi (ROS 2 DDS)       ┌──────────────────────────────────────┐
│         ROBOT (RPi 4)           │ ◄───────────────────────►  │              PC                      │
│                                 │                            │                                      │
│  • TurtleBot3 drivers (motors)  │  /scan, /odom, /tf         │  • Cartographer (SLAM)               │
│  • LiDAR (LDS-02)               │  /camera/rgbd/compressed ► │  • Image Decompressor                │
│  • RealSense D435i (6 FPS)      │                            │  • YOLO Inference (object detection) │
│  • Static TF (camera_link)      │  ◄── /cmd_vel              │  • Navigation Node (exploration)     │
│                                 │                            │  • FSM Orchestrator (state machine)  │
└─────────────────────────────────┘                            │  • RViz2 (visualization)             │
                                                               └──────────────────────────────────────┘
```

### Prerequisites

#### On the PC
Same as the simulation setup (ROS 2 Humble, Python deps, colcon build). Additionally set:

```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
```

Add to `~/.bashrc` for persistence:
```bash
echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
```

#### On the Robot (TurtleBot3)
The robot must have the `robot_orchestrator` and `mqtt_rgb_bridge` packages built in `~/groupe7_ws`:

```bash
# On the robot
cd ~/groupe7_ws
colcon build --symlink-install
source install/setup.bash
```

Ensure the robot's `~/.bashrc` contains:
```bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
```

### Hardware Setup
- Connect the **RealSense D435i** to the Raspberry Pi via **USB 3.0** port.
- Verify detection: `lsusb | grep Intel` should show `Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i`.
- Ensure the robot and PC are on the **same WiFi network**.

### Launch Procedure

#### Step 1 — On the Robot
```bash
# SSH into the robot
ssh turtlebot1@<ROBOT_IP>

# Launch hardware drivers only (lightweight)
ros2 launch robot_orchestrator robot.launch.py
```

Expected output (no errors):
```
[turtlebot3_ros]: Run!
[realsense_rgbd_publisher]: ✅ RealSense RGB-D Packed Publisher started: /camera/rgbd/compressed (424x240 @ 6fps)
[camera_tf_broadcaster]: Spinning until stopped - publishing transform
```

#### Step 2 — On the PC
```bash
colcon build && source install/setup.bash

# Launch all intelligence (SLAM + Detection + Navigation + RViz)
ros2 launch robot_orchestrator pc.launch.py target_class:=<TARGET>
```

Replace `<TARGET>` with any COCO class name. Examples:
```bash
ros2 launch robot_orchestrator pc.launch.py target_class:=chair
ros2 launch robot_orchestrator pc.launch.py target_class:=person
ros2 launch robot_orchestrator pc.launch.py target_class:=dog
```

The robot will autonomously explore the environment, detect the target, and stop when found, displaying the target in the rviz with a marker.

### Troubleshooting

#### RealSense fails to start (`Couldn't resolve requests`)
The camera only supports specific framerates: **6, 15, 30, 60 FPS**. Do not use values like 4 or 5.

#### Robot doesn't move
- Confirm `turtlebot3_ros` is running without errors on the robot.
- Confirm the PC can see robot topics: `ros2 topic list | grep scan`
- Check `ROS_DOMAIN_ID` matches on both machines.

---

## Troubleshooting

### Gazebo Flickering or Black Screen (Wayland/VM)
If Gazebo window is flickering, black, or not displaying the scene properly on systems using Wayland or in VMs (especially VMware):

> [!IMPORTANT]
> This issue commonly occurs when running Ubuntu in VMware

**Solution:**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export MESA_GL_VERSION_OVERRIDE=3.3

# Then launch normally
ros2 launch robot_orchestrator orchestrator.launch.py sim:=true
```

- `LIBGL_ALWAYS_SOFTWARE=1` forces CPU-based rendering (avoids GPU driver issues)
- `QT_QPA_PLATFORM=xcb` forces X11 instead of Wayland
- `MESA_GL_VERSION_OVERRIDE=3.3` ensures OpenGL 3.3 compatibility

> [!NOTE]
> Software rendering is slower but much more stable in VMs and Wayland environments.
> The simulation will run correctly, just with lower FPS in the Gazebo GUI.