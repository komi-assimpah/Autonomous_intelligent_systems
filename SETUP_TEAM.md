# Setup Instructions for Team

## Prerequisites
- ROS2 Humble
- Python 3.10+
- Git

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/komi-assimpah/Autonomous_intelligent_systems.git
cd Autonomous_intelligent_systems
```

### 2. Install vcstool (if not already installed)
```bash
sudo apt install python3-vcstool
```

### 3. Import external dependencies
```bash
vcs import src < dependencies.repos
```

### 4. Install ROS dependencies
```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Install Python dependencies
```bash
pip install 'numpy<2'  # For cv_bridge compatibility
```

### 6. Build the workspace
```bash
colcon build
source install/setup.bash
```

## Run the mission

### Launch complete search mission
```bash
ros2 launch object_search_navigation search_mission.launch.py
```

### Manual control (in separate terminal)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Visualize in RViz
```bash
rviz2 -d $(ros2 pkg prefix object_search_navigation)/share/object_search_navigation/rviz/camera_view.rviz
```

## Packages Overview

- `object_search_navigation`: Navigation + camera preprocessing
- `object_detector`: Object detection (mock for now, AI team will integrate YOLO)
- External: DynamixelSDK, TurtleBot3 packages

## Development

### AI Team: Integrate YOLO
Edit `src/object_detector/object_detector/simple_detector_node.py`
Replace the `detect_object()` method with your YOLO model.

### Navigation Team: Integrate Nav2
Edit `src/object_search_navigation/object_search_navigation/navigation_node.py`
Add Nav2 autonomous exploration logic.