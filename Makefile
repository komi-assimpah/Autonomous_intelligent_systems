.PHONY: all help build clean
.PHONY: sim_gazebo sim_gazebo_headless sim_cartographer sim_navigation teleop teleop_joy
.PHONY: rviz robot

# Default target
all: build

# Install dependencies and initialize submodules
# install:
# 	git submodule update --init --recursive

# Build all ROS 2 packages
build:
	. /opt/ros/*/setup.sh && \
		colcon build \
			--base-paths src \
			--build-base build \
			--install-base install \
			--cmake-args -DBUILD_TESTING=ON

# Launch TurtleBot3 in Gazebo simulation
sim_gazebo: build
	export LIBGL_ALWAYS_SOFTWARE=1 && \
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch cartographer for SLAM
sim_cartographer: build
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch turtlebot3_cartographer cartographer.launch.py

# Launch navigation stack
sim_navigation: build
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch turtlebot3_navigation2 navigation2.launch.py

# Keyboard teleoperation
teleop:
	. /opt/ros/*/setup.sh && \
		ros2 run teleop_twist_keyboard teleop_twist_keyboard \
			--ros-args --remap cmd_vel:=/cmd_vel

# Joystick teleoperation
teleop_joy:
	. /opt/ros/*/setup.sh && \
		ros2 launch teleop_twist_joy teleop-launch.py \
			config_filepath:="$(CURDIR)/utils/xone.config.yaml"

# Clean build artifacts
clean:
	rm -rf build/ install/ log/