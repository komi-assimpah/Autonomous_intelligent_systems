.PHONY: all help build clean
.PHONY: sim_house_gazebo search_nodes search_full teleop kill
.PHONY: slam slam_search map_save nav2 fsm_explore

# Default target
all: build

# Build all ROS 2 packages
build:
	. /opt/ros/*/setup.sh && \
		colcon build \
			--base-paths src \
			--build-base build \
			--install-base install \
			--cmake-args -DBUILD_TESTING=ON


# Launch TurtleBot3 in Gazebo simulation (house world)
sim_house_gazebo:
	export LIBGL_ALWAYS_SOFTWARE=1 && \
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py


# Launch TurtleBot3 with D435i camera (world environment + RViz)
sim_d435i:
	export LIBGL_ALWAYS_SOFTWARE=1 && \
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation sim_d435i.launch.py


# Launch D435i simulation WITHOUT Gazebo GUI (faster, RViz only)
sim_fast:
	export LIBGL_ALWAYS_SOFTWARE=1 && \
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation sim_d435i.launch.py headless:=true

# Launch object search nodes ONLY (requires Gazebo already running)
search_nodes:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation search_nodes.launch.py

# Keyboard teleoperation
teleop:
	. /opt/ros/*/setup.sh && \
		ros2 run teleop_twist_keyboard teleop_twist_keyboard \
			--ros-args --remap cmd_vel:=/cmd_vel


# Launch Cartographer SLAM only (requires Gazebo already running)
slam:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# SLAM + autonomous exploration (random, for quick map)
slam_explore:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch object_search_navigation slam_explore.launch.py

# SLAM + wall-following (systematic, for precise map)
slam_mapping:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch object_search_navigation slam_mapping.launch.py

# SLAM + autonomous exploration + YOLO Detection (unknown environment)
slam_search:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch object_search_navigation slam_search.launch.py

# Save the map after SLAM (run while slam is still running)
map_save:
	. /opt/ros/*/setup.sh && \
		ros2 run nav2_map_server map_saver_cli -f maps/my_map

# Kill all ROS/Gazebo processes
kill:
	-killall -9 ign gzserver gzclient ruby rviz2 2>/dev/null
	-pkill -9 -f "ros2 run object_search_navigation" 2>/dev/null
	-pkill -9 -f "ros2 run ia_package" 2>/dev/null
	-pkill -9 -f "ros2 run object_detector" 2>/dev/null
	-pkill -9 -f "parameter_bridge" 2>/dev/null
	@echo "✅ All Gazebo, RViz, and project nodes killed."

clean:
	rm -rf build/ install/ log/
	rm -rf src/build/ src/install/ src/log/

help:
	@echo "=== Object Search Robot ==="
	@echo ""
	@echo "Simulation:"
	@echo "  sim_house_gazebo - Launch Gazebo with TurtleBot3"
	@echo ""
	@echo "Mode 1 - Unknown Environment (SLAM + FSM):"
	@echo "  slam_search      - SLAM + FSM exploration + detection"
	@echo "  slam             - SLAM only (use with teleop/fsm_explore)"
	@echo "  fsm_explore      - Autonomous FSM exploration"
	@echo "  map_save         - Save map to maps/my_map"
	@echo ""
	@echo "Mode 2 - Known Environment (with map):"
	@echo "  nav2 MAP=file    - Navigation2 with pre-recorded map"
	@echo ""
	@echo "Other:"
	@echo "  search_full      - Launch Gazebo + search nodes"
	@echo "  teleop           - Keyboard control"
	@echo "  build            - Build all packages"
	@echo "  kill             - Kill Gazebo processes"