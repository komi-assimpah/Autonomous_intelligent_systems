.PHONY: all help build clean
.PHONY: sim_house_gazebo search_nodes search_full teleop kill
.PHONY: slam map_save

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

# Launch object search nodes ONLY (requires Gazebo already running)
search_nodes:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation search_nodes.launch.py

# Launch complete search mission (Gazebo + nodes together)
search_full:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch object_search_navigation search_mission.launch.py

# Keyboard teleoperation
teleop:
	. /opt/ros/*/setup.sh && \
		ros2 run teleop_twist_keyboard teleop_twist_keyboard \
			--ros-args --remap cmd_vel:=/cmd_vel

# Autonomous exploration with FSM (alternative to teleop for SLAM)
fsm_explore:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 run turtlebot3_fsm fsm_node

# Launch Cartographer SLAM (requires Gazebo already running)
# Use teleop in another terminal to move the robot and build the map
slam:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# Save the map after SLAM (run while slam is still running)
map_save:
	. /opt/ros/*/setup.sh && \
		ros2 run nav2_map_server map_saver_cli -f maps/my_map

# Kill all ROS/Gazebo processes (run in separate terminal if needed)
kill:
	-killall -9 ign gzserver gzclient ruby 2>/dev/null
	@echo "Gazebo processes killed. Use Ctrl+C to stop ROS nodes."

clean:
	rm -rf build/ install/ log/
	rm -rf src/build/ src/install/ src/log/

help:
	@echo "Available targets:"
	@echo "  build          - Build all ROS2 packages"
	@echo "  sim_house_gazebo - Launch Gazebo with TurtleBot3"
	@echo "  search_nodes   - Launch search nodes (Gazebo must be running)"
	@echo "  search_full    - Launch Gazebo + search nodes together"
	@echo "  teleop         - Keyboard control"
	@echo "  slam           - Launch Cartographer SLAM (Gazebo must be running)"
	@echo "  map_save       - Save the map to maps/my_map"
	@echo "  kill           - Kill all ROS/Gazebo processes"
	@echo "  clean          - Remove build artifacts"