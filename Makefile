.PHONY: all help build clean
.PHONY: sim_house_gazebo search_nodes search_full teleop kill
.PHONY: slam slam_search map_save nav2 fsm_explore

# Default target
all: build

build:
	. /opt/ros/*/setup.sh && \
		colcon build \
			--base-paths src \
			--build-base build \
			--install-base install \
			--cmake-args -DBUILD_TESTING=ON


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
search_target:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation search_nodes.launch.py

keyboard_teleop:
	. /opt/ros/*/setup.sh && \
		ros2 run teleop_twist_keyboard teleop_twist_keyboard \
			--ros-args --remap cmd_vel:=/cmd_vel


# Launch Cartographer SLAM only (requires Gazebo already running)
slam:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		export TURTLEBOT3_MODEL=burger && \
		ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# TODO: SLAM + wall-following (systematic, for precise map)

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