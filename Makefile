.PHONY: clean
.PHONY: sim_d435i sim_fast orchestrator slam_explore map_save teleop kill

# Main Simulation (Gazebo + Nav2 + RViz)
sim_d435i:
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation sim_d435i.launch.py

# Simulation Headless (No Gazebo GUI, only RViz - faster)
sim_fast:
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation sim_d435i.launch.py headless:=true

# Orchestrator (Intelligence: FSM, Inference, Nav, Vision)
# Usage:
#   make orchestrator                              -> Default (Dog)
# ros2 launch robot_orchestrator orchestrator.launch.py target_class:='cat'
#   make orchestrator ARGS="target_class:='cat' "   -> Search for a Cat
orchestrator:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch robot_orchestrator orchestrator.launch.py $(ARGS)

slam_explore:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation slam_explore.launch.py

# Save the map after SLAM (run while slam is still running)
map_save:
	. /opt/ros/*/setup.sh && \
		ros2 run nav2_map_server map_saver_cli -f maps/my_map

teleop:
	. /opt/ros/*/setup.sh && \
		ros2 run teleop_twist_keyboard teleop_twist_keyboard \
			--ros-args --remap cmd_vel:=/cmd_vel

kill:
	-killall -9 ign gzserver gzclient ruby rviz2 2>/dev/null
	-pkill -9 -f "ros2" 2>/dev/null
	@echo "All clean."

clean:
	rm -rf build/ install/ log/