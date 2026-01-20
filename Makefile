.PHONY: clean
.PHONY: sim_d435i sim_fast orchestrator slam_explore map_save teleop kill

sim_d435i:
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation sim_d435i.launch.py

# Usage: make orchestrator_sim ARGS="target_class:='dog'"
orchestrator_sim:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch robot_orchestrator orchestrator.launch.py sim:=true $(ARGS)

# Usage: make orchestrator_real ARGS="target_class:='cat'"
orchestrator_real:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch robot_orchestrator orchestrator.launch.py sim:=false $(ARGS)





sim_fast:
		. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation sim_d435i.launch.py headless:=true

slam_explore:
	. /opt/ros/*/setup.sh && \
		. install/setup.sh && \
		ros2 launch object_search_navigation slam_explore.launch.py

map_save:
	. /opt/ros/*/setup.sh && \
		ros2 run nav2_map_server map_saver_cli -f maps/my_map

kill:
	-killall -9 ign gzserver gzclient ruby rviz2 2>/dev/null
	-pkill -9 -f "ros2" 2>/dev/null
	@echo "All clean."

clean:
	rm -rf build/ install/ log/