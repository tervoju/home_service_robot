
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/home_service_robot/src/world/juha2.world " &
sleep 5

xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/home_service_robot/src/map/map.yaml" &
sleep 5

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
