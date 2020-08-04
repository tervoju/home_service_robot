
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/new_home_service_robot/home_service_robot/src/world/juha2.world " &
sleep 5

xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/new_home_service_robot/home_service_robot/src/map/map.yaml" &
sleep 5

xterm -e " rosrun rviz rviz -d /home/workspace/new_home_service_robot/home_service_robot/src/rvizConfig/rvizConfig.rviz " &
sleep 15

xterm -e "rosrun add_markers add_markers x:=4.0 y:=1.0" &
sleep 5

xterm -e "rosrun pick_objects pick_objects"