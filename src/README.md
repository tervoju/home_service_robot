# Home Service Robot

turtlebot robot simulation where:

- initially show marker to be picked up from pickup zone
- hide the marker once robot reach the pickup zone and wait 5 sec to simulate a pickup
- shows the marker at the drop off zone once robot reaches it

# node or modules

## add_marker
ROS node to show markers (either on pickup or dropoff) based on estimated position.

## map

new simple building, world and map created for this home service robot

## pick_object 
ROS node to move turtlebot based on target pickup and dropoff zones.

## scripts
all scripts to test and develop home service robot


# ROS libraries 

wiki.ros.org/gmapping

wiki.ros.org/turtlebot_teleop

wiki.ros.org/turtlebot_rviz_launchers

wiki.ros.org/turtlebot_gazebo


## House

originally used world as in the earlier assignments ( based on my country house) - if it works in this case. seems to be that doors might be problematic
retried map creation with https://github.com/udacity/pgm_map_creator
in the map there is no doors. decided to do simple house without doors, similar as  in examples of this assignment.

# Environment
virtual environment 
Distributor ID:	Ubuntu
Description:	Ubuntu 16.04.6 LTS
Release:	16.04
Codename:	xenial

# ROS
[kinetic](wiki.ros.org/kinetic)

## virtual env. Issues

### issue 1
turtlebot description caused error

while processing /home/workspace/home_service_robot/src/turtlebot_simulator/turtlebot_gazebo/launch/includes/kobuki.launch.xml:
Invalid <param> tag: Cannot load command parameter [robot_description]: command [/opt/ros/kinetic/lib/xacro/xacro 

pip install rospkg 

seem to fix the issue in the virtual environment. 

# orientation issue
generated map was flipped 90 degree 