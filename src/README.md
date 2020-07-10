# Home Service Robot



# Environment
Distributor ID:	Ubuntu

Description:	Ubuntu 16.04.6 LTS

Release:	16.04

Codename:	xenial

# ROS
[kinetic](wiki.ros.org/kinetic)


# ROS libraries

wiki.ros.org/gmapping

wiki.ros.org/turtlebot_teleop

wiki.ros.org/turtlebot_rviz_launchers

wiki.ros.org/turtlebot_gazebo

# House

juhas.world my country house - if it works. seems to be that doors might be problematic
retried map creation with https://github.com/udacity/pgm_map_creator
in the map there is no doors. decided to do simple house without doors.

# issues

turtlebot description caused error

while processing /home/workspace/home_service_robot/src/turtlebot_simulator/turtlebot_gazebo/launch/includes/kobuki.launch.xml:
Invalid <param> tag: Cannot load command parameter [robot_description]: command [/opt/ros/kinetic/lib/xacro/xacro 

'''
pip install rospkg 
'''

seem to fix the issue