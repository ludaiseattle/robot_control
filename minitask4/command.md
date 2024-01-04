roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun map_server map_saver -f /home/ubuntu/catkin_ws/src/ars-2022-g31/minitask4/maps/map

execute this command under the folder minitask4:
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=`pwd`/maps/map.yaml

