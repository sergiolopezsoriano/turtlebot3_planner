# turtlebot3_planner

This package provides a simple ROS node that allows the user to schedule navigation waypoints from configuration files, namely a file with the names of the waypoints in a trajectory and a file containing the waypoints poses. The executed node saves the amcl robot poses in a database stored in the output folder. The names of the config and output files are configured in the waypoint_planner.launch file.

Launch instructions:

1. start the roscore.
2. start the robot actuators and sensors.

$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

3. Launch the nodes required for navigation.

$ roslaunch "path to package"/launch/turtlebot3_navigation.launch map_file:="path to workspace"/src/turtlebot3/turtlebot3_navigation/maps/"map_file".yaml

4. Launch this node.

$ roslaunch turtlebot3_planner waypoint_planner.launch
