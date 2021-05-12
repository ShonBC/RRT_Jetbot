# RRT_Turtlebot
Navigate a differential drive robot (TurtleBot 2 / TurtleBot 3) in a given map environment from a given start point to a given goal point. Consider differential drive constraints while implementing the A* algorithm, with 8 set of action space.

ENPM 661 - Planning for Autonomous Robots

Project 3 - Phase 3 A* Simulation on Turtlebot using ROS

Shon Cortes, Bo-Shiang Wang

Packages Used:

	1. import numpy
    2. import matplotlib.pyplot as pl 
        - Used for visualization of the path planning.
    3. from logging import shutdown
    4. import rospy
    5. from geometry_msgs.msg import Twist, Point

Run the program:

    1. Import ROS package into catkin workspace src folder.

    2. Rebuild the catkin workspace.

    3. Re-source:

        source devel/setup.bash

	Initialize a ROS Master in the terminal:

        roscore
    
    Launch the planning.launch file to start gazebo:

        roslaunch planning_turtlebot planning.launch
    
    Run the a_star.py file to plan a path from start (0.5, 0.5) to goal (9, 9):

        rosrun planning_turtlebot a_star.py
    
Program Summary:
    
    Uses A* path planning algorithm to find the optimal path from the hard coded start and goal points using set RPM values
    for the right and left wheels. Once a path is found, the wheel RPM values are published to the cmd_vel topic. The Gazebo 
    simulation shows the turtlebot following the planned path by following the RPM values published. 