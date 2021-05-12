#! /usr/bin/env python
# 
# # ENPM 661 - Planning for Autonomous Robots:
# Project 5 Phase 2 - RRT on Jetbot
# Shon Cortes, Bo-Shiang Wang

from logging import shutdown
import numpy as np
import matplotlib.pyplot as plt
import rospy
# from geometry_msgs.msg import Twist, Point
import random
from std_msgs.msg import String

# width = 400
# height = 300
# start_x = 0
# start_y = 0
# theta = 45
# goal_x = 200
# goal_y = 50

width = 248 # Units are inches
height = 245
start_x = 150
start_y = 150
theta = 45
goal_x = 208
goal_y = 105

start_theta = 0
step_size = 6
jbot_clearance = 4

# Initialize your ROS node
rospy.init_node("move_robot")
# Set up a publisher to the /cmd_vel topic
pub = rospy.Publisher("/jetbot_motors/cmd_str", String, queue_size=5) # (topic, data type, queue size)
rate = rospy.Rate(4)  # 4 Hz


# Class for storing node position, cost to come, parent index, and prev_orientation.
class Node:
    def __init__(self, x, y, radius=0, cost=0, parent_index=-1, prev_orientation=0, curr_orientation=0):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.radius = radius
        self.prev_orientation = prev_orientation
        self.curr_orientation = curr_orientation
        self.parent_node = None


def move_check(child_node):  # Check if the move is allowed.

    # Check if out of puzzle boundary
    if child_node.x < 0 or child_node.y < 0 or child_node.x >= width or child_node.y >= \
            height:
        return False

    # Check if obstacle
    elif obstacles_chk(child_node):
        return False

    else:
        return True


# Check if position is in Robot Adjusted obstacle space.
# Obstacle space was expanded by a radius of 10 + 5 for clearance for a total of 15.
# Warning obstacles appear larger than they are.
# def obstacles_chk(NODE):
#     node = [NODE.x, NODE.y]
#     # print(node)
#     # Rectangle
#     if (node[0] * 0.7) + 74.39 - 15 <= node[1] <= (node[0] * 0.7) + 98.568 + 15 \
#             and (node[0] * -1.428) + 176.554 - 15 <= node[1] <= (node[0] * -1.428) + 438.068 + 15:
#         return True

#     # Circle
#     elif (node[0] - 90) ** 2 + (node[1] - 70) ** 2 <= (35 + 15) ** 2:
#         return True

#     # Ellipse
#     elif ((node[0] - 246) ** 2) / ((60 + 15) ** 2) + ((node[1] - 145) ** 2) / ((30 + 15) ** 2) <= 1:
#         return True

#     # 3 section Rectangular Area
#     elif 200 - 15 <= node[0] <= 230 + 15 \
#             and 230 - 15 <= node[1] <= 280 + 15:  # First section
#         return True

#     else:
#         return False

def obstacles_chk(node):

    if node.x <= 40 + jbot_clearance and node.y <= 121 + jbot_clearance: # Obstacle 1
        return True
    
    if node.x <= 26 + jbot_clearance: # Obstacle 2
        return True

    if node.x <= 119 + jbot_clearance and node.y >= 219 - jbot_clearance: # Obstacle 3
        return True
    
    if node.x >= 208 - jbot_clearance and node.y >= 136 - jbot_clearance: # Obstacle 4
        return True
    
    if node.x >= 77 - jbot_clearance and node.y <= 74 + jbot_clearance: # Obstacle 5
        return True
    
    if node.x <= 89 + jbot_clearance and node.x >= 53 - jbot_clearance and node.y <= 182 + jbot_clearance and node.y >= 121 - jbot_clearance: # Obstacle 6
        return True

def begin():  # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:

        prev_orientation = start_theta
        # Initialize start and goal nodes from node class
        start_node = Node(start_x, start_y, 15, 0, -1, prev_orientation, prev_orientation)
        goal_node = Node(goal_x, goal_y, 15, 0, -1, 0, 0)

        start_node.cost = euclidean_dist(goal_node, start_node)
        # Check if obstacle
        if obstacles_chk(start_node):
            print("Start position is in an obstacle.")
        elif obstacles_chk(goal_node):
            print("Goal position is in an obstacle.")

        # Check if values are positive and within the map
        elif start_node.x < 0 or start_node.y < 0 or start_node.x > width or start_node.y > \
                height:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")
        elif goal_node.x < 0 or goal_node.y < 0 or goal_node.x > width or goal_node.y > \
                height:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")

        else:
            break

    return start_node, goal_node, step_size


def euclidean_dist(goal_node, node): # Calculate cost to goal
    dist = np.sqrt((goal_node.x - node.x) ** 2 + (goal_node.y - node.y) ** 2)
    return dist


def motion_model(orientation):
    orientation = np.deg2rad(orientation)
    theta_rad = np.deg2rad(theta)
    model = [[step_size * np.cos(2 * theta_rad + orientation), step_size * np.sin(2 * theta_rad + orientation), 1,
              np.rad2deg(2 * theta_rad + orientation)],  # 90
             [step_size * np.cos(theta_rad + orientation), step_size * np.sin(theta_rad + orientation), 1,
              np.rad2deg(theta_rad + orientation)],  # 45
             [step_size * np.cos(orientation), step_size * np.sin(orientation), 1, np.rad2deg(orientation)],  # 0
             [step_size * np.cos(-theta_rad + orientation), step_size * np.sin(-theta_rad + orientation), 1,
              360 - np.rad2deg(-theta_rad + orientation)],  # -45
             [step_size * np.cos(-2 * theta_rad + orientation), step_size * np.sin(-2 * theta_rad + orientation), 1,
              360 - np.rad2deg(-2 * theta_rad + orientation)]  # -90
             ]

    return model

def a_star(start_node, goal_node):
    # Initialize dictionaries
    path, distance, queue, visited = dict(), dict(), dict(), dict()

    queue[(start_node.x, start_node.y)] = start_node  # Initialize queue with startnode for Dijkstra algorithm.
    distance[(start_node.x, start_node.y)] = 0  # Initialize distance traveled.

    # threshold for the matrix V
    threshold = 0.5

    # Dictionary for orientation
    orientation_dict = {0: 0, 45: 1, 90: 2,
                        135: 3, 180: 4, 225: 5,
                        270: 6, 315: 7, 360: 0}

    # Create V matrix to store the information of the visited nodes.
    V = np.zeros((int(width / threshold) + 1, int(height / threshold) + 1, int(360 / 45)))

    while True:  # Start of A star Algorithm.
        # Find the node in queue with the minimum cost.
        cur_index = min(queue, key=lambda o: queue[o].cost + euclidean_dist(goal_node, queue[o])) # Assign node in queue with minimum cost to be the current node to be tested.
        cur = queue[cur_index]
        orientation = cur.prev_orientation

        # If goal node is reached, Break the while loop.
        # Add a threshold(circle) for the goal node
        if (goal_node.x - cur.x) ** 2 + (goal_node.y - cur.y) ** 2 <= (1.5 * step_size) ** 2:

            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            goal_node.curr_orientation = cur.curr_orientation
            print('Goal Found')
            break

        del queue[cur_index]  # Remove the current node from the queue.
        visited[(cur.x, cur.y, cur.prev_orientation)] = cur  # Add current node to visited list.

        # Mark 1 for visited nodes in matrix V
        a = int(round(cur.x) / threshold)
        b = int(round(cur.y) / threshold)
        c = orientation_dict[orientation]
        V[a][b][c] = 1

        # Initialize action set with orientation and step_size
        motion = motion_model(orientation)

        # Generate children of current node based on the action set.
        for i in range(len(motion)):
            next_x = round(cur.x + motion[i][0], 3)
            next_y = round(cur.y + motion[i][1], 3)
            child_orientation = round(motion[i][3])

            # Generate child node
            node = Node(next_x, next_y, 15, cur.cost + motion[i][2], cur_index, child_orientation, orientation)
            # Assign child node position
            node_index = (node.x, node.y)

            if move_check(node):  # Check if child is within the map or in an obstacle.
                pass
            else:  # If out of bounds or an obstacle, restart loop and choose new node.
                continue

            a = int(round(node.x))
            b = int(round(node.y))
            # c = int(node.prev_orientation / 30)
            if node.prev_orientation > 360:
                node.prev_orientation = node.prev_orientation - 360
            c = orientation_dict[node.prev_orientation]
            # If the next node is already visited, skip it
            if V[a][b][c] == 1:
                continue

            # Visualize motion
            plt.quiver(cur.x, cur.y, motion[i][0], motion[i][1], units='xy', scale=1, color='r', width=.1)
            plt.pause(.0001)

            # If the child node is already in the queue, compare and update the node's cost and parent as needed.
            if node_index in queue:
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:  # Else add child to the queue.
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path_x, path_y = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    child = visited[(parent_index[0], parent_index[1], goal_node.curr_orientation)]
    plt.quiver(child.x, child.y, goal_node.x - child.x, goal_node.y - child.y,
               units='xy', scale=1, color='r', width=.1)

    ori = child.prev_orientation

    while parent_index != (start_node.x, start_node.y):  # Follow the parents from the goal node to the start node and add them to the path list.
        n = visited[(parent_index[0], parent_index[1], ori)]
        path_x.append(n.x)
        path_y.append(n.y)
        parent_index = n.parent_index
        ori = n.curr_orientation

    path_x.append(start_node.x)
    path_y.append(start_node.y)

    return path_x, path_y


def get_nearest_node_index(node_list, rand_node):
    # Generate a list to store the euclidean distance between all the nodes in node_list and the random node
    dist_list = [(node.x - rand_node.x)**2 + (node.y - rand_node.y)**2
                 for node in node_list]
    min_index = dist_list.index(min(dist_list))
    return min_index


def calc_dist_angle(node_1, node_2):
    d = euclidean_dist(node_2, node_1)
    angle = np.arctan2((node_2.y - node_1.y), (node_2.x - node_1.x))
    return d, angle


def node_expansion(nearest_node, rand_node):
    new_node = Node(nearest_node.x, nearest_node.y)
    motion = motion_model(orientation=0)
    node_path_x = [new_node.x]
    node_path_y = [new_node.y]

    dist, angle = calc_dist_angle(new_node, rand_node)
    angle = np.rad2deg(angle)


    next_x = new_node.x + step_size*np.cos(angle)
    next_y = new_node.y + step_size*np.sin(angle)



    new_node.x = next_x
    new_node.y = next_y

    dist = euclidean_dist(rand_node, new_node)
    if dist <= step_size:

        new_node.x = rand_node.x
        new_node.y = rand_node.y

    node_path_x.append(new_node.x)
    node_path_y.append(new_node.y)

    new_node.parent_index = (new_node.x, new_node.y)
    return new_node, node_path_x, node_path_y


def rrt(start_node, goal_node):
    node_list = [start_node]
    max_iteration = 500
    for i in range(max_iteration):
        rand_node = Node(random.randint(0, width), random.randint(0, height))
        nearest_node_index = get_nearest_node_index(node_list, rand_node)
        nearest_node = node_list[nearest_node_index]
        next_node, node_path_x, node_path_y = node_expansion(nearest_node, rand_node)

        if move_check(next_node):  # Check if child is within the map or in an obstacle.
            pass
        else:  # If out of bounds or an obstacle, restart loop and choose new node.
            continue

        next_node.cost = euclidean_dist(goal_node, next_node) # Cost to go
        next_node.parent_node = nearest_node

        # Visualize path
        plt.quiver(nearest_node.x, nearest_node.y, next_node.x - nearest_node.x, next_node.y - nearest_node.y,
                units='xy', scale=1, color='r',
                width=.1)
        plt.pause(.0001)

        node_list.append(next_node)

    return node_list

"""
RRT list (random order. size 1000)
start from start point 
generate actions based off action set
pick points that are within a threshold of RRT list points and adds it to the visited list
Picks the node with the loswet cost in visited list to generate next actions and repeat the above steps
FIND GOAL
"""

def backtrack(start_node, node_list):
  
    node  = min(node_list, key=lambda o: o.cost) # goal
    path = [node]
    while node is not start_node:

        parent = node.parent_node
        path.append(parent)

        # Visualize path
        plt.quiver(parent.x, parent.y, node.x - parent.x, node.y - parent.y,
                units='xy', scale=1, color='g',
                width=.1)
        plt.pause(.0001)

        node = parent
    path.pop()
    path.reverse() # Start to goal
    return path

def move_bot(start_node, path):

    current = start_node
    prev_rotate = 0

    for i in range(len(path)):

        waypoint = path[i]
        rotate = np.rad2deg(np.arctan2((waypoint.y - current.y), (waypoint.x - current.x)))
        # # calc magnitude / stepsize    

        if rotate > 0:
            rotate = abs(rotate) + prev_rotate
            time = rotate / 90
            pub.publish("left")
            rospy.sleep(time)
            pub.publish("stop")

        elif rotate < 0:
            rotate = abs(rotate) + prev_rotate
            time = rotate / 90
            pub.publish("right")
            rospy.sleep(time)
            pub.publish("stop")
        
        pub.publish("forward")
        rospy.sleep(1) # 1 second for a 6 inch move forward
        pub.publish("stop")

        current = waypoint

        if rotate > 360:
            prev_rotate = rotate - 360
        else:
            prev_rotate = rotate

    print('done')




def main():
    # set obstacle positions
    ox, oy = [], []
    for i in range(0, width):
        for j in range(0, height):
            # # Circle
            # if (i - 90) ** 2 + (j - 70) ** 2 <= 35 ** 2:
            #     ox.append(i)
            #     oy.append(j)

            # # Ellipse
            # if ((i - 246) ** 2) / (60 ** 2) + (((j - 145) ** 2) / (30 ** 2)) <= 1:
            #     ox.append(i)
            #     oy.append(j)

            # if (i * 0.7) + 74.39 <= j <= (i * 0.7) + 98.568 \
            #         and (i * -1.428) + 176.554 <= j <= (i * -1.428) + 438.068:
            #     ox.append(i)
            #     oy.append(j)

            # # 3 Section Rectangular Area
            # if 200 <= i <= 210 and 230 <= j <= 280:
            #     ox.append(i)
            #     oy.append(j)
            # if 210 <= i <= 230 and 270 <= j <= 280:
            #     ox.append(i)
            #     oy.append(j)
            # if 210 <= i <= 230 and 230 <= j <= 240:
            #     ox.append(i)
            #     oy.append(j)

            if i <= 40 + jbot_clearance and j <= 121 + jbot_clearance: # Obstacle 1
                ox.append(i)
                oy.append(j)
            
            if i <= 26 + jbot_clearance: # Obstacle 2
                ox.append(i)
                oy.append(j)

            if i <= 119 + jbot_clearance and j >= 219 - jbot_clearance: # Obstacle 3
                ox.append(i)
                oy.append(j)
            
            if i >= 208 - jbot_clearance and j >= 136 - jbot_clearance: # Obstacle 4
                ox.append(i)
                oy.append(j)
            
            if i >= 77 - jbot_clearance and j <= 74 + jbot_clearance: # Obstacle 5
                ox.append(i)
                oy.append(j)
            
            if i >= 53 - jbot_clearance and i <= 89 + jbot_clearance and j <= 182 + jbot_clearance and j >= 121 - jbot_clearance: # Obstacle 6
                ox.append(i)
                oy.append(j)

    start_node, goal_node, step_size = begin()

    plt.xlim([0, 400])
    plt.ylim([0, 300])
    plt.plot(ox, oy, ".k")
    plt.plot(start_x, start_y, "xr")
    plt.plot(goal_x, goal_y, "xr")
    plt.grid(True)
    plt.axis("equal")

    a = [start_node.x, start_node.y]
    b = [goal_node.x, goal_node.y]

    if a != b:
        # path_x, path_y = a_star(start_node, goal_node)  # Call A star algorithm
        #
        # plt.plot(path_x, path_y, "-g")
        # plt.pause(0.0001)
        # plt.show()
        node_list = rrt(start_node, goal_node)
        #Use node list in A*
        path = backtrack(start_node, node_list)
        move_bot(start_node, path)
        print('done')
        plt.show()

    else:
        print("Start position equals the goal position.")


if __name__ == '__main__':
    main()


"""
jetbot_motor.py edited maxpwn to be 1/4 speed.
This reduced the motor speed by 1/4th.
a 90 deg turn takes about 1 second.
"""