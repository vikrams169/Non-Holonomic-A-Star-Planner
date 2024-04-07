#!/usr/bin/env python3

'''
Link to the GitHub Repository:
https://github.com/vikrams169/Non-Holonomic-A-Star-Planner
'''

# Importing the required libraries
from heapq import *
import numpy as np
import cv2
import time
import copy
#ROS Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Size of the grid and dimensions of the grid block
map_size = [6000,2000]
CLEARANCE = 10
ROBOT_RADIUS = 220
WHEEL_RADIUS = 33
WHEEL_SEPARATION = 287
RPM1 = 30
RPM2 = 60
GOAL_RADIUS = ROBOT_RADIUS//2
ACTIONS = [[0,RPM1],[0,RPM2],[RPM1,0],[RPM2,0],[RPM1,RPM1],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
ACTION_COSTS = [0,0,0,0,0,0,0,0]
dt_timestep = 0.5
dt_sample = 0.25

#Initialising variables for map and frame
frame = np.full([map_size[1],map_size[0],3],(0,255,0)).astype(np.uint8)
obstacle_frame = np.full([map_size[1],map_size[0],3], (0,255,0)).astype(np.uint8)

min_heap = [] # Min-heap to store nodes initialized as a list
heapify(min_heap) # Making the min-heap list into a heap using the heapq library
start_pos = None # Start position of the planner
goal_pos = None # Goal position of the planner
solved = False # Boolean to check whether the path has been fully solved or not
iteration = 0 # Number of iterations in A* so far
frame_number = 0 # Current number of frames that have been saved

#Velocity array
velocity_nodes = []

# Return a boolean value of whether a grid cell lies within an obstacle (or in its clearance)
def in_obstacle(loc):
    global obstacle_frame
    if np.all(obstacle_frame[int(loc[1]),int(loc[0])]):
        return False
    else:
        return True

# Returning whether a current grid cell is the goal or not
def reached_goal(loc):
    if ((goal_pos[0]-loc[0])**2 + (goal_pos[1]-loc[1])**2)**0.5 <= GOAL_RADIUS:
        return True
    else:
        return False

# Backtracking to find the path between the start and goal locations
def compute_final_path(final_reached_loc):
    global frame, velocity_nodes
    path_nodes = [final_reached_loc[0:3]]
    velocity_nodes.append(final_reached_loc[3:5])
    while not np.array_equal(path_nodes[-1][:2],start_pos[:2]):
        parent_loc = path_nodes[-1]
        path_nodes.append((int(node_info[int(parent_loc[0]/distance_threshold),
                                        int(parent_loc[1]/distance_threshold),
                                        int(parent_loc[2]%360/angular_threshold),1]),
                           int(node_info[int(parent_loc[0]/distance_threshold),
                                        int(parent_loc[1]/distance_threshold),
                                        int(parent_loc[2]%360/angular_threshold),2]),
                           int(node_info[int(parent_loc[0]/distance_threshold),
                                        int(parent_loc[1]/distance_threshold),
                                        int(parent_loc[2]%360/angular_threshold),3])))
        velocity_nodes.append((node_info[int(parent_loc[0]/distance_threshold),
                                        int(parent_loc[1]/distance_threshold),
                                        int(parent_loc[2]%360/angular_threshold),4],
                              node_info[int(parent_loc[0]/distance_threshold),
                                        int(parent_loc[1]/distance_threshold),
                                        int(parent_loc[2]%360/angular_threshold),5]))
    path_nodes.reverse()
    velocity_nodes.reverse()

# Initializing start and goal positions using user input
def initialize_start_and_goal_pos():
    global start_pos, goal_pos
    print("Please enter the goal positions (in the coordinate system with the origin at the bottom left corner) starting from index 0")
    print("Make sure to to add locations within the 6000mm*2000mm grid map avoiding obstacles accounting for a 5mm clearance")
    while(start_pos is None or goal_pos is None):
        start_x = 500
        start_y = 1000
        start_theta = 0
        goal_x = int(input("Enter the X coordinate of the goal position: ") or 5750)
        goal_y = map_size[1] - int(input("Enter the Y coordinate of the goal position: ") or 1000)
        if goal_x < 0 or goal_x >= map_size[0] or goal_y < 0 or goal_y >= map_size[1] or in_obstacle((goal_x,goal_y)):
            print("Try again with a valid set of values")
            continue
        start_pos = (int(start_x),int(start_y),int(start_theta),0,0)
        goal_pos = (int(goal_x),int(goal_y))
    
    print("Starting position of robot", start_pos)
    print("Goal position of robot", goal_pos)

#Function to set obstacles in a frame
def set_obstacles(frame):
    #Half planes
    #Clear area
    frame[CLEARANCE:map_size[1]-CLEARANCE, CLEARANCE:map_size[0]-CLEARANCE] = [255,255,255]
    #Rectangle clearance
    frame[0:1000+CLEARANCE, 1500-CLEARANCE:1500+250+CLEARANCE] = [0,255,0]
    frame[1000-CLEARANCE:2000, 2500-CLEARANCE:2500+250+CLEARANCE] = [0,255,0]
    #Rectangle obstacles
    frame[0:1000, 1500:1500+250] = [0,0,0]
    frame[1000:2000, 2500:2500+250] = [0,0,0]
    #Circle
    center = (4200, 800)
    radius = 600
    Y, X = np.ogrid[:2000, :6000]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
    #Clearance
    mask = dist_from_center <= radius+CLEARANCE
    frame[mask] = [0,255,0]
    # #Obstacle
    mask = dist_from_center <= radius
    frame[mask] = [0,0,0]
    return frame

# Initializing the map with obstacles in PyGame
def initialize_map():
    global obstacle_frame, frame, CLEARANCE, ROBOT_RADIUS, test_frame
    frame = set_obstacles(frame)
    # #Radius of robot
    CLEARANCE = CLEARANCE + ROBOT_RADIUS
    obstacle_frame = set_obstacles(obstacle_frame)

# Visualizing the possible action by drawing the spline in the frame and observing the cost of that action
def action_step(parent_loc,u_l,u_r,action_number):
    global ACTION_COSTS, frame
    num_timesteps = int(dt_timestep/dt_sample)
    x, y, theta, linear, angular = parent_loc
    parent_theta = theta
    spline_length = 0
    complete_path = True
    for i in range(num_timesteps):
        x_dot = (WHEEL_RADIUS/2)*(u_l+u_r)*np.cos(np.deg2rad(theta))
        y_dot = -(WHEEL_RADIUS/2)*(u_l+u_r)*np.sin(np.deg2rad(theta))
        theta_dot = (WHEEL_RADIUS/WHEEL_SEPARATION)*(u_r-u_l)
        x_new = x + x_dot*dt_sample
        y_new = y + y_dot*dt_sample
        theta_new = np.rad2deg(np.deg2rad(theta) + theta_dot*dt_sample)

        if x_new < 0 or x_new >= map_size[0] or y_new < 0 or y_new >= map_size[1] or in_obstacle([x_new,y_new]):
            complete_path = False
            break

        spline_length += ((x_dot*dt_sample)**2 + (y_dot*dt_sample)**2)**0.5
        frame = cv2.line(frame,(int(x),int(y)),(int(x_new),int(y_new)),(255,0,0),1)
        x, y, theta = x_new, y_new, theta_new
    
    #Velocity calculation
    linear_vel = (u_l + u_r)*WHEEL_RADIUS/2 
    angular_vel = np.deg2rad((theta-parent_theta))/dt_timestep

    if complete_path:
        ACTION_COSTS[action_number] = spline_length
    x, y, theta = int(x), int(y), int(theta)

    if node_info[int(x/distance_threshold), int(y/distance_threshold), int(theta%360/angular_threshold),0] == 1:
        return -1
    else:
        return [x,y,theta,linear_vel,angular_vel]

# Adding a node to the min-heap
def add_node_to_heap(parent_loc,new_loc,action):
    global min_heap
    dist_c2c = node_info[int(parent_loc[0]/distance_threshold),
                        int(parent_loc[1]/distance_threshold),
                        int(parent_loc[2]%360/angular_threshold),7] + ACTION_COSTS[action]
    dist = dist_c2c + ((goal_pos[0]-new_loc[0])**2 + (goal_pos[1]-new_loc[1])**2)**0.5
    heappush(min_heap,(dist,new_loc,parent_loc,action))

# Adding all the neighbors of the current node to the min-heap
def add_neighbors(loc):
    global frame
    i = 0
    for action in ACTIONS:
        u_l = (action[0]*2*np.pi)/60
        u_r = (action[1]*2*np.pi)/60
        new_loc = action_step(loc,u_l,u_r,i)
        if new_loc != -1:
            add_node_to_heap(loc,new_loc,i)
        i += 1

# Processing the current node that was returned by popping the min-heap
def process_node(node):
    global node_info, solved, iteration, frame
    dist, loc, parent_loc, action = node
    #if visited
    if node_info[int(loc[0]/distance_threshold),
                 int(loc[1]/distance_threshold),
                 int(loc[2]%360/angular_threshold),0] == 1:
        return
    #mark visited
    node_info[int(loc[0]/distance_threshold),
              int(loc[1]/distance_threshold),
              int(loc[2]%360/angular_threshold),0] = 1
    #mark parent loc
    node_info[int(loc[0]/distance_threshold),
              int(loc[1]/distance_threshold),
              int(loc[2]%360/angular_threshold),1] = parent_loc[0]
    node_info[int(loc[0]/distance_threshold),
              int(loc[1]/distance_threshold),
              int(loc[2]%360/angular_threshold),2] = parent_loc[1]
    node_info[int(loc[0]/distance_threshold),
              int(loc[1]/distance_threshold),
              int(loc[2]%360/angular_threshold),3] = parent_loc[2]
    #velocities
    node_info[int(loc[0]/distance_threshold),
              int(loc[1]/distance_threshold),
              int(loc[2]%360/angular_threshold),4] = loc[3]
    node_info[int(loc[0]/distance_threshold),
              int(loc[1]/distance_threshold),
              int(loc[2]%360/angular_threshold),5] = loc[4]
    #total cost
    node_info[int(loc[0]/distance_threshold),
                 int(loc[1]/distance_threshold),
                 int(loc[2]%360/angular_threshold),6] = dist
    #cost to start (c2c)
    if np.array_equal(loc,start_pos):
        node_info[int(loc[0]/distance_threshold),
                 int(loc[1]/distance_threshold),
                 int(loc[2]%360/angular_threshold),7] = 0
    else:
        node_info[int(loc[0]/distance_threshold),
                 int(loc[1]/distance_threshold),
                 int(loc[2]%360/angular_threshold),7] = node_info[int(parent_loc[0]/distance_threshold),
                                                            int(parent_loc[1]/distance_threshold),
                                                            int(parent_loc[2]%360/angular_threshold),7] + ACTION_COSTS[action] 
    #goal check
    if reached_goal(loc):
        solved = True
        compute_final_path(loc)
    add_neighbors(loc)

# Wrapper function for the A* Algorithm
def a_star():
    global min_heap, solved
    starting_dist = ((goal_pos[0]-start_pos[0])**2 + (goal_pos[1]-start_pos[1])**2)**0.5
    heappush(min_heap,(starting_dist,list(start_pos),list(start_pos),-1))
    while not solved:
        try:
            node = heappop(min_heap)
        except:
            print("Cannot find solution")
            break
        process_node(node)

#Robot Controller Node
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def drive_robot(self):
        velocity_message = Twist()
        global velocity_nodes
        for velocity in velocity_nodes:
            velocity_message.linear.x = velocity[0]/1000
            velocity_message.angular.z = velocity[1]
            self.cmd_vel_pub.publish(velocity_message)
            time.sleep(dt_timestep)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = 0.0
        self.cmd_vel_pub.publish(velocity_message)

def main(args=None):
    global node_states, start_pos, goal_pos, CLEARANCE, RPM1, RPM2, angular_threshold, distance_threshold, node_info

    rclpy.init(args=args)
    node = RobotControlNode()
    # Taking the clearnace as a user-input
    CLEARANCE = int(input("Enter the clearance/bloating value for the obstales and walls: ") or 10)

    # Initializing the map with obstacles
    initialize_map()

    # Taking the wheel RPMs as a user-input
    RPM1 = int(input("Enter the first possible non-zero wheel RPM value: "))
    RPM2 = int(input("Enter the second possible non-zero wheel RPM value: "))

    # Defining the distance threshold as a function of the user-given RPM values
    angular_threshold = 30
    distance_threshold = int(min(RPM1,RPM2)/2)
    # Node Info Explanation
    '''
    for each cell in the grid (indexed by node_states[row_index][column_index]), there are FIVE dimensions
    Index 0: Node state i.e. -1 for not explored yet, 0 for open, 1 for closed
    Index 1: Parent node row index
    Index 2: Parent node column index
    Index 3: Parent node angle index
    Index 4: linear_vel to get to node
    Index 5: angular_vel to get to node
    Index 4: Total cost (C2C + C2G)
    Index 5: CTC
    '''

    node_info = -1*np.ones((int(map_size[0]/distance_threshold),
                            int(map_size[1]/distance_threshold),
                            int(360/angular_threshold),
                            8)) # Information about node distance, parents, and status

    # Initializing the start and goal positions of the path from user input
    initialize_start_and_goal_pos()

    print("\nStarting the Pathfinding Process! This may take upto a minute.\n")
    # Running the A* Algorithm
    start_time = time.time()
    a_star()
    end_time = time.time()

    print("\nSuccess! Found the Optimal Path\n")
    print("\nTime taken for the pathfinding process using the A* Algorithm: ",end_time-start_time," seconds\n")
    print("\nDriving the robot in gazebo\n")
    node.drive_robot()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()