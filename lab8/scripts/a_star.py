#!/usr/bin/env python
# license removed for brevity
from cProfile import label
from cmath import sqrt
from re import X
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
import math
from queue import PriorityQueue
import matplotlib.pyplot as plt


# declare global variables
reached_node = False
node_threshold = 0.1
robot_x = None
robot_y = None
robot_theta = None
goal_pose = None
path_found = False

# declare map
my_map = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0], 
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]


# find node from priority quest that has the lowest value in priority queue
def find_lowest_node_in_dict(self,dict):
    try:
        min_val = np.Infinity
        for i in range(len(self.queue)):
            if self.queue[i] in dict.keys():
                if min > dict[self.queue[i]]:
                    min = dict[self.queue[i]]
                    lowest_node = self.queue[i]
        return lowest_node
    except IndexError:
        print()
        exit()

# find node from priority quest that has the lowest value in priority queue
def find_lowest_node_in_dict(my_prior_q,dict):
    try:
        min_val = np.Infinity
        for items in my_prior_q:
            item = items[1]
            if item in dict.keys():
                if min_val > dict[item]:
                    min_val = dict[item]
                    lowest_node = item
        return lowest_node

    except IndexError:
        print()
        exit()

# store robot's pose
def get_curr_pos(odom):
    global robot_x, robot_y,robot_theta

    # store robot pose
    robot_x = odom.pose.pose.position.x
    robot_y = odom.pose.pose.position.y
    orient = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    _,_,yaw = euler_from_quaternion(orient)
    robot_theta = yaw


# determine distance to goal    
def get_node_dist(node_x,node_y,robot_x,robot_y):
    return math.sqrt((node_x-robot_x)**2 + (node_y-robot_y)**2)


# compute translation vel based on if the robot is at a specific node or not    
def compute_trans_vel(rot_vel):
    if rot_vel == 0:
        return 0.6
    else:
        return 0    

# compute rotation of robot to face the goal during GOAL SEEK
def compute_node_seek_rot(node_pose):
    global robot_theta,robot_x,robot_y

    node_angle = math.atan2(node_pose[1]-robot_y,node_pose[0]-robot_x) - robot_theta

    if abs(node_angle) < math.pi/50:
        return 0
    else:
        return node_angle

# get path in an array form from cameFrom dict filled during A-star
def reconstruct_path(cameFrom, current, map):
    total_path = [convert_to_actual_coordinate(current)]
    while current in cameFrom.keys():
        current = cameFrom[current]
        current_act = convert_to_actual_coordinate(current)
        total_path.insert(0,current_act)
    return total_path

# convert the real position coordinate to map index
def convert_pos_to_mapindex(actual_node):
    zero_pos = [9,9] #[9,10]
    return [int(zero_pos[0]-actual_node[1]),int(zero_pos[1]+actual_node[0])]

# convert the map indexes to actual map coordinates
def convert_to_actual_coordinate(current_node):
    
    zero_pos = [9,9] #[9,10]
    str_cord = current_node[1:len(current_node)-1]
    cord_arr = str.split(str_cord,',')

    y = int(zero_pos[0]-int(cord_arr[0]))
    x = int(-zero_pos[1]+int(cord_arr[1]))
    return [x,y]

# compute the distance between 2 nodes
def compute_heuristic(start_map_index,goal_map_index):    
    return math.sqrt((start_map_index[0]-goal_map_index[0])**2+(start_map_index[1]-goal_map_index[1])**2)

# get 8 neighbors(not filled) of the current node
def get_neighbors_of_current_node(current_node,map):
    str_len = len(current_node)
    str_trun = current_node[1:str_len-1]
    split_arr = str.split(str_trun,',')
    x_cord = int(split_arr[0])
    y_cord = int(split_arr[1])

    neighbor_arr = []
    map = np.array(map)
    # top
    if (x_cord-1)>=0: 
        if map[x_cord-1,y_cord] != 1:
            neighbor_arr.append([x_cord-1,y_cord])

    # bottom
    if (x_cord+1)<len(map):
        if map[x_cord+1,y_cord] != 1:
            neighbor_arr.append([x_cord+1,y_cord])

    # left
    if (y_cord-1)>=0:
        if map[x_cord,y_cord-1] != 1:
            neighbor_arr.append([x_cord,y_cord-1])        

    # right
    if (y_cord+1)<len(map[0]): 
        if map[x_cord,y_cord+1] != 1:
            neighbor_arr.append([x_cord,y_cord+1])

    # top-left
    if (x_cord-1)>=0 and (y_cord-1)>=0:
        if map[x_cord-1,y_cord-1] != 1 and (map[x_cord-1,y_cord] == 0 or map[x_cord,y_cord-1] == 0):
            neighbor_arr.append([x_cord-1,y_cord-1])

    # top-right
    if (x_cord-1)>=0 and (y_cord+1)<len(map[0]):
        if map[x_cord-1,y_cord+1] != 1 and (map[x_cord-1,y_cord] == 0 or map[x_cord,y_cord+1] == 0):
            neighbor_arr.append([x_cord-1,y_cord+1])        

    # bottom-left
    if (x_cord+1)<len(map) and (y_cord-1)>=0:
        if map[x_cord+1,y_cord-1] != 1 and (map[x_cord+1,y_cord] == 0 or map[x_cord,y_cord-1] == 0):
            neighbor_arr.append([x_cord+1,y_cord-1])

    # bottom-right
    if (x_cord+1)<len(map) and (y_cord+1)<len(map[0]):
        if map[x_cord+1,y_cord+1] != 1 and (map[x_cord+1,y_cord] == 0 or map[x_cord,y_cord+1] == 0):
            neighbor_arr.append([x_cord+1,y_cord+1])        

    return neighbor_arr

# find if the current neighbor node is present in the priority queue or not
def find_neighbor_in_queue(neighbor_key,my_prior_q):    
    found = False
    for items in my_prior_q:
        item = items[1]
        if item == neighbor_key:
            found = True

    return found

def custom_print(my_map,planned_path):
    
    x_arr_map = []
    y_arr_map = []
    
    for i in range(len(planned_path)):
        map_index_arr = convert_pos_to_mapindex(planned_path[i])
        my_map[map_index_arr[0]][map_index_arr[1]] = 2
        
        x_arr_map.append(planned_path[i][0]-0.5)
        y_arr_map.append(planned_path[i][1]-0.5)

    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if my_map[i][j] == 1:
                print('1 ',end =" ")
            elif my_map[i][j] == 2:
                print('* ',end =" ")            
            else:
                print('  ',end =" ")

        print('\n')    
    text_labels = ['obstacle', 'explored', 'planned path']
    plt.scatter(x_arr_map,y_arr_map,color='red')
    plt.legend(text_labels)
    plt.grid()
    plt.show()

def compute_d(neighbor,current_node):
    str_cord = current_node[1:len(current_node)-1]
    cord_arr = str.split(str_cord,',')

    current = [int(cord_arr[0]),int(cord_arr[1])]

    return math.sqrt((neighbor[0]-current[0])**2+(neighbor[1]-current[1])**2)

# A* finds a path from start to goal.
def A_Star(start,goal,map):

    # declare cameFrom to be used for node selection list, g(n) and f(n) of the entire map
    cameFrom = {}
    gScore = {}
    fScore = {}
    epsilon = 0.1 #1
    visited_arr = []
    visited = False

    # get start key and goal key in map index
    start_map_index = convert_pos_to_mapindex(start)
    start_key = '('+str(start_map_index[0])+','+str(start_map_index[1])+')'
    
    goal_map_index = convert_pos_to_mapindex(goal)
    goal_key = '('+str(goal_map_index[0])+','+str(goal_map_index[1])+')'

    # assign all node's value as inifinity
    for i in range(len(map)*len(map[0])):
        key = '('+str(math.floor(i/len(map[0])))+','+str(i%len(map[0]))+')'
        gScore[key] = np.Infinity
        fScore[key] = np.Infinity

    # update g(n) and f(n) of the starting position
    gScore[start_key] = 0
    fScore[start_key] = compute_heuristic(start_map_index,goal_map_index)
    my_prior_q = []
    my_prior_q.append((fScore[start_key],start_key))
    
    # for plotting in MATPLOTLIB
    x_arr = []
    y_arr = []
    real_arr = []
    real_arr_map = []

    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] == 1:
                key = '('+str(i)+','+str(j)+')'
                # current_node = convert_to_actual_coordinate(key)
                real_arr_map.append(convert_to_actual_coordinate(key))
    
    x_arr_map = [i[0] for i in real_arr_map]
    y_arr_map = [i[1] for i in real_arr_map]            
    # plt.scatter(x_arr_map,y_arr_map,color='black')             

    while len(my_prior_q) != 0:
        # sort priority queue to get the node with lowest fscore        
        my_prior_q.sort(reverse=True)
        
        # get and delete the lowest fscore valued node from the priority queue
        current_node = my_prior_q.pop()[1]        
        
        visited_arr.append(current_node)

        real_arr.append(convert_to_actual_coordinate(current_node))
        x_arr = [i[0] for i in real_arr]
        y_arr = [i[1] for i in real_arr]

        plt.scatter(x_arr_map,y_arr_map,color='black')
        plt.scatter(x_arr,y_arr,color='blue')
        plt.grid()
        # plt.show()

        if current_node == goal_key:
            # get path in an array
            planned_path = reconstruct_path(cameFrom, current_node, map) 
            return planned_path

        # get neighbors of the current node
        neighbor_arr = get_neighbors_of_current_node(current_node,map)
        for neighbor in neighbor_arr:
            neighbor_key = '('+str(neighbor[0])+','+str(neighbor[1])+')'
            # tentative_gScore = gScore[current] + d(current, neighbor)
            tentative_gScore = gScore[current_node] + compute_d(neighbor,current_node)

            if tentative_gScore < gScore[neighbor_key]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor_key] = current_node
                gScore[neighbor_key] = tentative_gScore
                fScore[neighbor_key] = tentative_gScore + epsilon*compute_heuristic(neighbor,goal_map_index)
                if not find_neighbor_in_queue(neighbor_key,my_prior_q):
                    my_prior_q.append((fScore[neighbor_key],neighbor_key))
           
    # Open set is empty but goal was never reached
    return None

def a_star_processing():
    global goal_pose

    # declare map
    map = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0], 
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]
    
    #get goal's position from ros params
    goal_pose_x = float(rospy.get_param('goal_pose_x'))
    goal_pose_y = float(rospy.get_param('goal_pose_y'))
    goal_pose_z = float(rospy.get_param('goal_pose_z'))

    # store them in a list
    goal_pose = [goal_pose_x,goal_pose_y,goal_pose_z]
    # store goal and starting position
    p1 = np.array([-8.00,-2.00])
    goal_pos = [goal_pose[0],goal_pose[1]]
    start_pos = [-8.00,-2.00]

    pose_arr_act = A_Star(start_pos,goal_pos,map)

    # if no path found
    if pose_arr_act == None:
        print('No way to goal position!!')
        return 0

    actual_loc = []
    # add bias
    for pose in pose_arr_act:
        loc = [pose[0]+0.5,pose[1]+0.5]
        actual_loc.append(loc) 

    return actual_loc

# publish velocity commands based on the planned path
def execute_path(planned_path):
    global robot_x,robot_y,robot_theta,node_threshold,goal_pose

    # declare velocity publishing object
    vel_pub_obj = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    vel_command = Twist()
    
    # declare motion type
    motion_type = 'FOLLOW_PATH'

    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown() and motion_type != 'DONE':
        if robot_x != None and robot_y != None and robot_theta != None:
            
            # determine goal distance 
            goal_dist = get_node_dist(goal_pose[0],goal_pose[1],robot_x,robot_y)

            # check if robot already in the goal position or not
            if goal_dist<node_threshold:
                print('goal reached')
                fwd_vel = 0.0
                rot_vel = 0.0
                atGoal = True
                motion_type = 'DONE'

            if motion_type != 'DONE':
                for pose in planned_path:
                    # compute distance to node from the current pose of the robot
                    node_dist = get_node_dist(pose[0],pose[1],robot_x,robot_y)
                    while node_dist >= node_threshold:
                        
                        # compute rotaional velocity
                        rot_vel = compute_node_seek_rot(pose)

                        # compute translation velocity
                        trans_vel = compute_trans_vel(rot_vel)

                        # publish velocity command
                        vel_command.linear.x = trans_vel
                        vel_command.angular.z = rot_vel
                        vel_pub_obj.publish(vel_command)

                        vel_pub_obj.publish(vel_command)

                        # compute distance to node from the current pose of the robot
                        node_dist = get_node_dist(pose[0],pose[1],robot_x,robot_y)
                
                # Goal reached after traversing all position
                motion_type = 'DONE'
        
        if motion_type == 'DONE':
            print('Goal reached!')
            break
        # start iterating after 1/rate secs
        rate.sleep()


def initiate_controller():
    global my_map

    # odom subscriber to know robot's pose
    rospy.Subscriber("odom",Odometry,get_curr_pos, queue_size= 10)    

    #path planning using A star
    planned_path = a_star_processing()

    # #excute the planned path
    if planned_path != 0:
        execute_path(planned_path)
        custom_print(my_map,planned_path)

    rospy.spin()

if __name__ == '__main__':
    try:
        # declare node 
        rospy.init_node('a_star_node', anonymous=True)
        
        initiate_controller()
    except rospy.ROSInterruptException:
        pass
