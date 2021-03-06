#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math
import sys
import numpy as np
from Point import Point

#predefined trajectory 
traj = [Point(10,0),Point(-6,12),Point(2,24),Point(-20,-16),Point(-3,-7),Point(20,-20)]
brojac_traj = 0

#for laser readings
front = "front"
fleft = "fleft"
fright = "fright"
left = "left"
right = "right"

#current goal from traj
goal = traj[brojac_traj]
#current position of robot
current_pos = Point()
#current orientation in global coordinate 
orientation = 0.0
#variable for cmd_vel
twist = Twist()
left_oi_chosen = False

#data from laser
angle_min = None
angle_max = None
angle_increment = None
time_increment = None
scan_time = None
range_min = None
range_max = None
ranges = [0]*720
intensities = []


#acceptable error for angle and distance
eps_angle = math.radians(3) #3 degrees accptable
eps_dist = 0.1 #10 cm acceptable

#local minimum heuristic distance from goal
min_dist_to_goal = np.inf
min_dist_to_goal_point = None

#0 go to goal
#1 go to oi
#2 follow obstacle 
#3 stop
state = 0

#for arguments from terminal for goal point
def my_node(xarg,yarg):
    global goal
    goal.x = float(xarg)
    goal.y = float(yarg)

#reading current pose of robot
def odometrija(pose):
    global current_pos
    global orientation

    current_pos.x = pose.pose.pose.position.x
    current_pos.y = pose.pose.pose.position.y
    q = pose.pose.pose.orientation
    q_euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    yaw = q_euler[2]
    orientation = yaw

#transform global coordinates to local
def global2robot_transform(point):
    c = np.cos(orientation)
    s = np.sin(orientation)
    #transformation matrix
    transf_matrix = np.array([[c,s,0,-(c*current_pos.x+s*current_pos.y)],[-s,c,0,s*current_pos.x-c*current_pos.y],[0,0,1,0],[0,0,0,1]])
    np_point = np.array([point.x,point.y,0,1])
    np_point = np_point.transpose()
    tf_point = transf_matrix.dot(np_point)
    return Point(tf_point[0],tf_point[1])

#transform local coordinates to global
def robot2global_transform(point):
    c = np.cos(orientation)
    s = np.sin(orientation)
    transf_matrix = np.array([[c,-s,0,current_pos.x],[s,c,0,current_pos.y],[0,0,1,0],[0,0,0,1]])
    np_point = np.array([point.x,point.y,0,1])
    np_point = np_point.transpose()
    tf_point = transf_matrix.dot(np_point)
    return Point(tf_point[0],tf_point[1])

#reading laser data
def skeniranje(scan):
    global angle_min 
    global angle_max
    global angle_increment
    global time_increment
    global scan_time
    global range_min
    global range_max
    global ranges
    global intensities
    
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_increment = scan.angle_increment
    time_increment = scan.time_increment
    scan_time = scan.scan_time
    range_min = scan.range_min
    range_max = scan.range_max
    ranges = scan.ranges
    intensities = scan.intensities

#formating angle from -pi to pi
def format_angle(angle):
    while angle < - math.pi:
        angle = angle + 2 * math.pi
    while angle > math.pi:
        angle = angle - 2 * math.pi
    return angle

#angle of goal in local coordinates
def angle_of_goal():
    angle = math.atan2(goal.y-current_pos.y,goal.x-current_pos.x)-orientation
    angle = format_angle(angle)
    return angle

#transform angle to index of laser data list
def degrees_to_index(angle):
    return int(math.floor((angle+135)/math.degrees(angle_increment)))

#transform angle in radians to index of laser data list
def radians_to_index(angle):
    return int(math.floor((angle+math.radians(135))/angle_increment))

#return coordinates of point that laser read at particular angle 
def point_coordinates(angle,index_par = None):
    if index_par != None:
        index = index_par
        angle = math.radians(-135+index*math.degrees(angle_increment))
    else:
        index = degrees_to_index(angle)
    dist = ranges[index]
    x = dist * math.cos(angle)
    y = dist * math.sin(angle)
    return Point(x,y)

#return start and stop point of obstacle 
#if None no obstacle detected
#return in local coordinates
def find_ois_coordinates():
    global min_dist_to_goal
    global min_dist_to_goal_point
    goal_angle = angle_of_goal()

    #return none if robot cant see goal
    if goal_angle>math.radians(135) or goal_angle<math.radians(-135):
        return None
    ind_goal_ang = radians_to_index(goal_angle)
    if ind_goal_ang < 0 or current_pos.distance(goal)<ranges[ind_goal_ang] or ind_goal_ang>719:
        return None

    pom_ranges = list(ranges)
    pom_ranges.insert(0,np.inf)
    diff_np_ranges = np.diff(np.array(pom_ranges))
    indices_inf = list(np.where(diff_np_ranges == np.inf)[0]-1) #ends of obstacles 
    indices_minf = list(np.where(diff_np_ranges == -np.inf)[0]) #beginings of obstacles
    if len(indices_inf)<len(indices_minf):
        indices_inf.append(719) #end at end of laser readings

    for i in range(len(indices_inf)):
        if ind_goal_ang <= indices_inf[i]and ind_goal_ang >= indices_minf[i]:
            point1 = point_coordinates(None,indices_inf[i]) #end point
            point2 = point_coordinates(None,indices_minf[i])#start point

            #for setting local minimum of heuristic distance
            point_between = []
            for j in range(indices_minf[i],indices_inf[i]):
                point_between.append(point_coordinates(None,j))
            for j in point_between:
                p = robot2global_transform(j)
                dist = goal.distance(p)
                if dist<min_dist_to_goal:
                    min_dist_to_goal = dist
                    min_dist_to_goal_point = j
            return [point1,point2]

    return None

#return oi with smaller heuristic distance
#in local coordinates
def choose_oi(ois):
    global left_oi_chosen

    oi0_global = robot2global_transform(ois[0])
    oi1_global = robot2global_transform(ois[1])

    dist1 = current_pos.distance(oi0_global)+goal.distance(oi0_global)
    dist2 = current_pos.distance(oi1_global)+goal.distance(oi1_global)
    if dist1<dist2:
        left_oi_chosen = True
        return ois[0]
    elif math.fabs(dist1-dist2)<0.001:
        if left_oi_chosen == True:
            return ois[0]
        else:
            left_oi_chosen = False
            return ois[1]
    else:
        left_oi_chosen = False
        return ois[1]
    
#return min dist of laser region - 5 regions
def min_ranges(direction):
    if direction == front:
        return min(min(ranges[288:431]), 10)
    if direction == right:
        return min(min(ranges[0:143]), 10)
    if direction == fright:
        return min(min(ranges[144:287]), 10)
    if direction == fleft:
        return min(min(ranges[432:575]), 10)
    if direction == left:
        return min(min(ranges[576:719]), 10)

#set cmd_vel variable 
def sleep():
    global twist
    twist.linear.x = 0
    twist.angular.z = 0

def go_forward():
    global twist
    twist.linear.x = 1
    twist.angular.z = 0

def turn_left():
    global twist
    twist.linear.x = 0.1
    twist.angular.z = 0.7

def turn_right():
    global twist
    twist.linear.x = 0.1
    twist.angular.z = -0.7

def go_back():
    global twist
    twist.linear.x = -1
    twist.angular.z = 0

def go_fleft():
    global twist
    twist.linear.x = 0.3
    twist.angular.z = 0.3

def go_fright():
    global twist
    twist.linear.x = 0.3
    twist.angular.z = -0.3

#go to goal
def state_0():
    print "Heading toward target"
    if math.fabs(angle_of_goal())<eps_angle:
        go_forward()
        return
    if angle_of_goal()>0: 
        turn_left()
    else:
        turn_right()

#go to oi
def state_1():
    global state
    print "Heading toward Oi"
    p = find_ois_coordinates()
    if p == None:
        state = 0
        return 
    oi = choose_oi(p)
    oi_angle = math.atan2(oi.y,oi.x)
    if math.fabs(oi_angle)<eps_angle:
        go_forward()
    elif oi_angle>0:
        go_fleft()
    else:
        go_fright()
    return

#follow obstacle
def state_2():
    global state
    d = 1.5
    print "Following the obstacle"
    if min_ranges(front)>d+0.2 and min_ranges(fleft)>d+0.2 and min_ranges(fright)>d+0.2 and min_ranges(left) > d+0.2 and min_ranges(right) >d+0.2:
        state = 0
        return
    elif min_ranges(front)>d and min_ranges(fleft)>d and min_ranges(fright)>d and min_ranges(left) > d and min_ranges(right)>d:
        go_forward()
        return
    elif min_ranges(front)>d and min_ranges(fleft)>d and min_ranges(fright)>d and (min_ranges(left) <d or min_ranges(right) <d):
        go_forward()
        return
    if min_ranges(front)<d and min_ranges(fleft)>d and min_ranges(fright)>d:
        if left_oi_chosen == True:
            turn_left()
        else:
            turn_right()
    elif min_ranges(front)>d and min_ranges(fleft)>d and min_ranges(fright)<d:
        go_forward()
    elif min_ranges(front)>d and min_ranges(fleft)<d and min_ranges(fright)>d:
        go_forward()
    elif min_ranges(front)<d and min_ranges(fleft)>d and min_ranges(fright)<d:
        if left_oi_chosen == True:
            turn_left()
        else:
            turn_right()
    elif min_ranges(front)<d and min_ranges(fleft)<d and min_ranges(fright)>d:
        if left_oi_chosen == True:
            turn_left()
        else:
            turn_right()
    elif min_ranges(front)<d and min_ranges(fleft)<d and min_ranges(fright)<d:
        if left_oi_chosen == True:
            turn_left()
        else:
            turn_right()
    elif min_ranges(front)>d and min_ranges(fleft)<d and min_ranges(fright)<d:
        go_forward()
 
 #stop
def state_3():
    print "I am done."
    sleep()

#publisher function
def publisher():
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    rospy.init_node('publisher',anonymous=True)
    rate = rospy.Rate(25)
    
    global current_pos
    global orientation
    global ranges
    global goal
    global state
    global twist
    global brojac_traj

    while not rospy.is_shutdown():
        if state == 0:
            if current_pos.distance(goal)<eps_dist:
                brojac_traj = brojac_traj+1
                if brojac_traj == len(traj):
                    state = 3
                    state_3()
                else:
                    goal = traj[brojac_traj]
            elif min_ranges(front)<1.5:
                state = 2
                state_2()
            elif find_ois_coordinates()==None:
                
                state = 0
                state_0()
            
            else:
                state = 1
                state_1()
        elif state == 1:
            if current_pos.distance(goal)<eps_dist:
                brojac_traj = brojac_traj+1
                if brojac_traj == len(traj):
                    state = 3
                    state_3()
                else:
                    goal = traj[brojac_traj]
            elif current_pos.distance(goal)<min_dist_to_goal or find_ois_coordinates()==None:
                state = 0
                state_0()
            elif min_ranges(front)<1.5:
                state = 2
                state_2()
            else:
                state = 1
                state_1()
        elif state == 2:
            if current_pos.distance(goal)<eps_dist:
                brojac_traj = brojac_traj+1
                if brojac_traj == len(traj):
                    state = 3
                    state_3()
                else:
                    goal = traj[brojac_traj]
            elif current_pos.distance(goal)< min_dist_to_goal and min_ranges(front)>1.5 and min_ranges(fleft)>1 and min_ranges(fright)>1:
                state = 0
                state_0()
            else:
                state = 2
                state_2()
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    #my_node(sys.argv[1],sys.argv[2])
    rospy.Subscriber("/odom",Odometry,odometrija)
    rospy.Subscriber("/front/scan",LaserScan,skeniranje)
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass