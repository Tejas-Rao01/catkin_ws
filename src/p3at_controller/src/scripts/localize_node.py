#!/usr/bin/env python3
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import wall_localization
import time 

#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x=[]
robot_position_y=[]
robot_orientation =[]

#defining two axes and figures for plotting 
fig , ax1 = plt.subplots()

#plot robot

pts = ax1.scatter(np.empty((1147,1)),np.empty((1147,1)),c='r')
X_cor = [0.0, 1.0]
Y_cor = [0.0, 0.0]
heading_1, = ax1.plot(X_cor, Y_cor,'b-') # plotting the intial heading from start to another point 
lines = ax1.plot(np.empty((0, 100)), np.empty((0, 100)),color='black', lw=2)
ax1.set(xlim=(-8,8), ylim=(-8,8))
traj = ax1.plot([0, 0], [0, 0])
robotX = -1.5
robotY = 0 
robotTheta = 0
unrobotX = robotX
unrobotY = 0
unrobotTheta = 0 

wallx = []
wally = [] 

trajectory_x = []
trajectory_y = []

P = np.zeros(shape=(3,3))

def plot_data2Str(robotX, robotY, robotTheta, P, walls):
    plot_vars = f'{robotX} {robotY} {robotTheta} \n'
    P_str = ''
    for i in range(3):
        for j in range(3):
            P_str += str(P[i, j]) + ' '
    plot_vars += P_str
    plot_vars += '\n'
    for wall in walls:
        p1, p2 = wall
        p_str = f'{p1[0]} {p1[1]} {p2[0]} {p2[1]} '
        plot_vars += p_str
        plot_vars += '\n'
    return plot_vars
    


def pose_callback(data):
    print("callback")
    #acessing the required global variables 
    global robot_position_x
    global robot_position_y
    global ax1
    global robotX
    global robotY
    global robotTheta
    global unrobotX
    global unrobotY
    global unrobotTheta
    global odometry_data
    global P
    global heading_1 
    global lines
    global pts
    #finding th erequired points to be plotted 
    X = []
    Y = []
    odometry_data =[0,0,0]
    P = np.eye(3)
    lidar_data = data.ranges
    step_size = data.angle_increment
    
    print('robotx before', robotX)
    robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P, plot_vars=  wall_localization.localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P)
    print('robotX after', robotX)

    if plot_vars !=[]:
        print("publishing")
        X1,robotX, robotY, robotTheta, walls = plot_vars
        plot_data = plot_data2Str(robotX, robotY, robotTheta, P, walls)
        pub.publish(plot_data)
        
    else:
    	print("publishing without localizing")
    	
    	
    	
    	
            
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)
    
    pub = rospy.Publisher("plot_data", String, queue_size=1)
    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/scan", LaserScan, pose_callback,queue_size=1)
    
    plt.show(block=True)
    rospy.spin()
