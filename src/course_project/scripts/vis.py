#!/usr/bin/env python3
from __future__ import division
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from math import cos, sin, pi
import numpy as np

import matplotlib.pyplot as plt

plt.ion()

K_samp = 0
def get_waypoint():
    return (0, 0)

TRACE_PATH = True  # False

LIM = 30
fig, ax = plt.subplots(1, 1)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_xlim([-LIM / 2, LIM / 2])
ax.set_ylim([-LIM / 2, LIM / 2])

ax.grid()
ax.legend()

# Data structures for waypoints and tracking
line_waypoints, = ax.plot([], [], 'g^', label="waypoint", ms=5)
line_ep, = ax.plot([], [], 'bx', label="estimated pose", ms=10)
line_poses, = ax.plot([], [], 'ro', label="robot", ms=15.0, alpha=0.3)
track, = ax.plot([], [], 'b:', lw=2, alpha=0.65)

# Track the robot and waypoint positions
X_track = []
Y_track = []
waypoints = []  # Properly define the waypoints list here

def pose_listener(data):
    global line_poses, X_track, Y_track
    # Extract position (not orientation) from the Odometry message
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    # Save the robot's position for plotting
    X_track.append(x)
    Y_track.append(y)

    # Update the plot data for robot position
    line_poses.set_data(X_track, Y_track)

def waypoint_listener(data):
    global line_waypoints, waypoints
    # Assuming data is a String that contains the waypoint in the form "x,y"
    waypoint = data.data.split(",")  # Split the string into x and y
    try:
        x, y = float(waypoint[0]), float(waypoint[1])
        waypoints.append((x, y))  # Append new waypoint to the list
    except ValueError:
        rospy.logwarn(f"Invalid waypoint data: {data.data}")
        return

    # Update the plot data for waypoints
    if waypoints:
        waypoint_x, waypoint_y = zip(*waypoints)  # Extract x and y coordinates
        line_waypoints.set_data(waypoint_x, waypoint_y)

def ep_listener(data):
    global line_ep
    # Assuming data is a String that contains the estimated pose as a 2D list (e.g., '[[x], [y], [theta]]')
    try:
        estimated_pose = eval(data.data)  # Use eval to convert the string representation of a list to actual list
        
        # Access the individual elements in the 2D list
        x = float(estimated_pose[0][0])  # First element in the first list (x)
        y = float(estimated_pose[1][0])  # First element in the second list (y)
    except Exception as e:
        rospy.logwarn(f"Invalid estimated pose data: {data.data} Error: {e}")
        return

    # Update the plot for the estimated pose
    line_ep.set_data(x, y)


def process():
    rospy.init_node('plotting_node', anonymous=True)
    rospy.Subscriber('/odom2', Odometry, pose_listener)
    rospy.Subscriber('/bot_0/waypoint', String, waypoint_listener)
    rospy.Subscriber('/bot_0/estimatedpose', String, ep_listener)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Redraw the canvas to update the plot
        fig.canvas.draw()
        fig.canvas.flush_events()

        # Sleep to maintain the rate
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Process started ")
        process()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Save the plot to a file when finished
        plt.xlabel('X (in meters)')
        plt.ylabel('Y (in meters)')
        plt.title('Robot Trajectory')
        plt.savefig('Robot_Trajectory.png')
