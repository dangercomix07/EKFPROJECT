#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from math import sin, pi

# Use 'Agg' backend for better stability with ROS
import matplotlib
matplotlib.use('Agg')

# Static Lissajous curve parameters
A, B = 1.0, 1.0  # Amplitudes
a, b = 1.0, 2.0  # Frequencies
delta = 0  # Phase shift

# Axes setup for the plot
LIM = 3
fig, ax = plt.subplots(1, 1, figsize=(8, 8))
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_xlim([-LIM / 2, LIM / 2])
ax.set_ylim([-LIM / 2, LIM / 2])

# Static Lissajous curve for the desired trajectory
t = np.linspace(0, 2 * pi, 1000)
lissajous_x = A * np.sin(a * t + delta)
lissajous_y = B * np.sin(b * t)
ax.plot(lissajous_x, lissajous_y, label='Lissajous curve', color='blue', lw=2)

# Initialize plot objects
line_waypoints, = ax.plot([], [], 'g^', label="Waypoint", ms=5)  # Green for waypoints
line_poses, = ax.plot([], [], 'ro', label="Robot Path", ms=10, alpha=0.7)  # Red for actual robot pose
line_ep, = ax.plot([], [], 'bx', label="Estimated Pose", ms=10)  # Blue for estimated pose
line_poses2, = ax.plot([], [], 'r', lw=2, alpha=0.5, label="Path")  # Path of the robot

# Add grid and legend
ax.grid()
ax.legend()

# Data containers for tracking
X_track = []
Y_track = []
waypoints_x = []
waypoints_y = []
estimated_x = []
estimated_y = []

def pose_listener(data):
    """Callback to update robot pose."""
    global X_track, Y_track
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    X_track.append(x)
    Y_track.append(y)

def waypoint_listener(data):
    """Callback to update waypoints."""
    global waypoints_x, waypoints_y
    waypoint = data.data.split(",")
    x, y = float(waypoint[0]), float(waypoint[1])
    waypoints_x.append(x)
    waypoints_y.append(y)

def ep_listener(data):
    """Callback to update the estimated pose."""
    global estimated_x, estimated_y
    try:
        estimated_pose = data.data.strip('[]').split(',')
        x, y, _ = float(estimated_pose[0]), float(estimated_pose[1]), float(estimated_pose[2])
        estimated_x.append(x)
        estimated_y.append(y)
    except Exception as e:
        rospy.logwarn(f"Error parsing estimated pose: {e}")

def update_plot():
    """Update plot with new data."""
    # Update robot path
    line_poses.set_data(X_track, Y_track)
    line_poses2.set_data(X_track, Y_track)

    # Update waypoints
    line_waypoints.set_data(waypoints_x, waypoints_y)

    # Update estimated pose
    if estimated_x and estimated_y:
        line_ep.set_data(estimated_x[-1], estimated_y[-1])  # Only show the latest estimated pose

    # Save the plot to a file for visualization
    plt.savefig('/home/ameya/EKFPROJECT/src/course_project/scripts/Robot_Trajectory.png')

def process():
    """ROS node for real-time visualization."""
    rospy.init_node('plotting_node', anonymous=True)
    rospy.Subscriber('/odom2', Odometry, pose_listener)
    rospy.Subscriber('/bot_0/waypoint', String, waypoint_listener)
    rospy.Subscriber('/bot_0/estimatedpose', String, ep_listener)

    rate = rospy.Rate(10)  # Update at 10 Hz
    while not rospy.is_shutdown():
        update_plot()  # Update the plot periodically
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Process started")
        process()
    except rospy.ROSInterruptException:
        pass
