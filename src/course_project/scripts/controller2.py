#!/usr/bin/env python3

from __future__ import division
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
from math import atan, atan2, pi, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from course_project.msg import Trilateration


# Define global variables Here (pose, prev_pose, sys_input, sampling gain
# predicted_pose, estimated_pose)

# Define system noise related variables here
q = np.array([0.1, 0.1, 0.1], dtype=np.float64)
Q = np.diag(q) # Q = Process Noise
r = np.array([1.0, 1.0, 1.0], dtype=np.float64)
R = np.diag(r) # R = Measurement noise imported from trilateration
p = np.array([0.5**2, 0.5**2, 0.1**2], dtype=np.float64)
P = np.diag(p) # P = Some reasonable initial values - State Covariance
# F = System matrix for discretized unicycle is Identity

# G = np.zeros((3, 2))
# I = identity matrix

FILTER_ORDER = 5  # Change filter settings
i = 0
filter_a = [0 for i in range(FILTER_ORDER)]
filter_b = [0 for i in range(FILTER_ORDER)]
filter_c = [0 for i in range(FILTER_ORDER)]
input_sys = np.array([[0.0],
                     [0.0]], dtype=np.float64)
noisy_pose = np.array([0.0,0.0,0.0], dtype=np.float64).reshape((3,1))
KSAMP = 0.03

idxA = 0
idxB = 0
idxC = 0

varX = 0.1
varY = 0.1
varTHETA = 0.1

odoo = Odometry()
depub = ''

estimated_pose = np.array([0.0,0.0,0.0], dtype=np.float64).reshape((3,1))
input_sys

estimated_angle = 0.0 
predicted_angle = 0.0

def heading_from_quaternion(x, y, z, w): #done
    ang_1 = 2*(w*z + x*y)
    ang_2 = 1-2*(y**2 + z**2)
    return atan2(ang_1,ang_2) % (2*pi)


def get_current_H(pose, lA, lB, lC): #done
    x1, y1 = lA.x, lA.y
    x2, y2 = lB.x, lB.y
    x3, y3 = lC.x, lC.y
    x, y = pose[0,0], pose[1,0]
    r1, r2, r3 = lA.distance, lB.distance, lC.distance
    H = [
        [(x-x1)/r1, (y-y1)/r1, 0],
        [(x-x2)/r2, (y-y2)/r2, 0],
        [(x-x3)/r3, (y-y3)/r3, 0]
    ]
    return np.array(H, dtype=np.float64)


def euclidean_dist(p1, p2): # done
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(0.5)


def predict_state(estimated_pose): # done
    # System evolution
    global noisy_pose, pose, input_sys, F, P, estimated_angle, predicted_angle 
    F = np.array([
        [1, 0, -input_sys[0, 0] * np.sin(estimated_pose[2, 0])],
        [0, 1,  input_sys[0, 0] * np.cos(estimated_pose[2, 0])],
        [0, 0,  1]
    ], dtype=np.float64)
    # Predicted state: x(k+1|k) = F * x(k|k) + B * u
    # Control input (u) includes velocity and angular velocity
    B = np.array([
        [np.cos(estimated_pose[2, 0]), 0],
        [np.sin(estimated_pose[2, 0]), 0],
        [0, 1]
    ], dtype=np.float64)

    predicted_pose = F @ estimated_pose + B @ input_sys
    return predicted_pose


def predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C): # done
    x, y, _ = predicted_pose

    x1, y1 = landmark_A.x, landmark_A.y
    x2, y2 = landmark_B.x, landmark_B.y
    x3, y3 = landmark_C.x, landmark_C.y
    
    # Calculate distances
    d1 = ((x - x1)**2 + (y - y1)**2)**0.5
    d2 = ((x - x2)**2 + (y - y2)**2)**0.5
    d3 = ((x - x3)**2 + (y - y3)**2)**0.5
    
    # Return the distances as a numpy array
    return np.array([d1, d2, d3], dtype=np.float64)


def odom_callback(data): #done
    global noisy_pose, varX, varY, varTHETA
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    noise = [np.random.normal(0, varX), np.random.normal(
        0, varY), np.random.normal(0, varTHETA)]
    noise = [0,0,0]
    bot_viz = Marker()
    bot_viz.header.frame_id = "base_link"
    bot_viz.type = Marker.ARROW
    bot_viz.action = Marker.ADD
    bot_viz.scale.x = 0.5  # Arrow shaft diameter
    bot_viz.scale.y = 0.1  # Arrow head diameter
    bot_viz.scale.z = 0.1  # Arrow head length
    bot_viz.color.a = 1.0  # Opacity
    bot_viz.color.r = 1.0
    bot_viz.color.g = 0.0
    bot_viz.color.b = 0.0
    bot_viz.pose.position = data.pose.pose.position
    bot_viz.pose.orientation = data.pose.pose.orientation
    bot_viz_pub.publish(bot_viz)
    noisy_pose = np.array([data.pose.pose.position.x + noise[0], data.pose.pose.position.y +
                          noise[1], euler_from_quaternion([x, y, z, w])[2] + noise[2]], dtype=np.float64).reshape(3, 1)


def callback_vicon(data): #done
    global noisy_pose
    pos_x = data.transform.translation.x
    pos_y = data.transform.translation.y
    orientation_q = data.transform.rotation
    heading = heading_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    noisy_pose = np.array([pos_x,pos_y, heading], dtype=np.float64).reshape(3,1)
    #print(noisy_pose,'NoisyPose')

    
def get_waypoint(): #done
    global waypoint_pub 
    t = (rospy.Time.now().to_sec() - start_time)*KSAMP
    A = 5           # Amplitude for x-coordinate trajectory
    B = 4           # Amplitude for y-coordinate trajectory
    a = 3           # Frequency component for x-coordinate
    b = 2           # Frequency component for y-coordinate
    delta = 0.5     # Phase shift for x-coordinate
    m = A * np.sin(a * t + delta)
    n = B * np.sin(b * t)

    goal_viz = Marker()
    goal_viz.header.frame_id = "base_link"
    goal_viz.pose.orientation.w = 1.0
    goal_viz.scale.x = 0.1  # Arrow shaft diameter
    goal_viz.scale.y = 0.1  # Arrow head diameter
    goal_viz.scale.z = 0.1  # Arrow head length
    goal_viz.pose.position.x = m
    goal_viz.pose.position.y = n
    goal_viz.color.a = 1.0  # Opacity
    goal_viz.color.r = 0.0
    goal_viz.color.g = 1.0
    goal_viz.color.b = 0.0
    waypoint_pub.publish(goal_viz)
    return [m, n]


def trilateration_callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, pose
    global P, Q, R, F
    global estimated_pose, noisy_pose, estimated_angle, predicted_angle

    lA = data.landmarkA 
    lB = data.landmarkB
    lC = data.landmarkC

    #######################################################
    # FILTERING VALUES
    #######################################################
    # Add value into r buffer at indices idxA, idxB, idxC
    filter_a[idxA] = lA.distance
    filter_b[idxB] = lB.distance
    filter_c[idxC] = lC.distance
    # Increment indexes
    idxA += 1
    idxB += 1
    idxC += 1

    # wrap around the indices if buffer full
    if idxA >= FILTER_ORDER:
        idxA = 0
    if idxB >= FILTER_ORDER:
        idxB = 0
    if idxC >= FILTER_ORDER:
        idxC = 0

    d1 = sum(filter_a)/FILTER_ORDER
    d2 = sum(filter_b)/FILTER_ORDER
    d3 = sum(filter_c)/FILTER_ORDER
    Y_measured = np.array([d1, d2, d3]).reshape(3,1)

    # EXTENDED KALMAN FILTER CODE GOES BELOW

    predicted_state = predict_state(estimated_pose) # x(k+1∣k)
    # todo: recheck the motion model
    

    # Covariance update:
    # Use the linearized state space matrix F = A to calculate the predicted covariance matrix
    # P(k+1|k) = SIGMA
    predicted_covariance = F @ P @ F.T + Q # P(k+1∣k)
    

    # Get measurement residual:
    # difference between y_measured and y_predicted of the (k+1)^th time instant
    
    predicted_measurement = predict_measurement(predicted_state, lA, lB, lC) # y_predicted(k+1)
    measurement_residual = Y_measured - predicted_measurement # yresidual(k+1)
    # print(Y_measured.shape, predicted_measurement.shape)
    # Write code for Kalman gain calculation:
    # use the linearized measurement matrix 'H(k+1|k)' to compute the kalman gain
    # W(k+1)
    H = get_current_H(estimated_pose, lA, lB, lC) # H(k+1∣k)
    S = np.dot(np.dot(H, predicted_covariance), H.T) + R 
    S = np.array(S, dtype=np.float64)
    # Compute the Kalman gain: W = P_predicted * H.T * S^(-1)
    S_inv = np.linalg.inv(S)  # Inverse of the innovation covariance
    W = np.dot(np.dot(predicted_covariance, H.T), S_inv) # W(k+1)
    

    # Write code to Update state with kalman gain x(k+1|k+1):
    # correct the predicted state using the measurement residual

    estimated_pose = predicted_state + np.dot(W, measurement_residual) # x(k+1|k+1)
    print(estimated_pose)
    #Define lambda and calculate estimated angle    


    # Write the code to Update covariance matrix
    I = np.eye(predicted_covariance.shape[0])
    P = np.dot(np.dot(I - np.dot(W, H), predicted_covariance), (I - np.dot(W, H)).T) + np.dot(np.dot(W, R), W.T) # P(k+1|k+1)

    # Do not modify the code below
    # Send an Odometry message for visualization (refer vis.py)
    x = estimated_pose[0,0]
    y = estimated_pose[1,0]
    theta = estimated_pose[2,0]
    # print(estimated_pose)
    # print(round(x,2), round(y,2), round(theta, 2))
    bot_viz = Marker()
    bot_viz.header.frame_id = "base_link"
    bot_viz.type = Marker.ARROW
    bot_viz.action = Marker.ADD
    bot_viz.scale.x = 0.5  # Arrow shaft diameter
    bot_viz.scale.y = 0.1  # Arrow head diameter
    bot_viz.scale.z = 0.1  # Arrow head length
    bot_viz.color.a = 1.0  # Opacity
    bot_viz.color.r = 0.0
    bot_viz.color.g = 1.0
    bot_viz.color.b = 0.0
    bot_viz.pose.position.x = x
    bot_viz.pose.position.y = y
    # print(theta, type(theta))
    quaternion_val = quaternion_from_euler(0, 0, theta, axes='sxyz')
    bot_viz.pose.orientation.x = quaternion_val[0]
    bot_viz.pose.orientation.y = quaternion_val[1]
    bot_viz.pose.orientation.z = quaternion_val[2]
    bot_viz.pose.orientation.w = quaternion_val[3]
    bot_est_pub.publish(bot_viz)

    odoo.pose.pose.position.x = x
    odoo.pose.pose.position.y = y
    
    odoo.pose.pose.orientation.x = quaternion_val[0]
    odoo.pose.pose.orientation.y = quaternion_val[1]
    odoo.pose.pose.orientation.z = quaternion_val[2]
    odoo.pose.pose.orientation.w = quaternion_val[3]
    depub.publish(odoo)


def get_controls(pose, waypoint):
    current_pose = noisy_pose
    # Extract waypoint (desired) coordinates
    waypoint_x, waypoint_y = waypoint

    # Calculate the distance to the waypoint (Euclidean distance)
    dx = waypoint_x - current_pose[0,0]
    dy = waypoint_y - current_pose[1,0]
    distance_to_waypoint = np.sqrt(dx**2 + dy**2)

    # Calculate the heading (desired angle) to the waypoint
    desired_angle = np.arctan2(dy, dx)

    # Proportional control to move towards the waypoint
    # Calculate the error in angle (heading)
    angle_error = desired_angle - current_pose[2,0]

    # Normalize the angle to be between -pi and pi
    if angle_error > np.pi:
        angle_error -= 2 * np.pi
    elif angle_error < -np.pi:
        angle_error += 2 * np.pi

    # Control parameters for linear and angular velocities
    Kp_linear = 0.25  # Linear velocity gain
    Kp_angular = 0.4  # Angular velocity gain

    # Proportional control for linear and angular velocities
    linear_velocity = Kp_linear * distance_to_waypoint
    angular_velocity = Kp_angular * angle_error
    # print(linear_velocity, angular_velocity)
    # Cap linear velocity to a maximum value to prevent high-speed motion
    max_linear_velocity = 1.0
    linear_velocity = np.clip(linear_velocity, -max_linear_velocity, max_linear_velocity)
    max_angular_velocity = 0.5
    angular_velocity = np.clip(angular_velocity, -max_angular_velocity, max_angular_velocity)

    # If close to the waypoint, stop the robot
    # if distance_to_waypoint < 0.1:
    #     linear_velocity = 0
    #     angular_velocity = 0
    
    return (linear_velocity, angular_velocity)


def control_loop():
    global depub, input_sys, estimated_pose, noisy_pose, estimated_angle, start_time, waypoint_pub, bot_viz_pub,bot_est_pub

    rospy.init_node('controller_node')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    waypoint_pub = rospy.Publisher('/goal/viz', Marker, queue_size=10)
    bot_viz_pub = rospy.Publisher('/bot/viz', Marker, queue_size=10)
    bot_est_pub = rospy.Publisher('/bot/est', Marker, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, trilateration_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    # rospy.Subscriber('/vicon/tb3_3/tb3_3', TransformStamped, callback_vicon)
    depub = rospy.Publisher('/odom2', Odometry, queue_size=10) # estimated pose for vis.py

    # pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)
    # pubep = rospy.Publisher('/bot_0/estimatedpose', String, queue_size=10)

    
    rate = rospy.Rate(5) # Setting the rate for loop execution

    # Twist values to move the robot
    timer = 0
    start_time = rospy.Time.now().to_sec()
    #updated_pose = estimated_pose

    while not rospy.is_shutdown():
        waypoint = get_waypoint()
        v, w = get_controls(estimated_pose, waypoint)
        # print(v,w)
        velocity_msg = Twist() 
        velocity_msg.linear.x = v
        velocity_msg.angular.z = w
        input_sys[0,0] = velocity_msg.linear.x
        input_sys[1,0] = velocity_msg.angular.z
        timer = timer + 0.004 * pi

        # If robot has reached the current waypoint
        # Sample a new waypoint
        # Apply proportional control to reach the waypoint

        cmd_pub.publish(velocity_msg)       
        # pubep.publish(str([estimated_pose.tolist()[i][0] for i in range(2)])) 
        # pubep.publish(str(estimated_pose.tolist())) 
        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
