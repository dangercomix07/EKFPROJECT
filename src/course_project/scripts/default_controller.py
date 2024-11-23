#!/usr/bin/env python3

from __future__ import division
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
from math import atan, atan2, pi, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from course_project.msg import Trilateration
from trilateration import varA, varB, varC, landmarkA, landmarkB, landmarkC

# Define global variables Here (pose, prev_pose, sys_input, sampling gain
# predicted_pose, estimated_pose)

# Define system noise related variables here
# Q = Process Noise
# R = Measurement noise imported from trilateration
# P = Some reasonable initial values - State Covariance
# F = System matrix for discretized unicycle is Identity
# get_current_H(pose, landmarkA, landmarkB, landmarkC)  ## H has to be calculated on the fly

# G = np.zeros((3, 2))
# I = identity matrix

FILTER_ORDER = 5  # Change filter settings
i = 0
filter_a = [0 for i in range(FILTER_ORDER)]
filter_b = [0 for i in range(FILTER_ORDER)]
filter_c = [0 for i in range(FILTER_ORDER)]

idxA = 0
idxB = 0
idxC = 0
theta = 0

odoo = Odometry()
depub = ''

estimated_angle = 0.0 
predicted_angle = 0.0

def heading_from_quaternion(x, y, z, w):
    ang_1 = 2*(w*z + x*y)
    ang_2 = 1-2*(y**2 + z**2)
    return atan2(ang_1,ang_2) % (2*pi)

def get_current_H(pose, lA, lB, lC):
    # Calculate the linearized measurement matrix H(k+1|k) at the current robot pose

    # x and y co-ordinates of landmarks    
    # current robot pose
    # Write the code for calculating Linearized H here    
    

    return 2 * np.array(H)


def sq_dist(p1, p2):
    # Given a pair of points the function returns euclidean distance
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)#**(0.5)


def predict_state(estimated_pose):
    # System evolution
    global noisy_pose, pose, input_sys, F, P, our_predicted_pose, estimated_angle, predicted_angle 
    input_sys.shape = (2, 1)
    # Write the code for predicted pose from estimated pose here    
   
    return predicted_pose 


def predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C):
    # Predicts the measurement (d1, d2, d3) given the current position of the robot
    
    return measurement


def callback2(data):
    global noisy_pose, varX, varY, varTHETA
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    noise = [np.random.normal(0, varX), np.random.normal(
        0, varY), np.random.normal(0, varTHETA)]
    noisy_pose = np.array([data.pose.pose.position.x + noise[0], data.pose.pose.position.y +
                          noise[1], euler_from_quaternion([x, y, z, w])[2] + noise[2]]).reshape(3, 1)

def callback_vicon(data):
    global noisy_pose
    pos_x = data.transform.translation.x
    pos_y = data.transform.translation.y
    orientation_q = data.transform.rotation
    heading = heading_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    noisy_pose = np.array([pos_x,pos_y, heading]).reshape(3,1)
    #print(noisy_pose,'NoisyPose')

    
def get_waypoint(t):
    global K_samp  
    # write the code to generate waypoints for the desired trajectory
    
    return [m, n]

def callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, theta, pose
    global P, Q, R, F
    global estimated_pose, noisy_pose, our_predicted_pose, estimated_angle, predicted_angle, pose_list  

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

    # Calculate filtered measurements (d1, d2, d3)
    # z vector

    # EXTENDED KALMAN FILTER CODE GOES BELOW

    # Prediction:
    # you may use the function 'predict_state()' defined above as a substitute for prediction
    # x(k+1|k)
    

    # Covariance update:
    # Use the linearized state space matrix F = A to calculate the predicted covariance matrix
    # P(k+1|k) = SIGMA
    

    # Get measurement residual:
    # difference between y_measured and y_predicted of the (k+1)^th time instant
    

    residual = Y_measured - predicted_measurement 

    # Write code for Kalman gain calculation:
    # use the linearized measurement matrix 'H(k+1|k)' to compute the kalman gain
    # W(k+1)
    
    filter_gain = np.matmul(PH_T, S_inv)
    

    # Write code to Update state with kalman gain x(k+1|k+1):
    # correct the predicted state using the measurement residual

    estimated_pose #x(k+1|k+1)

    #Define lambda and calculate estimated angle    

    x = estimated_pose[0]
    y = estimated_pose[1]
    theta = estimated_pose[2]
    # Write the code to Update covariance matrix
    # P(k+1|k+1)
    
    # Do not modify the code below
    # Send an Odometry message for visualization (refer vis.py)
    odoo.pose.pose.position.x = x
    odoo.pose.pose.position.y = y
    quaternion_val = quaternion_from_euler(0, 0, theta)
    odoo.pose.pose.orientation.x = quaternion_val[0]
    odoo.pose.pose.orientation.y = quaternion_val[1]
    odoo.pose.pose.orientation.z = quaternion_val[2]
    odoo.pose.pose.orientation.w = quaternion_val[3]
    depub.publish(odoo)

    
# This is where we will write code for trajectory following
def control_loop():
    global pose, depub, input_sys, estimated_pose, noisy_pose, our_predicted_pose, estimated_angle

    rospy.init_node('controller_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    rospy.Subscriber('/odom', Odometry, callback2)
    # rospy.Subscriber('/vicon/tb3_3/tb3_3', TransformStamped, callback_vicon)
    depub = rospy.Publisher('/odom2', Odometry, queue_size=10)

    pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)
    pubep = rospy.Publisher('/bot_0/estimatedpose', String, queue_size=10)

    # Setting the rate for loop execution
    rate = rospy.Rate(5)

    # Twist values to move the robot
    timer = 0
    #updated_pose = estimated_pose

    while not rospy.is_shutdown():

        # Write your code here for waypoint tracking


        velocity_msg = Twist() 
        velocity_msg.linear.x = 0.1
        velocity_msg.angular.z = 0.2
        input_sys[0] = velocity_msg.linear.x
        input_sys[1] = velocity_msg.angular.z
        timer = timer + 0.004 * pi

        # If robot has reached the current waypoint
        # Sample a new waypoint
        # Apply proportional control to reach the waypoint
        ####
        pub.publish(velocity_msg)       
        # pubep.publish(str([estimated_pose.tolist()[i][0] for i in range(2)])) 
        pubep.publish(str(noisy_pose.tolist())) 
        # # print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
