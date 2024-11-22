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


pose = np.zeros((3,1)) # Current pose (x,y,theta)
predicted_pose = np.zeros((3,1)) # Predicted pose (x,y,theta)
estimated_pose = np.zeros((3,1)) # Estimated pose (x,y,theta)
noisy_pose = np.zeros((3, 1))  # Initialize noisy_pose

P = np.eye(3)  # Initial state covariance
Q = np.diag([0.01, 0.01, 0.01])  # Process noise covariance
R = np.diag([varA, varB, varC])  # Measurement noise covariance
# Measurement Matrix is calculated real time
F = np.eye(3)  # State transition matrix (for the unicycle model)

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

input_sys = np.zeros((2, 1))  # Control inputs: [linear velocity, angular velocity]

varX = 0.1  # Standard deviation of noise in the x-direction
varY = 0.1  # Standard deviation of noise in the y-direction
varTHETA = 0.05  # Standard deviation of noise in the theta (orientation) direction

def heading_from_quaternion(x, y, z, w):
    ang_1 = 2*(w*z + x*y)
    ang_2 = 1-2*(y**2 + z**2)
    return atan2(ang_1,ang_2) % (2*pi)

def get_current_H(pose, lA, lB, lC):
    # Calculate the linearized measurement matrix H(k+1|k) at the current robot pose

    # x and y co-ordinates of landmarks    
    # current robot pose
    # Write the code for calculating Linearized H here    
    #px, py = pose[0, 0], pose[1, 0]

    # Extract scalar values from pose using .item()
    px = pose[0, 0].item()  # Convert numpy array to scalar
    py = pose[1, 0].item()  # Convert numpy array to scalar

    def compute_H(px, py, lx, ly):
        # Ensure that px, py, lx, ly are scalars
        #print(f"px: {px}, py: {py}, lx: {lx}, ly: {ly}")  # Debug print
        dx = px - lx
        dy = py - ly
        d = sqrt(dx**2 + dy**2)

        #rospy.loginfo(f"dx type: {type(dx)}, dy type: {type(dy)}, d type: {type(d)}")

        return np.array([dx / d, dy / d, 0])

    # Calculate the Jacobian for each landmark
    H1 = compute_H(px, py, lA[0], lA[1])
    H2 = compute_H(px, py, lB[0], lB[1])
    H3 = compute_H(px, py, lC[0], lC[1])

    # Debug prints to check the shapes
    #rospy.loginfo(f"Shape of H1: {H1.shape}, H2: {H2.shape}, H3: {H3.shape}")

    # Stack the Jacobian rows to form the full matrix H
    return np.vstack([H1, H2, H3])    
    #return 2 * np.array(H)


def sq_dist(p1, p2):
    # Given a pair of points the function returns euclidean distance
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(0.5)


def predict_state(estimated_pose):
    global P, F, Q
    # System evolution
    global noisy_pose, pose, input_sys, F, P, predicted_pose
    input_sys.shape = (2, 1)
    # Write the code for predicted pose from estimated pose here

    # Extract control inputs
    v = input_sys[0]  # Linear velocity
    w = input_sys[1]  # Angular velocity
    theta = estimated_pose[2, 0]

    # Time step
    dt = 0.1  # Sampling time

    # State evolution equations
    dx = v * cos(theta) * dt
    dy = v * sin(theta) * dt
    dtheta = w * dt

    # Update the predicted pose
    predicted_pose = estimated_pose + np.array([[dx], [dy], [dtheta]])

    # Update covariance matrix
    P = F @ P @ F.T + Q 
   
    return predicted_pose 


def predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C):
    # Predicts the measurement (d1, d2, d3) given the current position of the robot
    px, py = predicted_pose[0, 0], predicted_pose[1, 0]

    # Compute distances to each landmark
    d1 = sqrt((px - landmark_A[0])**2 + (py - landmark_A[1])**2)
    d2 = sqrt((px - landmark_B[0])**2 + (py - landmark_B[1])**2)
    d3 = sqrt((px - landmark_C[0])**2 + (py - landmark_C[1])**2)

    measurement = np.array([[d1],[d2],[d3]])
    return measurement

def update_state(predicted_pose, measurement, landmarks):
    global P,R
    # Extract landmarks
    landmark_A, landmark_B, landmark_C = landmarks

    # Compute H matrix (Jacobian of measurement function)
    H = get_current_H(predicted_pose, landmark_A, landmark_B, landmark_C)

    # Predicted measurement
    predicted_measurement = predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C)

    # Residual (difference between actual and predicted measurements)
    residual = measurement - predicted_measurement

    # Kalman gain
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    # Update state and covariance
    estimated_pose = predicted_pose + K @ residual
    P = (np.eye(3) - K @ H) @ P

    return estimated_pose

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
    # Sinusoidal trajectory
    A, B = 1.0, 1.0  # Amplitudes
    a, b = 1.0, 1.0  # Frequencies
    delta = 0.0      # Phase shift 

    # Calculate desired trajectory
    m = A * sin(a * t + delta)  # x-coordinate
    n = B * sin(b * t)          # y-coordinate
    return [m, n]


def callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, theta, pose
    global P, Q, R, F
    global estimated_pose, noisy_pose, predicted_pose, estimated_angle, predicted_angle, pose_list  

    lA = data.landmarkA
    lB = data.landmarkB
    lC = data.landmarkC

    landmarks = [landmarkA, landmarkB, landmarkC]

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
    # Measurement vector
    measurement = np.array([[filter_a[idxA-1]], 
                            [filter_b[idxB-1]], 
                            [filter_c[idxC-1]]])


    # EXTENDED KALMAN FILTER CODE GOES BELOW

    # Prediction:
    # you may use the function 'predict_state()' defined above as a substitute for prediction
    # x(k+1|k) 
    # P(k+1|k) = SIGMA

    predicted_pose = predict_state(estimated_pose)

    # UPDATE STEP
    # Write code to Update state with kalman gain x(k+1|k+1):
    # correct the predicted state using the measurement residual

    #x(k+1|k+1)
    estimated_pose = update_state(predicted_pose, measurement, landmarks)

    #Define lambda and calculate estimated angle    

    x = estimated_pose[0].flatten()[0] 
    y = estimated_pose[1].flatten()[0] 
    theta = estimated_pose[2].flatten()[0]  # Use flatten to ensure it's a scalar
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
    rate = rospy.Rate(10) # 5Hz

    # Twist values to move the robot
    timer = 0
    #updated_pose = estimated_pose

    while not rospy.is_shutdown():

        # Write your code here for waypoint tracking
        # Generate waypoint
        waypoint = get_waypoint(timer)

        #Control to reach waypoint
        velocity_msg = Twist()
        error_x = waypoint[0] - estimated_pose[0, 0]
        error_y = waypoint[1] - estimated_pose[1, 0]

        Kp = 0.1

        velocity_msg.linear.x = Kp * sqrt(error_x**2 + error_y**2)
        velocity_msg.angular.z = Kp * atan2(error_y, error_x)
        
        input_sys[0] = velocity_msg.linear.x
        input_sys[1] = velocity_msg.angular.z

        timer = timer + 0.004 * pi
        #timer += 1 / rate.sleep_dur.to_sec()

        rospy.loginfo(f"Estimated Pose: {estimated_pose.flatten()}")
        #rospy.loginfo(f"Measurement: {measurement.flatten()}")
        rospy.loginfo(f"Waypoint: {waypoint}")


        # # If robot has reached the current waypoint
        # # Sample a new waypoint
        # # Apply proportional control to reach the waypoint
        # # Threshold for reaching waypoint
        # proximity_threshold = 0.1  # Adjust as needed

        # distance_to_waypoint = sqrt(error_x**2 + error_y**2)
        # if distance_to_waypoint < proximity_threshold:
        #     rospy.loginfo("Waypoint reached!")
        #     timer = timer + 0.004 * pi  # Increment timer to sample the next waypoint
        ####
        pub.publish(velocity_msg)       
        # pubep.publish(str([estimated_pose.tolist()[i][0] for i in range(2)])) 
        pubep.publish(str(noisy_pose.tolist()))
        waypoint_str = ', '.join(map(str, waypoint))  # Convert the list to a comma-separated string
        pubw.publish(waypoint_str)  # Publish the string message
        # # print("Controller message pushed at {}".format(rospy.get_time()))

        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
