#!/usr/bin/env python3

from __future__ import division
import rospy
from geometry_msgs.msg import Twist, TransformStamped

from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
import numpy as np
from math import atan, atan2, pi, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from course_project.msg import Trilateration
from trilateration import varA, varB, varC, landmarkA, landmarkB, landmarkC
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# Global variables for cumulative MSE calculation
cumulative_squared_error = 0  # Sum of squared errors
iteration_count = 0  # Total number of iterations

# Initialising
#pose = np.zeros((3,1)) # Current pose (x,y,theta)
predicted_pose = np.zeros((3,1), dtype=np.float64)  # Predicted pose (x,y,theta)
estimated_pose = np.zeros((3,1), dtype=np.float64)  # Estimated pose (x,y,theta)
noisy_pose = np.zeros((3, 1), dtype=np.float64)     # Initialize noisy_pose

P = np.eye(3)  # Initial state covariance
Q = np.diag([0.01, 0.01, 0.01])  # Process noise covariance
R = np.diag([varA, varB, varC])  # Measurement noise covariance
# Measurement Matrix is calculated real time
A = np.eye(3)  # State transition matrix (for the unicycle model)

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

def predict_state(estimated_pose):
    global A, P, Q
    global input_sys, predicted_pose

    theta = estimated_pose[2, 0]

    # Control inputs
    input_sys.shape = (2, 1)
    v = input_sys[0].item()  # Linear velocity
    w = input_sys[1].item()  # Angular velocity
    
    # Time step
    dt = 0.1  # Sampling time

    # State evolution equations based on non-linear model
    dx = v * cos(theta) * dt
    dy = v * sin(theta) * dt
    dtheta = w * dt

    # Predicted pose
    predicted_pose = estimated_pose + np.array([[dx], [dy], [dtheta]])

    # State Matrix linearised at current step k
    A = np.array([
        [1, 0, -dt*v*sin(theta)],
        [0, 1,  dt*v*cos(theta)],
        [0, 0,  1]
    ], dtype=np.float64)

    # Predicted covariance matrix
    P = A @ P @ A.T + Q 
   
    return predicted_pose 

def get_current_H(pose, lA, lB, lC):
    # Calculate the linearized measurement matrix H(k+1|k) at the current robot pose
    px = pose[0, 0].item() 
    py = pose[1, 0].item()

    def compute_H(px, py, lx, ly):
        dx = px - lx
        dy = py - ly
        d = sqrt(dx**2 + dy**2)
        return np.array([dx/d, dy/d, 0],dtype=np.float64)

    # Calculate the Jacobian for each landmark
    H1 = compute_H(px, py, lA.x, lA.y)
    H2 = compute_H(px, py, lB.x, lB.y)
    H3 = compute_H(px, py, lC.x, lC.y)

    # Stack the Jacobian rows to form the full matrix H
    return np.vstack([H1, H2, H3])

def predict_measurement(predicted_pose, lA, lB, lC):
    # Predicted Measurement from non linear prediction eqn. zk = h(xk) 
    px, py = predicted_pose[0, 0], predicted_pose[1, 0]

    # Compute distances to each landmark
    d1 = sqrt((px - lA.x)**2 + (py - lA.y)**2)
    d2 = sqrt((px - lB.x)**2 + (py - lB.y)**2)
    d3 = sqrt((px - lC.x)**2 + (py - lC.y)**2)

    measurement = np.array([[d1],[d2],[d3]])
    return measurement

def update_state(predicted_pose, measurement,landmarks):
    global P,R
    lA, lB, lC = landmarks

    # Predicted Measurement from non linear prediction eqn. zk = h(xk)
    predicted_measurement = predict_measurement(predicted_pose,lA,lB,lC)
    residual = measurement - predicted_measurement

    H = get_current_H(predicted_pose,lA,lB,lC) # Linearised measurement model

    # KALMAN GAIN
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    # Update state and covariance
    estimated_pose = predicted_pose + K @ residual
    P = (np.eye(3) - K @ H) @ P
    
    return estimated_pose

def callback_odom(data):
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

def callback(data):
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global P, Q, R, A
    global predicted_pose, estimated_pose, noisy_pose

    lA = data.landmarkA
    lB = data.landmarkB
    lC = data.landmarkC

    landmarks = [lA, lB, lC]

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
    d1 = filter_a[idxA-1]
    d2 = filter_b[idxB-1]
    d3 = filter_c[idxC-1]

    # Measurement vector
    measurement = np.array([[d1],[d2],[d3]])

    # EXTENDED KALMAN FILTER
    # PREDICTION STEP
    predicted_pose = predict_state(estimated_pose)
    # UPDATE STEP
    estimated_pose = update_state(predicted_pose, measurement, landmarks)

    x = estimated_pose[0].flatten()[0] 
    y = estimated_pose[1].flatten()[0] 
    theta = estimated_pose[2].flatten()[0]  
    # Use flatten to ensure it's a scalar
    
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

def get_waypoint(t):
    # Sinusoidal trajectory
    A, B = 1.0, 1.0  # Amplitudes
    a, b = 1.0, 2.0  # Frequencies
    delta = pi      # Phase shift 

    # Calculate desired trajectory
    x = A * sin(a * t + delta)  # x-coordinate
    y = B * sin(b * t)          # y-coordinate
    return [x, y]

def calculate_mse(noisy_pose, estimated_pose):
    """Calculate Mean Squared Error (MSE) between noisy_pose and estimated_pose."""
    global cumulative_squared_error, iteration_count
    errors = (noisy_pose - estimated_pose).flatten()
    squared_error = np.sum(errors ** 2)

    cumulative_squared_error += squared_error
    iteration_count += 1
    mse = cumulative_squared_error / (iteration_count * 3)  # 3 dimensions (x, y, theta)
    return mse

# This is where we will write code for trajectory following
def control_loop():
    global pose, depub, input_sys, estimated_pose, noisy_pose

    rospy.init_node('controller_node')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('tb3_5/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.Subscriber('/vicon/tb3_5/tb3_5', TransformStamped, callback_vicon)

    depub = rospy.Publisher('/odom2', Odometry, queue_size=10)
    pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)
    pubep = rospy.Publisher('/bot_0/estimatedpose', String, queue_size=10)
    pubMSE = rospy.Publisher('/mse_error', Float64, queue_size=10)
    
    # Setting the rate for loop execution
    rate = rospy.Rate(5) # 5Hz
    # 10 Hz doesnt track properly
    timer = 0
    errorV_prev = 0
    errorTheta_prev = 0

    while not rospy.is_shutdown():

        waypoint = get_waypoint(timer)

        KpV = 0.5
        KpW = 0.4
        KdV = 0.01
        KdW = 0.01
        Vmax = 1.0
        Wmax = 0.5

        ex = waypoint[0] - estimated_pose[0, 0]
        ey = waypoint[1] - estimated_pose[1, 0]
        theta_desired = np.arctan2(ey,ex)
        etheta = theta_desired - estimated_pose[2,0]
        deW = etheta - errorTheta_prev

        # Wrap angle between -pi and pi
        if etheta> pi:
            etheta -= 2 * pi
        elif etheta < -np.pi:
            etheta += 2 * pi

        e = sqrt(ex**2 + ey**2)
        de = e - errorV_prev

        uV = np.clip(KpV*e + KdV*de,-Vmax,Vmax)
        uW = np.clip(KpW*etheta +KdW*deW,-Wmax,Wmax)

        velocity_msg = Twist() 
        velocity_msg.linear.x = uV
        velocity_msg.angular.z = uW

        input_sys[0] = velocity_msg.linear.x
        input_sys[1] = velocity_msg.angular.z

        # Calculate MSE
        mse = calculate_mse(noisy_pose, estimated_pose)

        timer = timer + 0.004 * pi

        rospy.loginfo(f"Estimated Pose: {estimated_pose.flatten()}")
        rospy.loginfo(f"Waypoint: {waypoint}")

        pub.publish(velocity_msg)
        waypoint_str = ', '.join(map(str, waypoint))  # Convert the list to a comma-separated string
        pubw.publish(waypoint_str)  # Publish the string message 
        pubep.publish(','.join(map(str, estimated_pose.flatten())))
        pubMSE.publish(mse)    

        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
