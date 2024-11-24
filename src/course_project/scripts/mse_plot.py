#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import time

mse_data = []
time_data = []

def mse_callback(data):
    """Callback function to process MSE data."""
    global mse_data, time_data

    # Append data for plotting
    mse_data.append(data.data)
    time_data.append(time.time())

def plot_mse():
    """Plot MSE vs. Time."""
    global mse_data, time_data

    plt.figure()
    plt.plot(time_data, mse_data, label='MSE vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('MSE')
    plt.title('Mean Squared Error Over Time')
    plt.legend()
    plt.grid()
    plt.show()

def main():
    rospy.init_node('mse_plotter', anonymous=True)
    rospy.Subscriber('/mse_error', Float64, mse_callback)
    rospy.loginfo("MSE Plotter Node Started")

    # Allow time to collect data before plotting
    rospy.spin()

    # Plot MSE after shutdown
    plot_mse()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
