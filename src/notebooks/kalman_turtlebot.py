#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from numpy.linalg import inv

class KalmanFilter:
    def __init__(self):
        rospy.init_node('kalman_filter', anonymous=True)

        self.prev_time = rospy.Time.now()
        self.curr_time = rospy.Time.now()
        
        self.dt = 0.0
        self.dt_imu = 0.0

        # State vector [x, y, theta, vx, vy, vtheta]
        self.X = np.zeros((6, 1))

        # State covariance matrix
        self.P = np.eye(6)

        # Control input matrix
        self.B = np.eye(6)

        # Measurement matrix
        self.H = np.eye(6)

        # Measurement covariance matrix
        self.R = np.eye(6)

        # Process noise covariance matrix
        self.Q = np.eye(6)

        # Subscriber for odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscriber for IMU data
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Publisher for filtered odometry data
        self.odom_pub = rospy.Publisher('/odom_filtered', Odometry, queue_size=10)

    def predict(self):
        # Compute time difference
        self.curr_time = rospy.Time.now()
        self.dt = (self.curr_time - self.prev_time).to_sec()

        # Prediction step
        self.X = np.dot(self.X, self.dt)  # Apply motion model
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, z):
        # Update step
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), inv(S))

        self.X = self.X + np.dot(K, (z - np.dot(self.H, self.X)))
        self.P = np.dot((np.eye(6) - np.dot(K, self.H)), self.P)

    def odom_callback(self, msg):
        z = np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y],
                      [msg.pose.pose.orientation.z],
                      [msg.twist.twist.linear.x],
                      [msg.twist.twist.linear.y],
                      [msg.twist.twist.angular.z]])

        self.predict()
        self.update(z)

        filtered_odom = Odometry()
        filtered_odom.header = msg.header
        filtered_odom.pose.pose.position.x = self.X[0, 0]
        filtered_odom.pose.pose.position.y = self.X[1, 0]
        filtered_odom.pose.pose.orientation.z = self.X[2, 0]
        filtered_odom.twist.twist.linear.x = self.X[3, 0]
        filtered_odom.twist.twist.linear.y = self.X[4, 0]
        filtered_odom.twist.twist.angular.z = self.X[5, 0]

        self.odom_pub.publish(filtered_odom)

    def imu_callback(self, msg):
        self.dt_imu = (rospy.Time.now() - self.curr_time).to_sec()

        if self.dt_imu != 0.0:
            self.A = np.array([[1, 0, 0, self.dt_imu, 0, 0],
                               [0, 1, 0, 0, self.dt_imu, 0],
                               [0, 0, 1, 0, 0, self.dt_imu],
                               [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 1]])


if __name__ == '__main__':
    try:
        kf = KalmanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
