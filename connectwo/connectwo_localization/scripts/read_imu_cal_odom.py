#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import numpy as np
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Initialize global variables
x = 0.0
y = 0.0
th = 0.0
vx = 0.0
vy = 0.0
vth = 0.0

# Scale factors for IMU data
acc_scale = 9.81 / 16384.0  # assuming the accelerometer provides data in g's
alpha = 0.5  # Low-pass filter smoothing factor
gyro_scale = pi / (180.0 * 131.0)  # assuming the gyro provides data in degrees/s

# Extended Kalman Filter (EKF) variables
P = np.eye(3) * 0.1  # Initial covariance matrix
Q = np.eye(3) * 0.01  # Process noise covariance matrix
R = np.eye(3) * 0.1  # Measurement noise covariance matrix

# Flag to track new measurements
new_measurement_available = False

def imu_callback(data):
    global vx, vy, vth, new_measurement_available
    try:
        # Update velocity based on IMU data
        vx = alpha * vx + (1 - alpha) * (data.linear_acceleration.x * acc_scale)
        vy = alpha * vy + (1 - alpha) * (data.linear_acceleration.y * acc_scale)
        vth = alpha * vth + (1 - alpha) * (data.angular_velocity.z * gyro_scale)

        #rospy.loginfo("IMU callback - vx: {}, vy: {}, vth: {}".format(vx, vy, vth))

        # Set flag indicating new measurement available
        new_measurement_available = True
        
    except Exception as e:
        rospy.logerr("Error in imu_callback: {}".format(e))

def odom_callback(data):
    global vth
    try:
        # Update angular velocity based on odom data
        vth = data.twist.twist.angular.z
        #rospy.loginfo("Odom callback - vth: {}".format(vth))
    except Exception as e:
        rospy.logerr("Error in odom_callback: {}".format(e))

def ekf_prediction(dt):
    global x, y, th, vx, vy, vth, P
    # State transition matrix
    F = np.array([
        [1, 0, -vx * sin(th) * dt],
        [0, 1, vx * cos(th) * dt],
        [0, 0, 1]
    ])
    # Control input matrix
    B = np.array([
        [cos(th) * dt, 0],
        [sin(th) * dt, 0],
        [0, dt]
    ])
    u = np.array([vx, vth])
    # Predict state
    x += vx * cos(th) * dt
    y += vx * sin(th) * dt
    th += vth * dt
    # Predict covariance
    P = np.dot(np.dot(F, P), F.T) + Q

def ekf_update(z):
    global x, y, th, P, new_measurement_available
    if new_measurement_available:
        try:
            # Measurement matrix
            H = np.eye(3)
            # Kalman gain
            S = np.dot(np.dot(H, P), H.T) + R
            K = np.dot(np.dot(P, H.T), np.linalg.inv(S))
            # Update state
            y_kalman = z - np.dot(H, np.array([x, y, th]))
            x += K[0, 0] * y_kalman[0]
            y += K[1, 1] * y_kalman[1]
            th += K[2, 2] * y_kalman[2]
            # Update covariance
            P = np.dot((np.eye(3) - np.dot(K, H)), P)
            
            # Reset measurement flag
            new_measurement_available = False
            
        except Exception as e:
            rospy.logerr("Error in ekf_update: {}".format(e))

def odom_publisher():
    global x, y, th, vx, vy, vth
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.init_node('odom_publisher', anonymous=True)
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber('/raw_odom', Odometry, odom_callback)
    rate = rospy.Rate(30)  # 30Hz

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        # EKF prediction step
        ekf_prediction(dt)

        # EKF update step with new measurement
        if new_measurement_available:
            z = np.array([x, y, th])  # Replace with actual measurements if available
            ekf_update(z)

        # Create quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # Publish the transform over tf
        # odom_broadcaster.sendTransform(
        #     (x, y, 0.),
        #     odom_quat,
        #     current_time,
        #     "base_footprint",
        #     "odom"
        # )

        # Publish the odometry message over ROS
        cal_odom = Odometry()
        cal_odom.header.stamp = current_time
        cal_odom.header.frame_id = "odom"

        # set the position
        cal_odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        cal_odom.child_frame_id = "base_footprint"
        # set the velocity
        cal_odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        pub.publish(cal_odom)

        last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
    except Exception as e:
        rospy.logerr("Unhandled Exception: {}".format(e))