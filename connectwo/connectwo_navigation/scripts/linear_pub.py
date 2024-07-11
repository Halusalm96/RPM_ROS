#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("fake_odom", Odometry, queue_size=50) 
odom_broadcaster = tf.TransformBroadcaster()

def cmd_vel_callback(data): #가속도를 cmd_vel에서 받아오는 함수
    global vx, vth
    vx = data.linear.x #가속도
   

def odom_callback(data): #각속도를 odom에서 받아오는 함수
    global vth
    vth = data.twist.twist.angular.z

cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback) #cmd_vel값을 subscriber로 받아오는 구문

odom_sub = rospy.Subscriber("odom", Odometry, odom_callback) #odom값을 subscriber로 받아오는 구문

x = 0.0
y = 0.0
th = 0.0
vx = 0.0  # Set an initial value for vx
vy = 0.0
vth = 0.0

orientation_z= 0.0
orientation_w = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF, create a quaternion from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )
    
    # publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
