#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose
 
def callback(msg):
    init_pose = SetPose()
    init_pose.pose = msg
    print(init_pose.pose)
    initialize_service(init_pose.pose)
 
 
rospy.init_node('pose_initializer', anonymous=True)
 
sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped,callback)
rospy.wait_for_service('/set_pose')
 
initialize_service = rospy.ServiceProxy('/set_pose', SetPose)
#  /initialpose topic을 받고, 그 데이터를 기반으로 /set_pose service를 호출 
 
rospy.spin()
