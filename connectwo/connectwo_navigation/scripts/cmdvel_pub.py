#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(data):
    # 수신한 메시지에서 angular.z 값을 n배로 증폭
    if data.linear.x == 0.0 and 0 < data.angular.z <= 2.5:
        data.angular.z = 3.5

    elif data.linear.x == 0.0 and -2.5 <= data.angular.z < 0:
        data.angular.z = -3.5

    elif 0 < data.linear.x < 0.05:
        data.linear.x *= 3.0
        if 0 < data.angular.z <= 2.5:
            data.angular.z = 2.8
        elif -2.5 <= data.angular.z < 0:
            data.angular.z = -2.8
        

        if data.angular.z > 3.3:
            data.angular.z = 3.3
        elif -3.3 > data.angular.z:
            data.angular.z = -3.3

    else:
        data.linear.x *= 1.1
        data.angular.z *= 3.3
        data.angular.z = round(data.angular.z, 2)
        if data.angular.z > 3.3:
            data.angular.z = 3.3
        elif -3.3 > data.angular.z:
            data.angular.z = -3.3

    # 증폭된 메시지를 다시 cmd_vel 토픽으로 게시
    cmd_vel_publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_amplifier', anonymous=True)
    
    # /cmd_vel 토픽에서 메시지를 받는 subscriber 생성
    rospy.Subscriber('fake_cmd_vel', Twist, cmd_vel_callback)
    
    # /cmd_vel 토픽으로 메시지를 게시하는 publisher 생성
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # 노드를 실행
    rospy.spin()
