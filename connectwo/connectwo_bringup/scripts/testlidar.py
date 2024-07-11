#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import pymysql
import signal

db = None

def update_robot_position(l_x, l_y):
    global db
    cursor = db.cursor()
    check_sql = "SELECT * FROM robot WHERE name = %s"
    cursor.execute(check_sql, ('RPM-01',))
    existing_row = cursor.fetchone()

    if existing_row is None:
        insert_sql = "INSERT INTO robot (name, lidar_x, ladar_y) VALUES (%s, %s, %s, %s)"
        cursor.execute(insert_sql, ('RPM-01', l_x, l_y))
    else:
        update_sql = "UPDATE robot SET lidar_x = %s, lidar_y = %s WHERE name = %s"
        cursor.execute(update_sql, (l_x, l_y, 'RPM-01'))

    db.commit()
    cursor.close()

def main():
    global db
    rospy.init_node('testdb_lidar')

    db = pymysql.connect(
        host="192.168.0.118",
        user="rpm",
        password="11223344",
        database="rpm"
        )

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:            
            (trans, rot) = listener.lookupTransform('/map', '/base_scan', rospy.Time(0))
            l_x, l_y = trans[0], trans[1]
            update_robot_position(l_x, l_y)
            rospy.loginfo("Updated position: l_x=%f, l_y=%f" % (l_x, l_y))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            continue

        rate.sleep()

    db.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass