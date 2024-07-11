#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import pymysql
import signal

db = None

def update_robot_position(x, y):
    global db
    cursor = db.cursor()
    check_sql = "SELECT * FROM robot WHERE name = %s"
    cursor.execute(check_sql, ('RPM-01',))
    existing_row = cursor.fetchone()

    if existing_row is None:
        insert_sql = "INSERT INTO robot (name, current_x, current_y, status) VALUES (%s, %s, %s, %s)"
        cursor.execute(insert_sql, ('RPM-01', x, y, 'online'))
    else:
        update_sql = "UPDATE robot SET current_x = %s, current_y = %s, status = %s WHERE name = %s"
        cursor.execute(update_sql, (x, y, 'online', 'RPM-01'))

    db.commit()
    cursor.close()

def update_robot_offline():
    global db
    cursor = db.cursor()
    update_sql = "UPDATE robot SET status = 'offline' WHERE name = 'RPM-01'"
    cursor.execute(update_sql)
    db.commit()
    cursor.close()

def signal_handler(signal, frame):
    global db
    rospy.loginfo("Node is Close..")
    update_robot_offline()
    rospy.signal_shutdown("Node is Closed")

def main():
    global db
    rospy.init_node('testdb')

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
            (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            x, y = trans[0], trans[1]
            update_robot_position(x, y)
            rospy.loginfo("Updated position: x=%f, y=%f" % (x, y))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            continue

        rate.sleep()

    db.close()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C 종료 시 시그널
    try:
        main()
    except rospy.ROSInterruptException:
        pass