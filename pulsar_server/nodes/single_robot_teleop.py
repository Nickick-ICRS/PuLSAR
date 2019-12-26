#!/usr/bin/env python3

PORT = 9286
IP = "localhost"

import socket

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pulsar_server.robot_connection import RobotConnection


def main():
    rospy.init_node('single_robot_teleop')
    rospy.loginfo("Listening for robot connection.")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((IP, PORT))
    server.listen(1)

    conn, addr = server.accept()
    rospy.loginfo("Received connection from {}".format(addr))

    robot = RobotConnection(conn)
    
    # 10 Hz update loop
    rate = rospy.Rate(10)

    cmd_vel = Twist()
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
