#!/usr/bin/env python

import math
import matplotlib.pyplot as plt

import rospy
from gazebo_msgs.srv import GetJointProperties

def main():
    rospy.init_node('pid_graph')

    rospy.wait_for_service('/gazebo/get_joint_properties')

    service = rospy.ServiceProxy(
        '/gazebo/get_joint_properties', GetJointProperties)

    while not rospy.is_shutdown():
        lvel = 0
        rvel = 0
        for i in range(10):
            l_state = service('PuLSAR::left_wheel_joint')
            r_state = service('PuLSAR::right_wheel_joint')
            lvel += l_state.rate[0]
            rvel += r_state.rate[0]
            rospy.sleep(0.01)
        lvel /= 10
        rvel /= 10

        t = rospy.Time.now()
        t = t.secs + t.nsecs / 1e9

        plt.scatter(t, lvel, c='#ff0000')
        plt.scatter(t, rvel, c='#00ff00')
        plt.legend(['l_vel', 'r_vel'], loc='bottom right')
        plt.xlabel('time')
        plt.ylabel('velocity')
        plt.xlim(t - 18, t + 2)
        plt.pause(0.002)

if __name__ == '__main__':
    main()
