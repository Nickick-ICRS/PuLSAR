#!/usr/bin/env python

import math
import matplotlib.pyplot as plt

import rospy
from gazebo_msgs.srv import GetLinkState

def main():
    rospy.init_node('pid_graph')

    rospy.wait_for_service('/gazebo/get_link_state')

    service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    while not rospy.is_shutdown():
        lvel = 0
        rvel = 0
        for i in range(10):
            l_state = service('PuLSAR::left_wheel_link', 'PuLSAR::base_link')
            r_state = service('PuLSAR::left_wheel_link', 'PuLSAR::base_link')
            lvel += math.sqrt(l_state.link_state.twist.angular.x ** 2 +
                             l_state.link_state.twist.angular.y ** 2 +
                             l_state.link_state.twist.angular.z ** 2)

            rvel += math.sqrt(r_state.link_state.twist.angular.x ** 2 +
                             r_state.link_state.twist.angular.y ** 2 +
                             r_state.link_state.twist.angular.z ** 2)
            rospy.sleep(0.05)
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
