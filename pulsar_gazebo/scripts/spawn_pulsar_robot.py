#!/usr/bin/env python
import os
import random

import roslaunch
import rospkg
import rospy

from std_srvs.srv import Empty


def spawn_robot(uuid, launch_file_path, x_range, y_range, ns):
    cli_args = [launch_file_path,
                "launch_gazebo:=false", 
                "start_x:={}".format(
                    random.uniform(x_range[0], x_range[1])), 
                "start_y:={}".format(
                    random.uniform(y_range[0], y_range[1])), 
                "start_yaw:={}".format(random.uniform(0, 6.28)),
                "robot_ns:={}".format(ns)]
    launch_file = [(roslaunch.rlutil.resolve_launch_arguments(
        cli_args)[0], cli_args[1:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
    launch.start()


def get_params():
    try:
        x_range = rospy.get_param("~x_range")
    except KeyError:
        x_range = [-1, 1]
        rospy.logwarn(
            "Failed to get param 'x_range', defaulting to {}".format(
                x_range))

    if x_range[0] > x_range[1]:
        tmp = x_range[1]
        x_range[1] = x_range[0]
        x_range[0] = tmp

    try:
        y_range = rospy.get_param("~y_range")
    except KeyError:
        y_range = [-1, 1]
        rospy.logwarn(
            "Failed to get param 'y_range', defaulting to {}".format(
                y_range))

    if y_range[0] > y_range[1]:
        tmp = y_range[1]
        y_range[1] = y_range[0]
        y_range[0] = tmp

    try:
        num_robots = rospy.get_param("~num_robots")
    except KeyError:
        num_robots = 5
        rospy.logwarn(
            "Failed to get param 'num_robots', defaulting to {}".format(
                num_robots))

    try:
        robot_prefix = rospy.get_param("~robot_prefix")
    except KeyError:
        robot_prefix = "pulsar"
        rospy.logwarn(
            "Failed to get param 'robot_prefix', defaulting to {}".format(
                robot_prefix))
        
    return x_range, y_range, num_robots, robot_prefix


def main():
    random.seed(None)
    rospy.init_node("spawn_pulsar_script")

    rospy.wait_for_service("/gazebo/pause_physics")
    rospy.wait_for_service("/gazebo/unpause_physics")
    pause_gazebo = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    x_range, y_range, num_robots, robot_prefix = get_params()

    rospack = rospkg.RosPack()
    launch_file_path = os.path.join(
        rospack.get_path("pulsar_gazebo"), "launch/one_robot.launch")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    pause_gazebo()

    for i in range(num_robots):
        robot_name = robot_prefix + "_{}".format(i)
        spawn_robot(uuid, launch_file_path, x_range, y_range, robot_name)

    unpause_gazebo()

    # If we just exit then the nodes are all shut down
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
