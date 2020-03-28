#!/usr/bin/env python
import os
import random
import math

import roslaunch
import rospkg
import rospy

from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid


def is_pose_valid(x, y):
    global map_grid
    global map_grid_data
    global robot_radius
    
    th = 2*math.acos(map_grid.info.origin.orientation.w)
    dx = x - map_grid.info.origin.position.x
    dy = y - map_grid.info.origin.position.y

    row = int((math.cos(th)*dx-math.sin(th)*dy) / map_grid.info.resolution + 0.5)
    col = int((math.sin(th)*dx+math.cos(th)*dy) / map_grid.info.resolution + 0.5)

    if row < 0:
        return False
    if col < 0:
        return False
    if row >= map_grid.info.height:
        return False
    if col >= map_grid.info.width:
        return False

    # Check the area around the pose
    for r in range(int(2*robot_radius / map_grid.info.resolution)+1):
        r2 = int(r - robot_radius / map_grid.info.resolution)
        if row + r2 < 0:
            continue
        if row + r2 >= map_grid.info.height:
            continue
        for c in range(int(2*robot_radius / map_grid.info.resolution)+1):
            c2 = int(c - robot_radius / map_grid.info.resolution)
            if col + c2 < 0:
                continue
            if col + c2 >= map_grid.info.width:
                continue
            dr = float(r2) * map_grid.info.resolution
            dc = float(c2) * map_grid.info.resolution
            if math.sqrt(dr * dr + dc * dc) < robot_radius:
                if map_grid_data[
                        (col+c2)*map_grid.info.height + (row+r2)] != 0:
                    return False
    return True


def update_map_with_robot(x, y):
    global map_grid
    global map_grid_data
    global robot_radius

    if not is_pose_valid(x, y):
        raise RuntimeError("Check for pose validity before calling this!")
    
    th = 2*math.acos(map_grid.info.origin.orientation.w)
    dx = x - map_grid.info.origin.position.x
    dy = y - map_grid.info.origin.position.y

    row = int((math.cos(th)*dx-math.sin(th)*dy) / map_grid.info.resolution + 0.5)
    col = int((math.sin(th)*dx+math.cos(th)*dy) / map_grid.info.resolution + 0.5)

    # Set the area around the pose as occupied
    for r in range(int(2*robot_radius / map_grid.info.resolution)+1):
        r2 = int(r - robot_radius / map_grid.info.resolution)
        if row + r2 < 0:
            continue
        if row + r2 >= map_grid.info.height:
            continue
        for c in range(int(2*robot_radius / map_grid.info.resolution)+1):
            c2 = int(c - robot_radius / map_grid.info.resolution)
            if col + c2 < 0:
                continue
            if col + c2 >= map_grid.info.width:
                continue
            dr = float(r2) * map_grid.info.resolution
            dc = float(c2) * map_grid.info.resolution
            if math.sqrt(dr * dr + dc * dc) < robot_radius:
                map_grid_data[
                    (col+c2)*map_grid.info.height + (row+r2)] = 100


def spawn_robot(uuid, launch_file_path, x_range, y_range, ns):
    while not rospy.is_shutdown():
        sx = random.uniform(x_range[0], x_range[1]) 
        sy = random.uniform(y_range[0], y_range[1]) 
        syaw = random.uniform(0, 6.28)
        if is_pose_valid(sx, sy):
            break

    update_map_with_robot(sx, sy)

    rospy.set_param(
        "/"+ns+"/initial_pose", 
        [sx, sy, 0, 0, 0, math.sin(syaw/2), math.cos(syaw/2)])

    cli_args = [launch_file_path,
                "launch_gazebo:=false", 
                "start_x:={}".format(sx),
                "start_y:={}".format(sy),
                "start_yaw:={}".format(syaw),
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

    try:
        robot_radius = rospy.get_param("~robot_radius")
    except KeyError:
        robot_radius = 0.05
        rospy.logwarn(
            "Failed to get param 'robot_radius', defaulting to {}".format(
                robot_radius))
        
    return x_range, y_range, num_robots, robot_prefix, robot_radius


def map_cb(msg):
    global map_grid
    global map_grid_data
    map_grid = msg
    map_grid_data = list(map_grid.data)


def main():
    global map_grid
    global robot_radius
    random.seed(None)
    rospy.init_node("spawn_pulsar_script")

    rospy.wait_for_service("/gazebo/pause_physics")
    rospy.wait_for_service("/gazebo/unpause_physics")
    pause_gazebo = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

    map_grid = None
    rospy.Subscriber("/map", OccupancyGrid, map_cb)

    x_range, y_range, num_robots, robot_prefix, robot_radius = get_params()

    rospack = rospkg.RosPack()
    launch_file_path = os.path.join(
        rospack.get_path("pulsar_gazebo"), "launch/one_robot.launch")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Wait for the map to exist
    while(map_grid == None):
        rospy.logwarn_throttle(5, "Waiting for /map to become available.")

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
