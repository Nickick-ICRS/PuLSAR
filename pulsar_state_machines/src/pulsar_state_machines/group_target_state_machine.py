import rospy
import tf2_ros
import random
import math

from pulsar_state_machines.base_state_machine import BaseState
from pulsar_state_machines.robot_state_machine import RobotStateData, RobotStateMachine

from geometry_msgs.msg import Twist, Quaternion, Point, PointStamped, PoseWithCovariance, PoseWithCovarianceStamped

def calc_pos_dist(a, b):
    """
    Calculates the distance between two points.

    @param a A point.

    @param b A point.
    """
    dx = a.x - b.x
    dy = a.y - b.y
    return math.sqrt(dx ** 2 + dy ** 2)


def calc_ang_diff(a, b):
    """
    Calculates the angle between two orientations.

    @param a A quaternion.

    @param b A quaternion.
    """
    ang_a = 2 * math.acos(a.w) * (-1 if a.z < 0 else 1)
    ang_b = 2 * math.acos(b.w) * (-1 if b.z < 0 else 1)
    diff = ang_a - ang_b
    while diff > math.pi:
        diff -= 2*math.pi
    while diff < -math.pi:
        diff += 2*math.pi
    return diff


class SwarmStateData:
    def __init__(self):
        # Robot specific data
        self.robot_data = RobotStateData()

        # Pose estimate of the swarm
        self.swarm_pose = PoseWithCovariance()

        # Target position of the pose estimate of the swarm
        self.target_point = Point()
        
        # If this is not true then either of the above two are invalid!
        self.pose_and_target_valid = False


class IdleState(BaseState):
    """
    This state just publishes a 0 cmd_vel to the robot.
    """
    def update(self, Z):
        cmd_vel = Twist()
        BaseState.pub.publish(cmd_vel)
        r = random.random()
        if r < 0.1:
            return random.choice([
                MoveToSwarmState(), MoveToTargetState(),
                TurnToSwarmState(), TurnToTargetState()])
        return self

    def __str__(self):
        return "IdleState"

    def __repr__(self):
        return "IdleState"


class TurnToAngleState(BaseState):
    """
    An abstract class to represent turning towards a requested angle.
    """
    def __init__(self):
        self._P_ANG   = 10.0
        self._MIN_VEL =  4.0
        self._MAX_VEL =  7.0

    def update_cmd_vel(self, Z, target_ang):
        """
        Updates the cmd_vel published to the robot to attempt to turn to
        a given angle.

        @param Z The current robot state data of type RobotStateData.

        @param target_ang The target angle for the robot to arrive at. As
                          a quaternion.
        """
        cmd_vel = Twist()
        if not Z.pose_valid:
            # Warn and publish 0 cmd_vel for safety
            rospy.logwarn_throttle(5, "Robot pose state is invalid...")
        else:
            # LEZ GOOOOO
            dang = calc_ang_diff(target_ang, Z.mTb.orientation)
            rospy.loginfo("dang: " + str(180*dang/math.pi))

            while dang > math.pi:
                dang -= 2*math.pi
            while dang < -math.pi:
                dang += 2*math.pi

            target_vel = self._P_ANG * (dang)
            current_vel = Z.twist.angular.z

            # Make everything positive, but remember the sign
            sign = 1
            if target_vel < 0:
                target_vel *= -1
                current_vel *= -1
                sign = -1

            # To ensure the robot doesn't get stuck
            dvel = target_vel - current_vel
            min_vel = 0
            if dvel > 0.05:
                min_vel = self._MIN_VEL

            cmd_vel.angular.z = sign * max(
                min_vel, min(target_vel, self._MAX_VEL))

        BaseState.pub.publish(cmd_vel)


class MoveState(BaseState):
    """
    An abstract class to represent moving forwards // backwards.
    """
    def __init__(self):
        self._P       = 0.5
        self._MIN_VEL = 0.1
        self._MAX_VEL = 0.3
        self._FUTURE  = 0.2 # Seconds
        self._TURN_P       = 0.16
        self._MIN_TURN_VEL = 4.00
        self._MAX_TURN_VEL = 7.00

    def update_cmd_vel(self, Z, sign=1):
        """
        Updates the cmd_vel to make the robot go forwards (or backwards),
        slowing down if there are obstacles in front of it.

        @param Z The current robot state data of type RobotStateData.

        @param sign The sign of the motion vector, i.e. 1 for forwards and
                    -1 for negative.
        """
        cmd_vel = Twist()
        if not Z.pose_valid:
            # Warn and publish 0 cmd_vel for safety
            rospy.logwarn_throttle(5, "Robot pose state is invalid...")
        else:
            # LEZ GOOOO
            # Modifier to adjust the output speed
            cmd_vel_mod = 1
            # Predict where we will be in a few timesteps
            m = Z.m.range
            vel = Z.twist.linear.x
            if vel * self._FUTURE > m:
                # We're gonna be in a wall or another robot!
                # Output a lower cmd_vel as a result
                cmd_vel_mod = 0.5
            elif vel * 2 * self._FUTURE < m:
                # Plenty of room. Go a bit faster.
                cmd_vel_mod = 2

            closest = min(Z.l.range - Z.l.min_range,
                          Z.m.range - Z.m.min_range,
                          Z.r.range - Z.r.min_range)
            
            target_vel = self._MIN_VEL + self._P * closest

            cmd_vel.linear.x = sign * min(
                cmd_vel_mod * target_vel, self._MAX_VEL)

            # Turn away if we're too close to an object
            turn_sign = 1
            if Z.l.range - Z.l.min_range < Z.r.range - Z.r.min_range:
                turn_sign = -1
            if closest < 0.001:
                ang = self._MAX_TURN_VEL
            else:
                ang = min(self._TURN_P / closest, self._MAX_TURN_VEL)
                if ang >= self._MIN_TURN_VEL:
                    cmd_vel.angular.z = turn_sign * ang
                    cmd_vel.linear.x = 0
        BaseState.pub.publish(cmd_vel)


class TurnToSwarmState(TurnToAngleState):
    """
    Turns the robot towards the swarm.
    """
    def __init__(self):
        super(TurnToSwarmState, self).__init__()
        self._ANG_MAX_TOL = 0.5

    def __str__(self):
        return "TurnToSwarmState"
    
    def __repr__(self):
        return "TurnToSwarmState"

    def update(self, Z):
        """
        Make the robot turn towards the swarm.

        @param Z State of the robot within the swarm of type SwarmStateData.
        """
        if not Z.pose_and_target_valid:
            rospy.logwarn_throttle(
                10, "Pose and/or target are invalid... Perhaps you haven't"
                + " set a target?")
            return IdleState()
        
        # Calculate the required angle and update velocity
        req_ang = math.atan2(
            Z.swarm_pose.pose.position.y - Z.robot_data.mTb.position.y,
            Z.swarm_pose.pose.position.x - Z.robot_data.mTb.position.x)

        req_quat = Quaternion()
        req_quat.w = math.cos(req_ang / 2)
        req_quat.z = math.sin(req_ang / 2)

        self.update_cmd_vel(Z.robot_data, req_quat)

        # Update the state
        r = random.random()
        p_idle = 0.01
        if r < p_idle:
            return IdleState()
        r -= p_idle
        
        p_toswarm = 1 - abs(
            calc_ang_diff(Z.robot_data.mTb.orientation, req_quat) 
            / self._ANG_MAX_TOL)
        if r < p_toswarm:
            return MoveToSwarmState()

        return self


class TurnToTargetState(TurnToAngleState):
    """
    Attempt to turn towards the target point.
    """
    def __init__(self):
        super(TurnToTargetState, self).__init__()
        self._ANG_MAX_TOL = 0.5

    def __str__(self):
        return "TurnToTargetState"
    
    def __repr__(self):
        return "TurnToTargetState"

    def update(self, Z):
        """
        Make the robot turn towards the target.

        @param Z State of the robot within the swarm of type SwarmStateData.
        """
        if not Z.pose_and_target_valid:
            rospy.logwarn_throttle(
                10, "Pose and/or target are invalid... Perhaps you haven't"
                + " set a target?")
            return IdleState()
        
        # Calculate the required angle and update velocity
        req_ang = math.atan2(
            Z.target_point.y - Z.robot_data.mTb.position.y,
            Z.target_point.x - Z.robot_data.mTb.position.x)

        req_quat = Quaternion()
        req_quat.w = math.cos(req_ang / 2)
        req_quat.z = math.sin(req_ang / 2)

        self.update_cmd_vel(Z.robot_data, req_quat)

        # Update the state
        r = random.random()
        p_idle = 0.01
        if r < p_idle:
            return IdleState()
        r -= p_idle
        
        p_totarget = 1 - abs(
            calc_ang_diff(Z.robot_data.mTb.orientation, req_quat) 
            / self._ANG_MAX_TOL)
        if r < p_totarget:
            return MoveToTargetState()

        return self


class MoveToSwarmState(MoveState):
    """
    A state which attempts to move the robot towards the epicenter of the
    swarm.
    """
    def __init__(self):
        super(MoveToSwarmState, self).__init__()
        self._ANG_TOL = 1

    def __str__(self):
        return "MoveToSwarmState"

    def __repr__(self):
        return "MoveToSwarmState"
    
    def update(self, Z):
        """
        Make the robot move towards the swarm.

        @param Z State of the robot within the swarm of type SwarmStateData.
        """
        if not Z.pose_and_target_valid:
            rospy.logwarn_throttle(
                10, "Pose and/or target are invalid... Perhaps you haven't"
                + " set a target?")
            return IdleState()

        # Move forwards
        if Z.robot_data.m.range - Z.robot_data.m.min_range > 0.01:
            self.update_cmd_vel(Z.robot_data, sign=1);
        else:
            self.update_cmd_vel(Z.robot_data, sign=-1);

        # Calculate the angle between us and the swarm and the distance
        dx = Z.swarm_pose.pose.position.x - Z.robot_data.mTb.position.x
        dy = Z.swarm_pose.pose.position.y - Z.robot_data.mTb.position.y
        req_ang = math.atan2(dy, dx)

        req_quat = Quaternion()
        req_quat.w = math.cos(req_ang / 2)
        req_quat.z = math.sin(req_ang / 2)

        dang = calc_ang_diff(req_quat, Z.robot_data.mTb.orientation)
        # Grab std deviations - we want to get within 1 std deviation
        stddevx = math.sqrt(Z.swarm_pose.covariance[0])
        stddevy = math.sqrt(Z.swarm_pose.covariance[7])
        # Particular case where there is 1 robot, cov = 0...
        if stddevx < 2e-2:
            stddevx = 2e-2
        if stddevy < 2e-2:
            stddevy = 2e-2

        # Update state
        r = random.random()

        arrived_p = min(
            1 - dx / stddevx, 1 - dy / stddevy,
            1 - (dx / stddevx) * (dy / stddevy))
        arrived_p = max(arrived_p, 0)
        if r < arrived_p:
            return TurnToTargetState()
        r -= arrived_p

        lost_p = abs(0.3 * dang / self._ANG_TOL)
        rospy.loginfo("lost_p: " + str(lost_p))
        if r < lost_p:
            return TurnToSwarmState()
        r-= lost_p

        idle_p = 0.03
        if r < idle_p:
            return IdleState()
        r -= idle_p

        return self


class MoveToTargetState(MoveState):
    """
    A state which attempts to move the robot towards the target point.
    """
    def __init__(self):
        super(MoveToTargetState, self).__init__()
        self._ANG_TOL = 1

    def __str__(self):
        return "MoveToTargetState"

    def __repr__(self):
        return "MoveToTargetState"
    
    def update(self, Z):
        """
        Make the robot move towards the swarm.

        @param Z State of the robot within the swarm of type SwarmStateData.
        """
        if not Z.pose_and_target_valid:
            rospy.logwarn_throttle(
                10, "Pose and/or target are invalid... Perhaps you haven't"
                + " set a target?")
            return IdleState()

        # Move forwards
        if Z.robot_data.m.range - Z.robot_data.m.min_range > 0.01:
            self.update_cmd_vel(Z.robot_data, sign=1);
        else:
            self.update_cmd_vel(Z.robot_data, sign=-1);

        # Calculate the angle between us and the target and the distance
        dx = Z.target_point.x - Z.robot_data.mTb.position.x
        dy = Z.target_point.y - Z.robot_data.mTb.position.y
        req_ang = math.atan2(dy, dx)

        req_quat = Quaternion()
        req_quat.w = math.cos(req_ang / 2)
        req_quat.z = math.sin(req_ang / 2)

        dang = calc_ang_diff(req_quat, Z.robot_data.mTb.orientation)
        # Grab std deviations - we want to get within 1 std deviation
        stddevx = math.sqrt(Z.swarm_pose.covariance[0])
        stddevy = math.sqrt(Z.swarm_pose.covariance[7])
        # Particular case where there is 1 robot, cov = 0...
        if stddevx < 2e-2:
            stddevx = 2e-2
        if stddevy < 2e-2:
            stddevy = 2e-2

        # Distance between us and the swarm
        sdx = Z.swarm_pose.pose.position.x - Z.robot_data.mTb.position.x
        sdy = Z.swarm_pose.pose.position.y - Z.robot_data.mTb.position.y

        # Update state
        r = random.random()

        left_p = max(
            sdx / stddevx - 1, sdy / stddevy - 1, 0)
        if r < left_p:
            return TurnToSwarmState()
        r -= left_p

        lost_p = abs(0.1 * dang / self._ANG_TOL)
        if r < lost_p:
            return TurnToTargetState()
        r-= lost_p

        idle_p = 0.03
        if r < idle_p:
            return IdleState()
        r -= idle_p

        return self


class GroupTargetStateMachine(RobotStateMachine):
    """
    Class to represent a robot state machine which attempts to move to a
    requested point while staying with the swarm.
    """
    def __init__(self, robot_ns, initial_state=IdleState()):
        """
        Constructor.

        @param robot_ns The name and namespace of the robot (e.g. 
                        'pulsar_0') that this state machine controls.

        @param initial_state Starting state of the robot.
        """
        super(GroupTargetStateMachine, self).__init__(
            robot_ns, initial_state=initial_state)

        self._target_point = None
        self._swarm_pose = None

        rospy.Subscriber(
            "/swarm_target_point", PointStamped, self.target_point_cb)

        rospy.Subscriber(
            "/swarm_pose_estimate", PoseWithCovarianceStamped,
            self.swarm_pose_cb)
    
    def get_data(self):
        Z = SwarmStateData()
        Z.robot_data = super(GroupTargetStateMachine, self).get_data()
        if self._swarm_pose != None and self._target_point != None:
            Z.swarm_pose = self._swarm_pose
            Z.target_point = self._target_point
            Z.pose_and_target_valid = True
        return Z

    def target_point_cb(self, msg):
        """
        Target point for the swarm to get to.

        @param msg Target point of type PointStamped
        """
        self._target_point = msg.point
    
    def swarm_pose_cb(self, msg):
        """
        Current pose estimate of the swarm as a whole.

        @param msg The pose estimate of type PoseWithCovarianceStamped
        """
        self._swarm_pose = msg.pose

    def __str__(self):
        return "GroupTargetStateMachine"
