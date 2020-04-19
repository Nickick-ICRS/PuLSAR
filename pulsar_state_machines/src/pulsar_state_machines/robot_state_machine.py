import rospy
import tf2_ros
import random

from pulsar_state_machines.base_state_machine import BaseState, BaseStateMachine

from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry


class RobotStateData:
    def __init__(self):
        """
        Constructor.
        """
        # Robot range sensor data
        self.l = Range()
        self.m = Range()
        self.r = Range()

        # Robot pose wrt map frame
        self.mTb = Pose()
        # If this is not true, the pose is invalid!
        self.pose_valid = False

        # Velocity of robot in its current frame
        self.twist = Twist()


class IdleState(BaseState):
    """
    This state just publishes a 0 cmd_vel to the robot.
    """
    def update(self, Z):
        cmd_vel = Twist()
        BaseState.pub.publish(cmd_vel)
        r = random.random()
        if r < 0.1:
            return ForwardsState()
        return self

    def __str__(self):
        return "IdleState"

    def __repr__(self):
        return "IdleState"


class ForwardsState(BaseState):
    """
    This state moves the robot forwards.
    """
    def __init__(self):
        self._MIN_VEL = 0.15
        self._MAX_VEL = 0.30
        self._P       = 0.75

    def update(self, Z):
        cmd_vel = Twist()
        closest = min(Z.l.range - Z.l.min_range,
                      Z.m.range - Z.m.min_range,
                      Z.r.range - Z.r.min_range)
        cmd_vel.linear.x = min(
            self._MIN_VEL + self._P * closest, self._MAX_VEL)
        BaseState.pub.publish(cmd_vel)
        r = random.random()
        idle_p = 0.05
        if r < idle_p:
            return IdleState()
        r -= idle_p
        back_p = max(1 - 5*(Z.m.range - Z.m.min_range), 0)
        if r < back_p:
            return BackwardsState()
        r -= back_p
        closest = min(Z.l.range - Z.l.min_range, Z.r.range - Z.r.min_range)
        turn_p = min(1 - 5*closest, 0)
        if r < turn_p:
            return TurningState()
        return self

    def __str__(self):
        return "ForwardsState"

    def __repr__(self):
        return "ForwardsState"


class BackwardsState(BaseState):
    """
    This state moves the robot backwards.
    """
    def __init__(self):
        self._MIN_VEL = 0.10
        self._MAX_VEL = 0.20
        self._P       = 0.05

    def update(self, Z):
        cmd_vel = Twist()
        closest = min(Z.l.range - Z.l.min_range,
                      Z.m.range - Z.m.min_range,
                      Z.r.range - Z.r.min_range)
        cmd_vel.linear.x = -min(
            self._MIN_VEL + self._P * closest, self._MAX_VEL)
        BaseState.pub.publish(cmd_vel)

        if Z.m.range - Z.m.min_range < 0.01:
            return self

        r = random.random()
        turn_p = 0.4
        if r < turn_p:
            return TurningState()
        r -= turn_p
        idle_p = 0.01
        if r < idle_p:
            return IdleState()
        r-= idle_p
        forw_p = 0.3
        if r < forw_p:
            return ForwardsState()
        return self

    def __str__(self):
        return "BackwardsState"

    def __repr__(self):
        return "BackwardsState"


class TurningState(BaseState):
    def __init__(self):
        self._MIN_VEL = 4
        self._MAX_VEL = 7
        self._P       = 2

    def update(self, Z):
        cmd_vel = Twist()
        l = Z.l.range - Z.l.min_range
        r = Z.r.range - Z.r.min_range
        if l < r:
            if l < 0.01:
                cmd_vel.angular.z = -self._MAX_VEL
            else:
                cmd_vel.angular.z = -min(
                    self._MIN_VEL + self._P / l, self._MAX_VEL)
        else:
            if r < 0.01:
                cmd_vel.angular.z = self._MAX_VEL
            else:
                cmd_vel.angular.z = min(
                    self._MIN_VEL + self._P / r, self._MAX_VEL)
        BaseState.pub.publish(cmd_vel)

        r = random.random()
        forw_p = 0.06
        if r < forw_p:
            return ForwardsState()
        r -= forw_p
        idle_p = 0.02
        if r < idle_p:
            return IdleState()
        return self

    def __str__(self):
        return "TurningState"
    
    def __repr__(self):
        return "TurningState"


class RobotStateMachine(BaseStateMachine):
    """
    Class to represent a robot state machine with data gathering from
    sensors and odometry.
    """
    def __init__(self, robot_ns, initial_state=IdleState()):
        """
        Constructor.

        @param robot_ns The name and namespace of the robot (e.g. 
                        'pulsar_0') that this state machine controls.

        @param initial_state Starting state of the robot.
        """
        super(RobotStateMachine, self).__init__(
            robot_ns, initial_state=initial_state)
        
        self._range_l = Range()
        self._range_m = Range()
        self._range_r = Range()
        self._odom = Odometry()

        rospy.Subscriber(
            "/"+robot_ns+"/left_range_finder/range", Range, self.range_cb)
        rospy.Subscriber(
            "/"+robot_ns+"/middle_range_finder/range", Range, self.range_cb)
        rospy.Subscriber(
            "/"+robot_ns+"/right_range_finder/range", Range, self.range_cb)
        rospy.Subscriber(
            "/"+robot_ns+"/odometry/filtered", Odometry, self.odom_cb)

        self._tf2 = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf2)

        self._base_link = robot_ns+"/base_link"
    
    def get_data(self):
        """
        Gets the current robot state data.
        """
        Z = RobotStateData()
        # Get most recent range data
        Z.l = self._range_l
        Z.m = self._range_m
        Z.r = self._range_r

        # Get most recent odometry data
        Z.twist = self._odom.twist.twist

        # Ask for most recent map pose data
        try:
            trans = self._tf2.lookup_transform(
                "map", self._base_link, rospy.Time(0))
            Z.pose_valid = True
            Z.mTb.position.x = trans.transform.translation.x
            Z.mTb.position.y = trans.transform.translation.y
            Z.mTb.position.z = trans.transform.translation.z
            Z.mTb.orientation = trans.transform.rotation
        except tf2_ros.TransformException as e:
            # If this fails then the pose is invalid by default
            rospy.logwarn(e)
        return Z

    def range_cb(self, msg):
        """
        Range sensor measurement callback.

        @param msg Range sensor data of type Range.
        """
        if "left" in msg.header.frame_id:
            self._range_l = msg
        elif "middle" in msg.header.frame_id:
            self._range_m = msg
        elif "right" in msg.header.frame_id:
            self._range_r = msg
        else:
            rospy.logerr(
                "Unknown range sensor coordinate frame: " + 
                msg.header.frame_id)

    def odom_cb(self, msg):
        """
        Odometry measurement callback.
        @param msg Odometry sensor data of type Odometry.
        """
        self._odom = msg

    def __str__(self):
        return "RobotStateMachine"
