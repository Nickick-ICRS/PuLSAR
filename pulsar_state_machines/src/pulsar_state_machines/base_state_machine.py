import rospy

from geometry_msgs.msg import Twist

class BaseState(object):
    """
    Abstract base class to represent a state within a state machine.

    Has a static publisher. This must be initialised by the state machine.
    """
    pub = None
    def __init__(self):
        """
        Constructor.
        """
        pass

    def update(self, Z):
        """
        The update function should run one iteration of the state, and
        return the new state (which can be itself)
        """
        raise NotImplementedError

    def __str__(self):
        return "BaseState"

    def __repr__(self):
        return "BaseState"

class BaseStateMachine(object):
    """
    Abstract base class to represent a state machine.
    """
    def __init__(self, robot_ns, initial_state=BaseState()):
        """
        Constructor.

        @param robot_ns The namespace of the robot.

        @param initial_state The starting state of the state machine.
        """
        self._current_state = initial_state
        BaseState.pub = rospy.Publisher(
            "/"+robot_ns+"/cmd_vel", Twist, queue_size=1)

    def update_loop(self, rate_hz, debug=False):
        """
        Runs a continuous update loop until ROS is shutdown.

        @param rate_hz The frequency in Hz of the update loop.

        @param debug If this is true then states will be printed every loop.
        """
        sleeper = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            Z = self.get_data()
            self.update_once(Z, debug=debug)
            sleeper.sleep()

    def update_once(self, Z, debug=False):
        """
        Runs a single state update.

        @param Z The current measurement data from robot sensors etc.

        @param debug If this is true then print the resulting state.
        """
        self._current_state = self._current_state.update(Z)
        if debug:
            print("["+str(self)+"] STATE: " + str(self._current_state))
    
    def get_data(self):
        """
        Overload this function so that it returns appropriate data from the
        robot etc.
        """
        raise NotImplementedError

    def __str__(self):
        return "BaseStateMachine"

    def __repr__(self):
        return str(self) + " with current state " + str(self._current_state)
