# pulsar_state_machines

Contains the state machines used for testing localisation. A random state machine and a state machine which attempts to move the robots forwards as a group both exist.

## Nodes

The `state_machine.py` node runs a state machine for a single robot. It takes the following parameters:
- state_machine - Which state machine to use. String. Default is "default". Options are:
    - default - The default, random movement state machine.
    - group_target - The group motion state machine.
- robot_ns - The robot name(space). String. Default is "pulsar_0".
- update_freq - The state machine update frequency in Hz. Decimal. Default is 10.
- delay - Delay the start of state machine logic in seconds. Decimal. Default is 0.

The node then runs one of the below state machines.

## Random Movement

`robot_state_machine.py` contains this state machine, which moves randomly. The robot may be in one of the following states:
- Idle
- Forwards
- Backwards
- Turning

When moving it attempts to avoid collisions, but otherwise picks movement directions randomly.

## Group Motion

This state machine (in `group_target_state_machine.py`) attempts to group with other robots within the swarm, before moving towards a target point. It has the following states:
- Idle
- TurnToSwarm
- TurnToTarget
- MoveToSwarm
- MoveToTarget

## New State Machines

All state machines must inherit from the BaseStateMachine class (in `base_state_machine.py`), and states must inherit from BaseState (in `base_state_machine.py`). These define key functions which are required to be filled in. See the two existing state machines for examples.

It may be beneficial to inherit from RobotStateMachine (in `robot_state_machine.py` to take advantage of ROS callbacks that are built in. In this case see the group motion state machine as an example.
