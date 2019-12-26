# Empty message (except message type and priority)
HANDSHAKE_INIT      = 0x01
# Empty message (except message type and priority)
HANDSHAKE_CONF      = 0x02
# ROS Odometry message
ODOMETRY            = 0x04
# ROS Twist message
TWIST               = 0x08

def message_length(msg_type):
    if msg_type == HANDSHAKE_INIT:
        return 0
    if msg_type == HANDSHAKE_CONF:
        return 0
    if msg_type == ODOMETRY:
        return 0
    if msg_type == TWIST:
        return 0

    raise ValueError("Unknown msg_type: " + msg_type)
