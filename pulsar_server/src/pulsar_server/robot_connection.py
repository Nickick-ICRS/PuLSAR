import threading
import socket


from pulsar_server import message_type


class RobotConnection(threading.Thread):
    class Message:
        """
        Helper class for the message queues
        """
        def __init__(self, msg, priority, timeout):
            self.msg = msg
            self.priority = priority
            self.timeout = timeout
            self.__arrived = rospy.Time.now()
            if timeout is not None:
                self.__ros_timeout = rospy.Duration(timeout)

        
        def timedout(self):
            """
            Check if the message has timed out
            """
            if self.timeout is None:
                return False

            if rospy.Time.now() - self.__arrived > self.__ros_timeout:
                return True
            return False


    def __init__(self, socket_connection):
        """
        Init function. Create relevant variables and then initiate the 
        threading section, which will run the 'run' method.
        """
        self.conn = socket_connection
        self.conn.settimeout(0.1)
        self.alive = True
        self.send_queue = []
        self.received_queue = []
        self.send_lock = threading.Lock()
        self.receive_lock = threading.Lock()

        super(self, threading.Thread).__init__(self, daemon=True)
        self.start()


    def kill(self):
        """ 
        Function to allow this instance to gracefully die in its own time.
        """
        self.alive = False


    def run(self):
        """
        Launches the receive thread, then runs the continuous send thread.
        Launched in the constructor.
        """
        # Run the handshake function to verify information about the robot
        self.__handshake()

        # Launch the continuous receiver
        rec_thread = threading.Thread(target=self.__run_receive)
        rec_thread.start()

        while self.alive:
            # If there's data to be sent, send it
            self.__send()

            # Check each queue to see if any messages have timed out
            with self.send_lock:
                to_remove = []
                for i, item in enumerate(self.send_queue):
                    if item.timedout():
                        to_remove.append(i)
                for i in reversed(to_remove):
                    self.send_queue.pop(i)

            with self.receive_lock:
                to_remove = []
                for i, item in enumerate(self.receive_queue):
                    if item.timedout():
                        to_remove.append(i)
                for i in reversed(to_remove):
                    self.receive_queue.pop(i)

        rec_thread.join()
        self.conn.close()


    def __run_receive(self):
        """ 
        Runs a continuous receive loop. Should be threaded.
        """

        while self.alive:
            # Get any new data that's available
            self. __receive()


    def send(self, msg, priority=0, timeout=0.5):
        """
        Adds a string message 'msg' to queue to be sent to the corresponding
        connected robot.
        0 is the lowest (and default) priority of the message.
        Items of identical priority will be sent in the order that
        they arrive.
        If an item has not been sent after timeout seconds, it will be
        removed from the queue. Set timout to None if the message should
        not timeout.
        """
        new_msg = Message(msg, priority, timeout)
        with self.send_lock:
            for i, item in enumerate(self.send_queue):
                if item.priority < priority:
                    self.send_queue.insert(i, new_msg)
                    return
            self.send_queue.append(new_msg)


    def __handshake(self):
        """
        Initial handshake to find out who the robot is. At the moment
        this is empty.
        """
        msg = ascii("9")
        msg_bytes = bytearray(msg, 'ascii')
        msg_bytes = bytearray(message_type.HANDSHAKE_INIT).extend(msg_bytes)
        m = Message(msg_bytes, 9, None)
        self.send_queue = [m]
        self.__send()
        self.__receive()


    def __send(self):
        """
        Sends a single message down the connection.
        """
        with self.send_lock:
            if len(self.send_queue) == 0:
                return
            full_msg = self.send_queue[0]
            self.send_queue.remove(full_msg)

        msg = full_msg.msg

        totalsent = 0
        MSG_LEN = len(msg)
        while totalsent < MSG_LEN:
            sent = self.conn.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent += sent


    def __receive(self):
        """
        The message format always starts with a single byte saying how long
        the message is, followed by a 2 byte ascii number describing the 
        priority. This is defined in the message_type file.
        """
        data = self.conn.recv(3)
        MSG_LEN = message_types.message_length(data[0])
        PRIORITY = int(data[1:].decode('ascii'))
        data = ""
        received = 0
        while received < MSG_LEN:
            new_data = self.conn.recv(min(MSG_LEN - received, 1024))
            if new_data == '':
                raise RuntimeError("socket connection broken")
            data.append(new_data)
            received += len(new_data)

        received_msg = Message(data.decode('ascii'), PRIORITY, None)
       
        with self.receive_lock:
            for i, item in self.received_queue:
                if item.priority < received_msg.priority:
                    self.received_queue.insert(i, received_msg)
                    return
            self.received_queue.append(received_msg)
