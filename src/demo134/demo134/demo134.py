#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy

from rclpy.node         import Node
from sensor_msgs.msg    import JointState


#
#   Definitions
#
RATE = 100.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.get_logger().info("Initial positions: %r" % self.position0)

        self.t0= self.get_clock().now().nanoseconds/10**9

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Grab a single feedback - do not call this repeatedly.
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos


    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Just print the position (for now).
        # print(list(fbkmsg.position))
        pass

    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # Build up the message and publish.
        timesec = self.get_clock().now().nanoseconds/10**9 - self.t0

        offset = 0.5 * (0 * np.sign(self.position0) + self.position0)
        # also valid:
        # offset = 0.5 * np.array(self.position0)
        amplitude = self.position0 - offset
        #amplitude = self.position0 - (- np.pi / 2 * np.sign(self.position0))
        frequency = 0.5


        p = [amplitude[0] * np.cos(frequency * timesec) + offset[0],
             amplitude[1] * np.cos(frequency * timesec) + offset[1],
             amplitude[2] * np.cos(frequency * timesec) + offset[2]]
        # p to prevent oscillating rotation
        #p = [self.position0[0],
        #     amplitude[1] * np.cos(frequency * timesec) + offset[1],
        #     amplitude[2] * np.cos(frequency * timesec) + offset[2]]
        
        #print(p)
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['one', 'two','three']
        self.cmdmsg.position     = p
        self.cmdmsg.velocity     = []
        self.cmdmsg.effort       = [0.0, 0.0, 0.0]
        self.cmdpub.publish(self.cmdmsg)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
