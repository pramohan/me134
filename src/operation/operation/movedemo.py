#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy

from rclpy.node             import Node
from sensor_msgs.msg        import JointState
from std_msgs.msg           import Empty
from rclpy.qos              import QoSProfile, DurabilityPolicy
from std_msgs.msg           import String
from urdf_parser_py.urdf    import Robot

from operation.TransformHelpers import *
from operation.KinematicChain import *
from operation.splines import *

RATE = 100.0            # Hertz

class DemoNode(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)
        self.grabfbk_ready = False
        while not self.grabfbk_ready:
            rclpy.spin_once(self)
        
        # Log initial position and time
        self.position0 = self.grabpos
        self.t0= self.get_clock().now().nanoseconds/10**9

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create an object for the kinematic chain
        # self.fnode = Node("fkin")
        # self.chain = KinematicChain(self.fnode, 'world', 'tip', ['base', 'shoulder', 'elbow'])
        # self.q_safe = np.array([0, -np.pi/2, np.pi/2])
        
        # Grab movement segments for task
        #self.segments_for_x()
        #self.segments_for_line()
        #self.cseg = 0

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

        # for ikin
        # none are used unless the stuff in get_position are uncommented
        self.err = 0
        self.q = np.array([0, 0, 0]).reshape(3,1)   
        
        # Save gravity model parameters
        self.coeffs = [0,0,0,-1.3]#[3.0, 0.5, -1.0, 1.0]

        self.prev_pos = 0
        self.moving = False
        self.flipping = False

    def ikin_NR(self, xd, q_guess):
        # performs the Newton-Raphson algorithm to find the ikin
        q_guess = np.reshape(q_guess, (3,1))
        for i in range(7):
            #J  = np.vstack((self.chain.Jv(),self.chain.Jw())) # shape (6,3)
            Jv = self.chain.Jv()
            dx = (xd - self.chain.ptip().T).T # shape (3,1)
            q_guess += (np.linalg.pinv(Jv) @ dx)
            self.chain.setjoints(q_guess)
        return np.array(q_guess).flatten()

    def sendcmd(self):
        timesec = self.get_clock().now().nanoseconds/10**9 - self.t0
        t = self.get_clock().now().nanoseconds/(10**9)

        # sendPos = self.update(t)
        #print(self.gravity(self.actpos))
        #print("")

        # if there's contact detected, the arm flips
        #self.check_contact(self.grabpos, t)

        # sendEff = self.gravity(self.grabpos)

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder','elbow', 'wrist', 'hand']
        self.cmdmsg.position     = [0,0,0,0,0]#sendPos.tolist()
        #self.cmdmsg.position     = [float("nan"), float("nan"), float("nan")]
        self.cmdmsg.velocity     = []
        #self.cmdmsg.velocity     = [float("nan"), float("nan"), float("nan")]
        self.cmdmsg.effort       = []
        #self.cmdmsg.effort       = sendEff.tolist()
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
