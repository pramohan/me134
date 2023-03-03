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


#
#   Main Test Code
#
#   Simply test the above functions to make sure we have them correct.
#
if __name__ == "__main__":
    # Prepare the print format.
    np.set_printoptions(precision=6, suppress=True)

    # Test...
    # R = Rotx(np.radians(45))
    # print("R:\n", R)

    # quat = quat_from_R(R)
    # print("quat:\n", quat)

    # print("R_from_quat():\n",  R_from_quat(quat))

#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = np.array(self.grabfbk())
        self.get_logger().info("Initial positions: %r" % self.position0)
        self.t0= self.get_clock().now().nanoseconds/10**9

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create an object for the kinematic chain
        self.fnode = Node("fkin")
        self.chain = KinematicChain(self.fnode, 'world', 'tip', ['base', 'shoulder', 'elbow', 'wrist', 'finger'])
        self.q_safe = np.array([0.0, 0.0, .0, 1.57, .0])
        
        # Grab movement segments for task
        self.segments_for_x()
        #self.segments_for_line()
        self.cseg = 0
        # self.ignore = 1
        # x_d1 = np.array([-0.4, -0., 0.2]) # the middle X on the table
        # self.segments = [Goto(self.chain.fkin(self.position0.flatten()), x_d1, 4, 'Task'),
        #                  Goto(x_d1, x_d1, 4, 'Task')]
        # Subscribe to the flip command
        #self.flipsub = self.create_subscription(Empty, '/flip', self.cb_flip, 1)

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
        
        # Subscribe to a number
        #self.numbersub = self.create_subscription(Float, '/number', self.cb_number, 1)


    def cb_flip(self, msg):
        self.flipping = True
        self.get_logger().info("Flipping...")
        q = np.copy(self.grabpos)
        q0 = np.array([q[0] + (np.pi if q[0] <0 else -np.pi),q[1],q[2]])
        q1 = np.array([q0[0], (np.pi-abs(q[1]))*np.sign(q[1]), q[2]])
        q2 = np.array([q1[0], q1[1], -q[2]])
        self.prev_pos = q2 # update for check_contact() comparison
        # self.get_logger().info(str(q))
        # self.get_logger().info(str(q0))
        # self.get_logger().info(str(q1))
        # self.get_logger().info(str(q2))
        # self.get_logger().info("bruh")
        self.chain.setjoints(q2)
        x_0 = self.chain.ptip().flatten()

        self.t0    = self.get_clock().now().nanoseconds/10**9 #self.t0 + self.segments[self.cseg].duration()

        self.segments = [Goto(q, q2, 4.0, 'Joint'),
                         #Goto(q0, q1, 4.0, 'Joint'),
                         #Goto(q1, q2, 4.0, 'Joint'),
                         Goto(x_0, self.x_SL[0], 4, 'Task'),
                         Goto(self.x_SL[0], self.x_SL[1], 4, 'Task'),
                         Goto(self.x_SL[1], self.x_SL[0], 4, 'Task')]
        self.ignore = 2 # ignore spline from 
        self.cseg = 0


    def segments_for_x(self):
        # moves arm to each point
        self.ignore = 1 # ignore spline from initial pos to home
        self.chain.setjoints(self.q_safe)
        x_0 = self.chain.ptip().flatten()
        x_d1 = np.array([-0.4, -0., 0.2]) # the middle X on the table
        x_d2 = np.array([-0.30, 0.15, 0.2]) # left X
        # x_d3 = np.array([-0.075, 0.105, 0.02]) # right X

        # self.segments = [Goto(self.position0, self.q_safe, 3.0, 'Joint'),
        #                  Goto(x_0, x_d1, 3.0, 'Task'),
        #                  Goto(x_d1, x_0, 3.0, 'Task'),
        #                  Goto(x_0, x_d2, 3.0, 'Task'),
        #                  Goto(x_d2, x_0, 3.0, 'Task'),
        #                  Goto(x_0, x_d3, 3.0, 'Task'),
        #                  Goto(x_d3, x_0, 3.0, 'Task'),]

        self.segments = [Goto(self.position0, self.q_safe, 3.0, 'Joint'),
                         Goto(x_0, x_d1, 3.0, 'Task'),
                         Goto(x_d1, x_0, 3.0, 'Task'),
                         Goto(x_0, x_d2, 3.0, 'Task'),
                         Goto(x_d2, x_0, 3.0, 'Task')]
    
    def segments_for_line(self):
        # moves arm horizontally in front
        self.ignore = 1 # ignore spline from initial pos to home
        self.chain.setjoints(self.position0)
        x_0 = self.chain.ptip().flatten()

        x_SL1 = np.array([-0.3, -0.15, 0.15])
        x_SL2 = np.array([-0.3, 0.15, 0.15])
        self.x_SL = [x_SL1, x_SL2]

        self.segments = [Goto(x_0, x_SL1, 4.0, 'Task'),
                         Goto(x_SL1, x_SL2, 4.0, 'Task'),
                         Goto(x_SL2, x_SL1, 4.0, 'Task'),]

        
    def ikin_NR(self, xd, q_guess):
        # performs the Newton-Raphson algorithm to find the ikin
        xd = xd.reshape(5,1)
        q_guess = np.reshape(q_guess.copy(), (5,1))
        self.chain.setjoints(q_guess)

        q_g0 = q_guess.copy()

        for ctr in range(20):
            #J  = np.vstack((self.chain.Jv(),self.chain.Jw())) # shape (6,3)
            x = np.vstack((self.chain.ptip(), 
                           q_guess[1]-q_guess[2]+q_guess[3]-1.57, 
                           q_guess[0]-q_guess[4]))
            
            Jv = self.chain.Jv()
            J = np.vstack((Jv, np.array([[0.,1.,-1.,1.,0.],[1.,0.,0.,0.,-1.]])))

            e = xd - x # shape (3,1)
            q_guess += (np.linalg.pinv(J) @ e)
            self.chain.setjoints(q_guess)
            #print(np.linalg.norm(e))
            if np.linalg.norm(e) < 10**-6:
                break
            if ctr > 2: 
                self.get_logger().warn("Warning: IKIN did not converge")

        return np.array(q_guess).flatten()

    def gravity(self, pos):
        # calculates effort required to hold position
        # i.e., counters gravity
        self.coeffs = [0,0,0,-1.2]
        t1 = pos[1]
        t2 = pos[2]
        tau1 =  self.coeffs[0]*np.sin(t1+t2) +\
                self.coeffs[1]*np.cos(t1+t2) +\
                self.coeffs[2]*np.sin(t1) +\
                self.coeffs[3]*np.cos(t1)
        tau2 = self.coeffs[0]*np.sin(t1+t2) + self.coeffs[1]*np.cos(t1 + t2)
        return np.array([0.0, tau1, -tau2])
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.fnode.destroy_node()
        self.destroy_node()


    # Grab a single feedback - do not call this repeatedly.
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = np.array(list(fbkmsg.position))
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
        self.grabpos   = np.array(list(fbkmsg.position))
    
    def update(self, t):
        # once segment completed, move on to new segment
        if(t - self.t0 >= self.segments[self.cseg].duration()):
            # update for check_contact()
            self.flipping = False
            self.t0    = self.t0 + self.segments[self.cseg].duration()
            # wrap around to beginning of list
            self.cseg = (self.cseg+1) % len(self.segments)
            # skip over the ignores
            self.cseg += self.ignore if self.cseg==0 else 0
        
        cpos = np.copy(self.grabpos)

        # Decide what to do based on the space.
        if (self.segments[self.cseg].space() == 'Joint'):
            # Set the message positions/velocities as a function of time.
            (position, velocity) = \
                self.segments[self.cseg].evaluate(t-self.t0)
        else: # Task space
            # Get the position and velocity of the task spline
            (position, velocity) = self.segments[self.cseg].evaluate(t-self.t0)
            
            x = np.vstack((position.reshape(-1,1), np.array([[0.01],[0.01]])))
            # Find the angle to move to and assign it to guess, since it will be
            # used as the guess for the next angle
            #print('prev ', self.prev_pos)
            position = self.ikin_NR(x, np.reshape(self.prev_pos,(5,1)))
            #print(position.shape)
            # print(self.ikin_NR(position, self.grabpos))
            # Get the translation and jacobian for the new angle (stored in guess)
            # (T, J) = self.kin.fkin(self.guess)
            
            # Calculate the angular velocity by inverting J and multiplying with xdot
            # cmdmsg.velocity = np.matmul(np.linalg.inv(J[0:3]), velocity)
            
            # New position is just the new theta, so the guess
            #cmdmsg.position = self.guess
            
        # Send the command (with the current time).
        #cmdmsg.header.stamp = rospy.Time.now()
        #self.pub.publish(cmdmsg)
        self.prev_pos = position
        return position

    def gravity(self, pos):
        # calculates effort required to hold position
        # i.e., counters gravity
        self.coeffs = [0,3,-0.2,0]
        t1 = pos[1]
        t2 = pos[2]
        tau1 =  self.coeffs[0]*np.sin(t1+t2) +\
                self.coeffs[1]*np.cos(t1+t2) +\
                self.coeffs[2]*np.sin(t1) +\
                self.coeffs[3]*np.cos(t1)
        tau2 = self.coeffs[0]*np.sin(t1+t2) + self.coeffs[1]*np.cos(t1 + t2)
        return np.array([0.0, tau1, -tau2])
    
    def check_contact(self, curr_pos, time):
        # compares current position with previous position
        # if the curent position is past an allowable threshold, the arm flips
        if self.flipping:
            return
        d_pos0 = np.abs(self.prev_pos[0]-curr_pos[0])
        d_pos1 = np.abs(self.prev_pos[1]-curr_pos[1])
        d_pos2 = np.abs(self.prev_pos[2]-curr_pos[2])
        max_d = 0.03
        if d_pos0 > max_d or d_pos1 > max_d or d_pos2 > max_d:
            self.get_logger().info(f"contact detected at {time}. effort:  {[d_pos0, d_pos1, d_pos2]}")   
            self.cb_flip('std_msgs/Empty')
                   
    # not in-use right now (from Week 4 goals)
    def cb_number(self, msg):
        self.A = msg.data
        self.get_logger().info("received: %r" % msg.data)

    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        timesec = self.get_clock().now().nanoseconds/10**9 - self.t0
        t = self.get_clock().now().nanoseconds/(10**9)

        sendPos = self.update(t)
        #print(self.gravity(self.actpos))
        #print("")

        # print forward kinematics position
        # fkin = self.chain.fkin(self.grabpos)
        # print(np.squeeze(fkin[0]))

        # if there's contact detected, the arm flips
        #self.check_contact(self.grabpos, t)

        sendEff = self.gravity(self.grabpos).tolist()
        sendEff.extend([0.0,0.0])

        #print(sendPos)
        self.get_logger().info(str(self.grabpos))


        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder','elbow', 'wrist', 'finger']
        # self.cmdmsg.position     = [0.0, 0.0, 0.0, 0.0, 0.0] # [float("nan"), float("nan"), float("nan"), float("nan"), float("nan")]
        self.cmdmsg.position     =  sendPos.tolist()
        self.cmdmsg.velocity     = []
        #self.cmdmsg.velocity     = [float("nan"), float("nan"), float("nan")]
        self.cmdmsg.effort       = [] #sendEff#.tolist().extend([0,0])
        #self.cmdmsg.effort       = sendEff.tolist()
        #self.cmdpub.publish(self.cmdmsg)



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
