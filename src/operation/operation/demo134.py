#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy
import cv2
import cv_bridge
import time

from rclpy.node             import Node
from sensor_msgs.msg        import JointState
from sensor_msgs.msg        import CameraInfo
from sensor_msgs.msg        import Image
from std_msgs.msg           import Empty
from rclpy.qos              import QoSProfile, DurabilityPolicy
from std_msgs.msg           import String, Int32, Int64, Float32MultiArray
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


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization code run at the beginning
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create aruco location subscriber
        self.aruco_loc = np.array([])
        self.aruco_sub = self.create_subscription(Float32MultiArray, '/aruco_location', self.cb_aruco, 10)
        self.get_logger().info("Waiting for camera publisher...")
        while len(self.aruco_loc) == 0:
            rclpy.spin_once(self)
        self.get_logger().info(str(self.aruco_loc))

        # Create joint state subscriber and save the initial position
        self.grabpos = np.array([])
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.cb_pos, 10)


        self.get_logger().info("Waiting for initial position...")
        while len(self.grabpos) == 0:
            rclpy.spin_once(self)
        self.position0 = np.copy(self.grabpos)
        self.get_logger().info("Initial positions: %r" % self.position0)

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Micro-ros parameters
        self.pot_position = np.array([])
        self.prev_angle = 0

        self.c_touch = 0
        self.em_but = 0
        self.ctrl_mode = 0
        self.joystick = np.zeros(3)

        self.jvnom = .1/100

        # Micro-ros subscribers and publishers
        self.pot_sub = self.create_subscription(Int64, '/pot_val', self.cb_pot, 10)
        self.sensor_sub = self.create_subscription(Int64, '/sensors', self.cb_sensors, 10)
        self.EM_pub = self.create_publisher(Int32, '/EM_enable', 10)

        self.get_logger().info("Waiting for initial potentiometer position...")
        while len(self.pot_position) == 0:
            rclpy.spin_once(self)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_commands subscriber...")
        while(not self.count_subscribers('/joint_commands')):
            pass
        
        self.get_logger().info("Waiting for a /EM_enable subscriber...")
        while(not self.count_subscribers('/EM_enable')):
            pass

        # Create an object for the kinematic chain
        self.fnode = Node("fkin")
        self.chain = KinematicChain(self.fnode, 'world', 'tip', ['base', 'shoulder', 'elbow', 'wrist', 'finger'])
        self.chain.setjoints(self.position0)
        self.q_safe = np.array([0.0, 0.0, .0, 1.57, .0])
        self.pinv_gam = 1

        # Grab movement segments for task
        self.mode = "auto"

        self.q_safe_joystick = np.array([-0.04132938, -0.25080585,  0.43648836,  2.2944777 ,  0.70725644])
        self.q_d_joystick = np.copy(self.q_safe_joystick)
        self.q_safe_dropoff = np.array([-0.95291615, -1.03956223, -0.65856385,  2.2692852 , -0.42099997])
        self.p_safe_dropoff = np.array([-0.35, 0.40, 0.05])
        #self.segments_for_aruco()
        #self.segments_for_x()
        #self.segments_for_line()
        #self.segments_for_pot(self.position0)
        #self.segments_for_joy(self.position0)
        self.segments_for_remove(self.position0)
        self.cseg = 0
        self.prev_pos = self.position0

        self.em_int = 0

        self.t0= self.get_clock().now().nanoseconds/10**9

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))

    def publish_EM(self, number):
        if number != 0 and number != 1:
            self.get_logger().warn("beech, the number is either 0 or 1")
        self.EM_pub.publish(Int32(data = number))

    # Aruco position callback
    def cb_aruco(self, msg):
        self.aruco_loc = np.array(msg.data)
    
    # Joint states callback
    def cb_pos(self, fbkmsg):
        self.grabpos   = np.array(list(fbkmsg.position))
    
    # Micro ros sensor callback
    def cb_sensors(self, msg):
        s_raw = msg.data
        zdir = s_raw%10
        zdir = -1 if zdir == 0 else 1
        s_raw = s_raw//10
        self.c_touch = s_raw%10
        s_raw = s_raw//10
        self.em_but = s_raw%10
        s_raw = s_raw//10
        self.ctrl_mode = s_raw%10 # 1 for joystick 0 for pots
        s_raw = s_raw//10
        self.joystick[-1] = (s_raw%10) * zdir
        s_raw = s_raw//10
        self.joystick[-2] = -(((s_raw%1000)  // 200) -2)
        self.joystick[-3] = -(((s_raw//1000) // 200) -2)
        self.joystick *= self.jvnom

    # Micro ros potentiometer callback
    def cb_pot(self, msg):
        pv_raw = msg.data
        joints = []
        for i in range(5):
            joints.append(pv_raw%1000 + 0.)
            pv_raw = pv_raw // 1000

        joints = np.array(joints)
        joints = joints/1000. -0.5
        joints *= np.pi/1.5
        joints[-2] += 1.57
        joints[-1] *= 2

        if len(self.pot_position) == 0:
            self.pot_position = np.copy(joints)
        else:
            self.pot_position += 1./RATE*(joints-self.pot_position)
    
    # Create segments to aruco reading
    def segments_for_aruco(self):
        self.ignore = 1 # ignore spline from initial pos to home
        self.chain.setjoints(self.q_safe)
        x_0 = self.chain.ptip().flatten()

        x_d1 = self.aruco_loc[0:3]
        x_d1_altitude = x_d1.copy()
        x_d1_altitude[2] += 0.05

        x_d2 = self.aruco_loc[3:]
        x_d2_altitude = x_d2.copy()
        x_d2_altitude[2] += 0.05

        # self.get_logger().info(str(x_d1))
        # self.get_logger().info(str(x_d2))

        self.segments = [Goto(self.position0, self.q_safe, 3.0, ['Joint', 0]),
                         Goto(x_0, x_d1_altitude, 3.0, ['Task', 0]),
                         Goto(x_d1_altitude, x_d1, 2.0, ['Task', 1]),
                         Hold(x_d1, 0.7, ['Task', 1]),
                         Goto(x_d1, x_d1_altitude, 2.0, ['Task', 1]),
                         Goto(x_d1_altitude, x_d2_altitude, 3.0, ['Task', 1]),
                         Goto(x_d2_altitude, x_d2, 2.0, ['Task', 1]),
                         Hold(x_d2, 0.7, ['Task', 0]),
                         Goto(x_d2, x_d2_altitude, 2.0, ['Task', 0]),
                         Goto(x_d2_altitude, x_0, 3.0, ['Task', 0])]
    
    def segments_for_remove(self, p0):
        self.chain.setjoints(p0)
        self.ignore = 1 # ignore spline from initial pos to home
        self.segments = [Goto(p0, self.q_safe_dropoff, 2.0, ['Joint', 0])]

    def segments_for_pot(self, p0):
        self.ignore = 1 # ignore spline from initial pos to home
        self.segments = [Goto(p0, self.q_safe, 2.0, ['Joint', -1]),
                         Goto(self.q_safe, self.pot_position, 2.0, ['Joint', -1])]

    def segments_for_joy(self, p0):
        self.q_d_joystick = np.copy(self.q_safe_joystick)
        self.ignore = 1 # ignore spline from initial pos to home
        self.segments = [Goto(p0, self.q_safe_joystick, 2.0, ['Joint', -1])]

    def segments_for_x(self):
        # moves arm to each point
        self.ignore = 1 # ignore spline from initial pos to home
        self.chain.setjoints(self.q_safe)
        x_0 = self.chain.ptip().flatten()
        x_d1 = np.array([-0.6, 0, 0.03]) # the middle X on the table
        x_d2 = np.array([-0.3, 0.165, 0.03]) # left X
        # x_d3 = np.array([-0.075, 0.105, 0.02]) # right X

        self.segments = [Goto(self.position0, self.q_safe, 3.0, 'Joint'),
                         Goto(x_0, x_d1, 3.0, 'Task'),
                         Goto(x_d1, x_0, 3.0, 'Task'),
                         Goto(x_0, x_d2, 3.0, 'Task'),
                         Goto(x_d2, x_0, 3.0, 'Task')]

        
    def ikin_NR(self, xd, q_g0):
        # performs the Newton-Raphson algorithm to find the ikin
        xd = np.copy(xd).reshape(5,1)
        q_guess = np.reshape(np.copy(q_g0), (5,1))
        self.chain.setjoints(q_guess)

        for ctr in range(7):
            #J  = np.vstack((self.chain.Jv(),self.chain.Jw())) # shape (6,3)
            x = np.vstack((self.chain.ptip(), 
                           q_guess[1]-q_guess[2]+q_guess[3]-1.57, 
                           q_guess[0]-q_guess[4]))
            
            J = np.vstack((self.chain.Jv(),
                            np.array([0.,1.,-1.,1.,0.]),
                            np.array([1.,0.,0.,0.,-1.])))

            e = xd - x # shape (3,1)

            J_inv = np.linalg.inv((J.T @ J) + (np.eye(J.shape[1]) * self.pinv_gam**2)) @ J.T
            q_guess += (J_inv @ e)
            self.chain.setjoints(q_guess)
            # self.get_logger().info(str(np.linalg.norm(e)))
            if np.linalg.norm(e) < 10**-6:
                break
            # if ctr > 2:
            #     self.get_logger().warn("IKIN not converge")
        

        return np.array(q_guess).flatten()
    
    def gravity(self, pos):
        self.coeffs = np.array([2.21423, 3.39517, 0.59880, 0.31392])
        t1 = pos[1]
        t2 = pos[2]
        t3 = pos[3]
        
        tau3 = self.coeffs[2]*np.sin(t3-t2+t1) - self.coeffs[3]*np.cos(t3-t2+t1) 
        tau2 =-self.coeffs[1]*np.cos(t2-t1) - tau3
        tau1 =-self.coeffs[0]*np.sin(t1) - tau2
        return np.array([0.,tau1,tau2,tau3,0.])
    
    def shutdown(self):
        self.publish_EM(0)
        # No particular cleanup, just shut down the node.
        self.fnode.destroy_node()
        self.destroy_node()

    def update_pot(self, t):
        if self.ctrl_mode == 1:
            self.mode = 'joy'
            self.segments = []
            self.segments_for_joy(self.grabpos)
            self.t0 = t
            self.cseg = 0

        if self.cseg >= len(self.segments):
            return self.pot_position, []
        (position, velocity) = self.segments[self.cseg].evaluate(t-self.t0)

        if(t - self.t0 >= self.segments[self.cseg].duration()):
            self.t0    = self.t0 + self.segments[self.cseg].duration()
            self.cseg = (self.cseg+1)
        
        return position, []
    
    def update_joy(self, t):
        if self.ctrl_mode == 0:
            self.mode = 'pot'
            self.segments = []
            self.segments_for_pot(self.grabpos)
            self.t0 = t
            self.cseg = 0
        
        if self.cseg < len(self.segments):
            (position, velocity) = self.segments[self.cseg].evaluate(t-self.t0)

            if(t - self.t0 >= self.segments[self.cseg].duration()):
                self.t0    = self.t0 + self.segments[self.cseg].duration()
                self.cseg = (self.cseg+1)
            
            #self.q_d_joystick[-1] = self.pot_position[-1]
            return position, []
        
        self.chain.setjoints(self.q_d_joystick)
        xc = self.chain.ptip().flatten()
        xd = xc + self.joystick

        QDJ = self.q_d_joystick.copy()
        QDJ[-1] = self.pot_position[-1]
        if np.sum(self.joystick) == 0: return QDJ, [0.,0.,0.,0.,0.02]
        xd = np.vstack((xd.reshape(-1,1), np.array([[0.],[0.]])))
        joints = self.ikin_NR(xd, np.copy(self.q_d_joystick))

        self.q_d_joystick = np.copy(joints)

        #self.get_logger().info(str(xc))
        #self.get_logger().info(str(xd))

        joints[-1] = self.pot_position[-1]
        return joints, []
    
    def append_remove_chunk(self, P_rem):
        x_d1_a = P_rem.copy() + np.array([0,0,0.05])
        self.chain.setjoints(self.q_safe_dropoff)
        p_safe = self.chain.ptip().flatten()
        self.segments.append(Goto(p_safe, self.p_safe_dropoff, 3.0, ['Task', 0]))
        self.segments.append(Goto(self.p_safe_dropoff, x_d1_a, 3.0, ['Task', 0]))
        self.segments.append(Goto(x_d1_a, P_rem, 3.0, ['Task', 1]))
        self.segments.append(Hold(P_rem, 0.7, ['Task', 1]))
        self.segments.append(Goto(P_rem, x_d1_a, 3.0, ['Task', 1]))
        self.segments.append(Goto(x_d1_a, self.p_safe_dropoff, 3.0, ['Task', 1]))
        self.segments.append(Hold(self.p_safe_dropoff, 0.7, ['Task', 0]))
        self.segments.append(Goto(self.p_safe_dropoff, p_safe, 3.0, ['Task', 0]))

    def update_remove(self, t):
        if(t - self.t0 >= self.segments[self.cseg].duration()):
            self.t0    = self.t0 + self.segments[self.cseg].duration()
            self.cseg = self.cseg +1

            if self.cseg >= len(self.segments):
                if len(self.aruco_loc) == 0:
                    self.t0 = t
                    self.cseg = 0
                    self.segments = []
                    if self.ctrl_mode == 0:
                        self.mode = 'pot'
                        self.segments_for_pot(self.grabpos)
                    elif self.ctrl_mode == 1:
                        self.mode = 'joy'
                        self.segments_for_joy(self.grabpos)
                    return self.grabpos, []
                else:
                    P = np.array([self.aruco_loc[0], self.aruco_loc[1], 0.01])
                    self.append_remove_chunk(P)
            
                    #self.get_logger().info("sending to")
                    #self.get_logger().info(str(P))

            magnet = self.segments[self.cseg].space()[1]
            if magnet != -1: self.em_int = magnet
        
        (position, velocity) = self.segments[self.cseg].evaluate(t-self.t0)
        #self.get_logger().info(str(position))
        if (self.segments[self.cseg].space()[0] == 'Task'):
            x = np.vstack((position.reshape(-1,1), np.array([[0.01],[0.01]])))
            #self.get_logger().info(str(x))
            position = self.ikin_NR(x, np.reshape(self.prev_pos,(5,1)))
        
            self.prev_pos = position
        return position, []


    def update(self, t):
        if self.mode == 'pot': return self.update_pot(t)
        if self.mode == 'joy': return self.update_joy(t)
        if self.mode == 'auto': return self.update_remove(t)
        
        # once segment completed, move on to new segment
        if(t - self.t0 >= self.segments[self.cseg].duration()):
            self.t0    = self.t0 + self.segments[self.cseg].duration()
            self.cseg = (self.cseg+1) % len(self.segments)
            self.cseg += self.ignore if self.cseg==0 else 0
            magnet = self.segments[self.cseg].space()[1]
            if magnet != -1: self.em_int = magnet
                
        # Decide what to do based on the space.
        (position, velocity) = self.segments[self.cseg].evaluate(t-self.t0)
        if (self.segments[self.cseg].space()[0] == 'Task'):
            (position, velocity) = self.segments[self.cseg].evaluate(t-self.t0)
            x = np.vstack((position.reshape(-1,1), np.array([[0.01],[0.01]])))
            position = self.ikin_NR(x, np.reshape(self.prev_pos,(5,1)))
        
            self.prev_pos = position
        return position, []

    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        timesec = self.get_clock().now().nanoseconds/10**9 - self.t0
        t = self.get_clock().now().nanoseconds/(10**9)

        sendPos, sendVel = self.update(t)
        sendPos = sendPos.tolist()
        sendEff = self.gravity(self.grabpos).tolist()

        # self.get_logger().info("haskjg")
        # self.get_logger().info(str(sendPos))
        # self.get_logger().info(str(sendVel))
        # self.get_logger().info(str(sendEff))
        #self.get_logger().info(str(self.pot_position))
        #self.get_logger().info(str(self.joystick))

        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name         = ['base', 'shoulder','elbow', 'wrist', 'finger']
        self.cmdmsg.position     = sendPos
        self.cmdmsg.velocity     = sendVel
        self.cmdmsg.effort       = sendEff
        self.cmdpub.publish(self.cmdmsg)


        if self.mode == 'joy' or self.mode == 'pot':
            self.publish_EM(self.em_but)
        else:
            self.publish_EM(self.em_int)


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
