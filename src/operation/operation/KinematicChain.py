import numpy as np
import rclpy

from rclpy.node             import Node
from sensor_msgs.msg        import JointState
from rclpy.qos                import QoSProfile, DurabilityPolicy
from std_msgs.msg             import String
from urdf_parser_py.urdf      import Robot

from operation.TransformHelpers import *

class KinematicChainData():
    def __init__(self):
        self.type = []          # List of 'revolute' or 'prismatic'
        self.e    = []          # List of 3x1 joint axes
        self.T    = []          # List of 4x4 transforms
        self.Ttip = None        # 4x4 tip transform

class KinematicChain():
    # Helper functions for printing info and errors.
    def info(self, string):
        # self.node.get_logger().info("KinematicChain: " + string)
        pass
    def error(self, string):
        self.node.get_logger().error("KinematicChain: " + string)
        raise Exception(string)
    
    # Initialization.
    def __init__(self, node, baseframe, tipframe, expectedjointnames):
        # Store the node (for the printing functions).
        self.node = node

        # Create a temporary subscriber to receive the URDF.  We use
        # the TRANSIENT_LOCAL durability, so that we see the last
        # message already published (if any).
        self.info("Waiting for the URDF to be published...")
        self.urdf = None
        def cb(msg):
            self.urdf = msg.data
        topic   = '/robot_description'
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        sub = node.create_subscription(String, topic, cb, quality)
        while self.urdf is None:
            rclpy.spin_once(node)
        node.destroy_subscription(sub)

        # Convert the URDF string into a Robot object and report.
        robot     = Robot.from_xml_string(self.urdf)
        self.name = robot.name
        self.info("URDF Robot '%s'" % self.name)
        
        # Convert the URDF string into a Robot object and parse to
        # create the list of joints from the base frame to the tip
        # frame.  Search backwards, as this could be a tree structure.
        # Meaning while a parent may have multiple children, every
        # child has only one parent!  That makes the chain unique.
        self.joints = []
        frame = tipframe
        while (frame != baseframe):
            joint = next((j for j in robot.joints if j.child == frame), None)
            if (joint is None):
                self.error("Unable find joint connecting to '%s'" % frame)
            if (joint.parent == frame):
                self.error("Joint '%s' connects '%s' to itself" %
                           (joint.name, frame))
            self.joints.insert(0, joint)
            frame = joint.parent

        # Report we found.
        self.dofs = sum(1 for j in self.joints if j.type != 'fixed')
        self.info("%d total joints in URDF, %d active DOFs:" %
             (len(self.joints), self.dofs))
        dof = 0
        for i,joint in enumerate(self.joints):
            if joint.type == 'fixed':
                self.info("Joint #%d fixed      '%s'" % (i, joint.name))
            elif joint.type == 'continuous' or joint.type == 'revolute':
                self.info("Joint #%d rot DOF #%d '%s'" % (i, dof, joint.name))
                dof = dof+1
            elif joint.type == 'prismatic':
                self.info("Joint #%d lin DOF #%d '%s'" % (i, dof, joint.name))
                dof = dof+1
            else:
                self.error("Joint '%s' has unknown type '%s'" %
                           (joint.name, joint.type))

        # Confirm this matches the expectation
        jointnames = [j.name for j in self.joints if j.type != 'fixed']
        if jointnames != list(expectedjointnames):
            self.error("Chain does not match the expected names: " +
                  str(expectedjointnames))

        # And pre-compute the kinematic chain data at the zero position.
        self.setjoints(np.zeros((self.dofs,1)))


    # Update the joint positions.  This recomputes the chain data values!
    def setjoints(self, q):
        # Check the number of joints
        if (len(q) != self.dofs):
            self.error("Number of joint angles (%d) does not match URDF (%d)",
                       len(q), self.dofs)

        # Remove any past data.
        self.data = KinematicChainData()

        # Initialize the T matrix to walk up the chain, w.r.t. world frame.
        T = np.eye(4)
        
        # Walk the chain, one URDF <joint> entry at a time, being
        # 'fixed' (just a fixed transform), 'continuous'/'revolute'
        # (both rotational, the latter with joint limits which we
        # ignore), or 'prismatic'.  By design, the URDF entries are
        # only the step-by-step transformations.  That is, the
        # information is *not* in world frame.  We have to append to
        # the chain...
        dof = 0
        for joint in self.joints:
            if (joint.type == 'fixed'):
                # Just append the fixed transform
                T = T @ T_from_URDF_origin(joint.origin)
                
            elif (joint.type == 'continuous') or (joint.type == 'revolute'):
                # Grab the joint axis in the local frame.
                elocal = e_from_URDF_axis(joint.axis)

                # First append the fixed transform, then the rotational
                # transform.  The joint angle comes from q-vector.
                T = T @ T_from_URDF_origin(joint.origin)
                T = T @ T_from_Rp(Rote(elocal, q[dof]), pzero())

                # Compute the joint axis w.r.t. world frame.
                e = R_from_T(T) @ elocal
    
                # Save the transform and advance the active DOF counter.
                self.data.type.append('revolute')
                self.data.e.append(e)
                self.data.T.append(T)
                dof += 1

            elif (joint.type == 'prismatic'):
                # Grab the joint axis in the local frame.
                elocal = e_from_URDF_axis(joint.axis)

                # First append the fixed transform, then the translational
                # transform.  The joint displacement comes from q-vector.
                T = T @ T_from_URDF_origin(joint.origin)
                T = T @ T_from_Rp(Reye(), elocal * q[dof])

                # Compute the joint axis w.r.t. world frame.
                e = R_from_T(T) @ elocal

                # Save the transform and advance the active DOF counter.
                self.data.type.append('prismatic')
                self.data.e.append(e)
                self.data.T.append(T)
                dof += 1

            else:
                # There shouldn't be any other types...
                self.error("Unknown Joint Type: %s", joint.type)

        # Also save the tip transform.
        self.data.Ttip = T


    # Extract the position/orientation data from the already computed
    # kinematic chain data in self.data!
    def ptip(self):
        return p_from_T(self.data.Ttip)
    def Rtip(self):
        return R_from_T(self.data.Ttip)
    def Ttip(self):
        return self.data.Ttip

    # Extract the Jacobian data.
    def Jv(self):
        J = np.zeros((3,self.dofs))
        for dof in range(self.dofs):
            if (self.data.type[dof] == 'revolute'):
                dp = p_from_T(self.data.Ttip) - p_from_T(self.data.T[dof])
                J[:,dof:dof+1] = cross(self.data.e[dof], dp)
            else:
                J[:,dof:dof+1] = self.data.e[dof]
        return J
    def Jw(self):
        J = np.zeros((3,self.dofs))
        for dof in range(self.dofs):
            if (self.data.type[dof] == 'revolute'):
                J[:,dof:dof+1] = self.data.e[dof]
            else:
                J[:,dof:dof+1] = np.zeros((3,1))
        return J

    # All-in-one call.
    def fkin(self, q):
        # Compute the chain data.
        self.setjoints(q)

        # Extract and return the data.
        return (self.ptip(), self.Rtip(), self.Jv(), self.Jw())
