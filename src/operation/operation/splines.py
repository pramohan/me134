#!/usr/bin/env python3
#
#   better_trajectory_generator.py
#
#   Create a continuous stream of joint positions, to be animated.
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
#import rospy
import math
import numpy as np

from sensor_msgs.msg   import JointState


#
#  Trajectory Segment Objects
#
#  All these segments are set up to start at time 0, running to some
#  specified end time.  They each store whichever internal parameters
#  they need.  And provide both an evaluate(t) and duration() method.
#  Using numpy, they work with arrays of joints.
#

class CubicSpline:
    # Initialize.
    def __init__(self, p0, v0, pf, vf, T, space=['Joint', 0]):
        # Precompute the spline parameters.
        self.T = T
        self.a = p0
        self.b = v0
        self.c =  3*(pf-p0)/T**2 - vf/T    - 2*v0/T
        self.d = -2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2
        # Save the space
        self.usespace = space

    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return(self.T)

    def update(self, pc, vc, pfn, vfn, T=None):
        if T is not None:
            self.T = T
        T = self.T
        self.a = pc
        self.b = vc
        self.c = 3*(pfn-pc)/T**2 - vfn/T    - 2*vc/T
        self.d = -2*(pfn-pc)/T**3 + vfn/T**2 +   vc/T**2

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t +   self.c * t**2 +   self.d * t**3
        v =          self.b     + 2*self.c * t    + 3*self.d * t**2
        return (p,v)

class Goto(CubicSpline):
    # Use zero initial/final velocities (of same size as positions).
    def __init__(self, p0, pf, T, space=['Joint', 0]):
        CubicSpline.__init__(self, p0, 0*p0, pf, 0*pf, T, space)
    
    def update(self, pc, vc, pfn, T=None):
        if T is not None:
            CubicSpline.update(self, pc, vc, pfn, 0, T)
        else:
            CubicSpline.update(self, pc, vc, pfn, 0)

class Hold(Goto):
    # Use the same initial and final positions.
    def __init__(self, p, T, space=['Joint', 0]):
        Goto.__init__(self, p, p, T, space)

class Stay(Hold):
    # Use an infinite time (stay forever).
    def __init__(self, p, space=['Joint', 0]):
        Hold.__init__(self, p, math.inf, space)

