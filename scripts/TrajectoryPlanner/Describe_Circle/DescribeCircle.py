#!/usr/bin/env python
# this line is just used to define the type of document

import numpy

class DescribeCircle():

    def __init__(self,offset,Rotation,parameters=None):

        self.offset   = offset

        self.Rotation = Rotation

        # radius
        self.r = parameters[0]
        # omega
        self.w = parameters[1]


    # trajectory of circle
    def traj_des(self,t):

        from numpy import cos as c
        from numpy import sin as s

        r = self.r
        w = self.w
        
        p = r*w**0*numpy.array([ c(w*t),-s(w*t),0.0]);
        v = r*w**1*numpy.array([-s(w*t),-c(w*t),0.0]);
        a = r*w**2*numpy.array([-c(w*t), s(w*t),0.0]);
        j = r*w**3*numpy.array([ s(w*t), c(w*t),0.0]);
        s = r*w**4*numpy.array([ c(w*t),-s(w*t),0.0]);
        
        return numpy.concatenate([p,v,a,j,s])

    def output(self,t):
        return self.add_offset_and_rot(self.traj_des(t))


    def add_offset_and_rot(self,position):

        pos_out = numpy.zeros(3*5)
        
        R = self.Rotation
        
        pos_out[0:3]   = R.dot(position[0:3])
        pos_out[3:6]   = R.dot(position[3:6])
        pos_out[6:9]   = R.dot(position[6:9])
        pos_out[9:12]  = R.dot(position[9:12])
        pos_out[12:15] = R.dot(position[12:15])

        pos_out[0:3] = pos_out[0:3] + self.offset   

        return pos_out