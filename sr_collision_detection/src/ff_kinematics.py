#! /usr/bin/env python

#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

"""
Denavith-Hartenberg model of the first/middle/ring finger of the shadow robot ethercat hand

**********************************************************************************

                      ++++++++++++++++++++++++++++++++
                      |  n  |  a |  alfa | d | theta |
                      ++++++++++++++++++++++++++++++++
                      |  1  |  0 | -pi/2 | 0 |   0   |
                      |  2  | a2 |   0   | 0 |   0   |
                      |  3  | a3 |   0   | 0 |   0   |
                      |  4  | a4 |   0   | 0 |   0   |
                      ++++++++++++++++++++++++++++++++  

**********************************************************************************
"""

from math import cos, sin, pi

a2 = 0.045; # [m]
a3 = 0.025; # [m]
a4 = 0.026; # [m]

def ff_FK(q):
    """
    This function computes the forward kinematics of the first finger of the shadow robot hand
    on the base of the D-H parameters described above.
    Since the kinematic model of the FF, the MF and the RF is the same, this function can be used
    with all the aforementioned fingers.
    
    .. note:: in the following formula q[0] has been replaced by (-q[0]) for taking into account
              the fact the the direction of z0 in the D-H model is opposed to the one in the real hand 
    
    Args:
           *joint position* : joint position [ff4 ff3 ff2 ff1]  
        
    Returns:
           *Transformation matrix*: Transformation matrix of the finger (base to tip)
    """

    m11 = cos((-q[0]))*cos(q[1] + q[2] + q[3]) 
    m12 = -cos((-q[0]))*sin(q[1] + q[2] + q[3])
    m13 = -sin((-q[0]))
    m14 = cos((-q[0]))*(a3*cos(q[1] + q[2]) + a2*cos(q[1]) + a4*cos(q[1] + q[2] + q[3]))

    m21 = cos(q[1] + q[2] + q[3])*sin((-q[0]))
    m22 = cos((-q[0]) + q[1] + q[2] + q[3])/2 - cos(q[1] - (-q[0]) + q[2] + q[3])/2
    m23 = cos((-q[0]))
    m24 = sin((-q[0]))*(a3*cos(q[1] + q[2]) + a2*cos(q[1]) + a4*cos(q[1] + q[2] + q[3]))

    m31 = -sin(q[1] + q[2] + q[3])
    m32 = -cos(q[1] + q[2] + q[3])
    m33 = 0
    m34 =  - a3*sin(q[1] + q[2]) - a2*sin(q[1]) - a4*sin(q[1] + q[2] + q[3])

    m41 = 0
    m42 = 0
    m43 = 0
    m44 = 1
    
    forward_kinematics = [ [ m11, m12, m13, m14 ], 
                           [ m21, m22, m23, m24 ], 
                           [ m31, m32, m33, m34 ], 
                           [ m41, m42, m43, m44 ] ]


    return forward_kinematics


if __name__ == '__main__':
    
        # Compute the forward kinematics for a set of joints
        
        import sys
        q = [float(sys.argv[1]), float(sys.argv[2]),float(sys.argv[3]), float(sys.argv[4])]
        FK = ff_FK(q) 
        
        print "--First/Middle/Ring finger tip position with respect to its base--" 
        print "X: ",FK[0][3]
        print "Y: ",FK[1][3]
        print "Z: ",FK[2][3] 
        



