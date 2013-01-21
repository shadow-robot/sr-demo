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
Denavith-Hartenberg model of the thumb of the shadow robot ethercat hand

**********************************************************************************

                      ++++++++++++++++++++++++++++++++
                      |  n  |  a |  alfa | d | theta |
                      ++++++++++++++++++++++++++++++++
                      |  1  |  0 | -pi/2 | 0 |   0   |
                      |  2  | a2 |   0   | 0 |  pi/2 |
                      |  3  |  0 | -pi/2 | 0 |   0   |
                      |  4  | a4 |   0   | 0 |   0   |
                      |  5  | a5 |   0   | 0 |   0   |
                      ++++++++++++++++++++++++++++++++  

**********************************************************************************
"""

from math import cos, sin, pi

a2 = 0.038   # [m]
a4 = 0.032   # [m]
a5 = 0.0275  # [m]

def th_FK(q):
    """
    This function computes the forward kinematics of the thumb of the shadow robot hand
    on the base of the D-H parameters described above.

    Args:
           *joint position* : joint position   
        
    Returns:
           *Transformation matrix*: Transformation matrix of the finger (base to tip)
    """
        

    m11 = cos(q[4])*(cos(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2])) + sin(q[0])*sin(q[3])) - sin(q[4])*(sin(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2])) - cos(q[3])*sin(q[0])) 
    m12 = - cos(q[4])*(sin(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2])) - cos(q[3])*sin(q[0])) - sin(q[4])*(cos(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2])) + sin(q[0])*sin(q[3]))
    m13 = -sin((q[1]+pi/2) + q[2])*cos(q[0])
    m14 = a2*cos(q[0])*cos((q[1]+pi/2)) + a4*sin(q[0])*sin(q[3]) + a5*cos(q[4])*(cos(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2])) + sin(q[0])*sin(q[3])) - a5*sin(q[4])*(sin(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2])) - cos(q[3])*sin(q[0])) + a4*cos(q[3])*(cos(q[0])*cos((q[1]+pi/2))*cos(q[2]) - cos(q[0])*sin((q[1]+pi/2))*sin(q[2]))
    
    m21 = cos(q[4])*(cos(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) - cos(q[0])*sin(q[3])) - sin(q[4])*(sin(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) + cos(q[0])*cos(q[3]))
    m22 = - sin(q[4])*(cos(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) - cos(q[0])*sin(q[3])) - cos(q[4])*(sin(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) + cos(q[0])*cos(q[3])) 
    m23 = -sin((q[1]+pi/2) + q[2])*sin(q[0])
    m24 = a4*cos(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) + a2*cos((q[1]+pi/2))*sin(q[0]) - a4*cos(q[0])*sin(q[3]) + a5*cos(q[4])*(cos(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) - cos(q[0])*sin(q[3])) - a5*sin(q[4])*(sin(q[3])*(cos((q[1]+pi/2))*cos(q[2])*sin(q[0]) - sin(q[0])*sin((q[1]+pi/2))*sin(q[2])) + cos(q[0])*cos(q[3]))

    m31 = -cos(q[3] + q[4])*sin((q[1]+pi/2) + q[2])
    m32 = sin((q[1]+pi/2) + q[2])*sin(q[3] + q[4])
    m33 = -cos((q[1]+pi/2) + q[2])
    m34 = - a2*sin((q[1]+pi/2)) - a5*cos(q[3] + q[4])*sin((q[1]+pi/2) + q[2]) - a4*sin((q[1]+pi/2) + q[2])*cos(q[3])

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
        q = [float(sys.argv[1]), float(sys.argv[2]),float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])]
        FK = th_FK(q) 
        
        print "--Thumb tip position with respect to its base--" 
        print "X: ",FK[0][3]
        print "Y: ",FK[1][3]
        print "Z: ",FK[2][3]


