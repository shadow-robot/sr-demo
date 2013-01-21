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

from ff_kinematics import ff_FK
from th_kinematics import th_FK

from math import pi, sqrt, radians

import roslib; roslib.load_manifest('sr_collision_detection')
import rospy


def collision_detection(q1, q2):
    """
    This function detects whether there is a collision between the tip of the thumb and
    the tip of the middle finger.
    n.b. The centre of the wrist has been selected as reference frame for both the fingers 

    Args:
           *q1* : joint position of the middle finger [ff4 ff3 ff2 ff1]
           *q2* : joint position of the thumb [th5 th4 th3 th2 th1]
        
        
    Returns:
           *pos_MF*: x, y and z position of the middle finger with respect to the reference frame    
           *pos_thumb*: x, y and z position of the thumb with respect to the reference frame
           
    """
    mf_offset = [0.099, 0.000, 0.000] # Offset between the base of the MF and the reference frame  
    th_offset = [0.029, 0.022, -0.0085] # Offset between the base of the TH and the reference frame

    fk_MF = ff_FK(q1) # Forward kinematics of the MF
   
    # Compute the position of the MF tip with respect to the reference frame 
    x_pos_mf = mf_offset[0] + fk_MF[0][3]
    y_pos_mf = mf_offset[1] + fk_MF[1][3]
    z_pos_mf = mf_offset[2] + fk_MF[2][3]

    

    print "--Middle finger tip position with respect to the reference frame--" 
    print "X: ", x_pos_mf
    print "Y: ", y_pos_mf
    print "Z: ", z_pos_mf

    fk_TH = th_FK(q2) # Forward kinematics of the TH
   
    # Compute the position of the MF tip with respect to the reference frame 
    # n.b the thumb position is rotated for taking into account the difference
    # between the base frame of the thumb and the reference frame by means of the
    # following formula --> rot_y(pi/2)*rot_x(-pi/4)*[X_th Y_th Z_th]'
    x_pos_th = th_offset[0] + (-0.7071*fk_TH[1][3] - 0.7071*fk_TH[2][3])
    y_pos_th = th_offset[1] + (0.7071*fk_TH[1][3] - 0.7071*fk_TH[2][3])
    z_pos_th = th_offset[2] + (fk_TH[0][3])

    print "--Thumb tip position with respect to the reference frame--" 
    print "X: ", x_pos_th
    print "Y: ", y_pos_th
    print "Z: ", z_pos_th

    thumb_pos = [x_pos_th, y_pos_th, z_pos_th]
    mf_pos = [x_pos_mf, y_pos_mf, z_pos_mf]

    return mf_pos, thumb_pos
    

if __name__ == '__main__':
    
    #rospy.init_node('collision_detection')
   
    # TODO: put a description
        
    import sys
    #q1 = [float(sys.argv[1]), float(sys.argv[2]),float(sys.argv[3]), float(sys.argv[4])]
    #q2 = [float(sys.argv[5]), float(sys.argv[6]),float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9])]
    
    q1 = [radians(0), radians(32), radians(65), radians(65)]
    q2 = [radians(35), radians(59), radians(8), radians(26), radians(19)]
     
    r1 = 0.030 # radius [m]
    r2 = 0.030 # radius [m]
        
    p1,p2 =collision_detection(q1, q2) 

    print p1, p2
    
    distance = sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2 )
    
    if distance < (r1 + r2):
        print distance," contact"
    else:
        print distance

    #rospy.spin()    
    
