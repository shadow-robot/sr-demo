#!/usr/bin/env python
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

from sr_robot_msgs.msg import joint

    
# start position for counting: punch   
punch_step1 = [ joint(joint_name = "THJ1", joint_target = 68),
                joint(joint_name = "THJ2", joint_target = 30),
                joint(joint_name = "THJ3", joint_target = 15),
                joint(joint_name = "THJ4", joint_target = 70),
                joint(joint_name = "THJ5", joint_target = 30),
                joint(joint_name = "FFJ0", joint_target = 0),
                joint(joint_name = "FFJ3", joint_target = 0),
                joint(joint_name = "FFJ4", joint_target = 0),
                joint(joint_name = "MFJ0", joint_target = 0),
                joint(joint_name = "MFJ3", joint_target = 0),
                joint(joint_name = "MFJ4", joint_target = 0),
                joint(joint_name = "RFJ0", joint_target = 0),
                joint(joint_name = "RFJ3", joint_target = 0),
                joint(joint_name = "RFJ4", joint_target = 0),
                joint(joint_name = "LFJ0", joint_target = 0),
                joint(joint_name = "LFJ3", joint_target = 0),
                joint(joint_name = "LFJ4", joint_target = 0),
                joint(joint_name = "LFJ5", joint_target = 0),
                joint(joint_name = "WRJ1", joint_target = 0),
                joint(joint_name = "WRJ2", joint_target = 0) ]

punch_step2 = [ joint(joint_name = "THJ1", joint_target = 68),
                joint(joint_name = "THJ2", joint_target = 30),
                joint(joint_name = "THJ3", joint_target = 15),
                joint(joint_name = "THJ4", joint_target = 70),
                joint(joint_name = "THJ5", joint_target = 30),
                joint(joint_name = "FFJ0", joint_target = 118),
                joint(joint_name = "FFJ3", joint_target = 45),
                joint(joint_name = "FFJ4", joint_target = 0),
                joint(joint_name = "MFJ0", joint_target = 118),
                joint(joint_name = "MFJ3", joint_target = 45),
                joint(joint_name = "MFJ4", joint_target = 0),
                joint(joint_name = "RFJ0", joint_target = 118),
                joint(joint_name = "RFJ3", joint_target = 45),
                joint(joint_name = "RFJ4", joint_target = 0),
                joint(joint_name = "LFJ0", joint_target = 118),
                joint(joint_name = "LFJ3", joint_target = 45),
                joint(joint_name = "LFJ4", joint_target = 0),
                joint(joint_name = "LFJ5", joint_target = 0),
                joint(joint_name = "WRJ1", joint_target = 0),
                joint(joint_name = "WRJ2", joint_target = 0) ]

# hand extended position
hand_extended_pos = [ joint(joint_name = "THJ1", joint_target = 21),
                      joint(joint_name = "THJ2", joint_target = 25),
                      joint(joint_name = "THJ3", joint_target = 0),
                      joint(joint_name = "THJ4", joint_target = 15),
                      joint(joint_name = "THJ5", joint_target = 9),
                      joint(joint_name = "FFJ0", joint_target = 0),
                      joint(joint_name = "FFJ3", joint_target = 0),
                      joint(joint_name = "FFJ4", joint_target = 0),
                      joint(joint_name = "MFJ0", joint_target = 0),
                      joint(joint_name = "MFJ3", joint_target = 0),
                      joint(joint_name = "MFJ4", joint_target = 0),
                      joint(joint_name = "RFJ0", joint_target = 0),
                      joint(joint_name = "RFJ3", joint_target = 0),
                      joint(joint_name = "RFJ4", joint_target = 0),
                      joint(joint_name = "LFJ0", joint_target = 0),
                      joint(joint_name = "LFJ3", joint_target = 0),
                      joint(joint_name = "LFJ4", joint_target = 0),
                      joint(joint_name = "LFJ5", joint_target = 0),
                      joint(joint_name = "WRJ1", joint_target = 0),
                      joint(joint_name = "WRJ2", joint_target = 0) ]
    
# one: FF up
one = [ joint(joint_name = "THJ1", joint_target = 68),
        joint(joint_name = "THJ2", joint_target = 30),
        joint(joint_name = "THJ3", joint_target = 15),
        joint(joint_name = "THJ4", joint_target = 70),
        joint(joint_name = "THJ5", joint_target = 30),
        joint(joint_name = "FFJ0", joint_target = 0),
        joint(joint_name = "FFJ3", joint_target = 0),
        joint(joint_name = "FFJ4", joint_target = 0),
	joint(joint_name = "MFJ0", joint_target = 118),
        joint(joint_name = "MFJ3", joint_target = 45),
        joint(joint_name = "MFJ4", joint_target = 0),
        joint(joint_name = "RFJ0", joint_target = 118),
        joint(joint_name = "RFJ3", joint_target = 45),
        joint(joint_name = "RFJ4", joint_target = 0),
        joint(joint_name = "LFJ0", joint_target = 118),
        joint(joint_name = "LFJ3", joint_target = 45),
        joint(joint_name = "LFJ4", joint_target = 0),
        joint(joint_name = "LFJ5", joint_target = 0),
        joint(joint_name = "WRJ1", joint_target = 0),
        joint(joint_name = "WRJ2", joint_target = 0) ]                                      


# two: FF + MF up
two = [ joint(joint_name = "THJ1", joint_target = 68),
        joint(joint_name = "THJ2", joint_target = 30),
        joint(joint_name = "THJ3", joint_target = 15),
        joint(joint_name = "THJ4", joint_target = 70),
        joint(joint_name = "THJ5", joint_target = 30),
        joint(joint_name = "FFJ0", joint_target = 0),
        joint(joint_name = "FFJ3", joint_target = 0),
        joint(joint_name = "FFJ4", joint_target = 0),
        joint(joint_name = "MFJ0", joint_target = 0),
        joint(joint_name = "MFJ3", joint_target = 0),
        joint(joint_name = "MFJ4", joint_target = 0),
        joint(joint_name = "RFJ0", joint_target = 118),
        joint(joint_name = "RFJ3", joint_target = 45),
        joint(joint_name = "RFJ4", joint_target = 0),
        joint(joint_name = "LFJ0", joint_target = 118),
        joint(joint_name = "LFJ3", joint_target = 45),
        joint(joint_name = "LFJ4", joint_target = 0),
        joint(joint_name = "LFJ5", joint_target = 0),
        joint(joint_name = "WRJ1", joint_target = 0),
        joint(joint_name = "WRJ2", joint_target = 0) ]

    	
# three: FF + MF + RF up
three = [ joint(joint_name = "THJ1", joint_target = 68),
          joint(joint_name = "THJ2", joint_target = 30),
          joint(joint_name = "THJ3", joint_target = 15),
          joint(joint_name = "THJ4", joint_target = 70),
          joint(joint_name = "THJ5", joint_target = 30),
          joint(joint_name = "FFJ0", joint_target = 0),
          joint(joint_name = "FFJ3", joint_target = 0),
          joint(joint_name = "FFJ4", joint_target = 0),
          joint(joint_name = "MFJ0", joint_target = 0),
          joint(joint_name = "MFJ3", joint_target = 0),
          joint(joint_name = "MFJ4", joint_target = 0),
          joint(joint_name = "RFJ0", joint_target = 0),
          joint(joint_name = "RFJ3", joint_target = 0),
          joint(joint_name = "RFJ4", joint_target = 0),
          joint(joint_name = "LFJ0", joint_target = 118),
          joint(joint_name = "LFJ3", joint_target = 45),
          joint(joint_name = "LFJ4", joint_target = 0),
          joint(joint_name = "LFJ5", joint_target = 0),
          joint(joint_name = "WRJ1", joint_target = 0),
          joint(joint_name = "WRJ2", joint_target = 0) ]
	

# four: FF + MF + RF + LF up
four = [ joint(joint_name = "THJ1", joint_target = 68),
         joint(joint_name = "THJ2", joint_target = 30),
         joint(joint_name = "THJ3", joint_target = 15),
         joint(joint_name = "THJ4", joint_target = 70),
         joint(joint_name = "THJ5", joint_target = 30),
         joint(joint_name = "FFJ0", joint_target = 0),
         joint(joint_name = "FFJ3", joint_target = 0),
         joint(joint_name = "FFJ4", joint_target = 0),
         joint(joint_name = "MFJ0", joint_target = 0),
         joint(joint_name = "MFJ3", joint_target = 0),
         joint(joint_name = "MFJ4", joint_target = 0),
         joint(joint_name = "RFJ0", joint_target = 0),
         joint(joint_name = "RFJ3", joint_target = 0),
         joint(joint_name = "RFJ4", joint_target = 0),
         joint(joint_name = "LFJ0", joint_target = 0),
         joint(joint_name = "LFJ3", joint_target = 0),
         joint(joint_name = "LFJ4", joint_target = 0),
         joint(joint_name = "LFJ5", joint_target = 0),
         joint(joint_name = "WRJ1", joint_target = 0),
         joint(joint_name = "WRJ2", joint_target = 0) ]


# five: FF + MF + RF + LF + TH up
five = hand_extended_pos

    
#A vector containing the different target, in the same order
# they are called.
numbers = [one, two, three, four, five] 


