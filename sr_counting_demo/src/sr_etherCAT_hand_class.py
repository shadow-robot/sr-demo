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

import math
import rospy

from sr_robot_msgs.msg import joint
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy

class sr_etherCAT_hand(object):
    """
    This class defines useful functions to be used with the etherCAT Shadow hand.
    """
    def __init__(self):
        
        # The hand publishers:
        # we use a dictionary of publishers, because on the etherCAT hand
        # you have one publisher per controller.
        self.hand_publishers = self.create_hand_publishers()
       
    def create_hand_publishers(self):
        """
        Creates a dictionnary of publishers to send the targets to the controllers
        on /sh_??j?_mixed_position_velocity_controller/command
        """
        hand_pub = {}

        for joint in ["FFJ0", "FFJ3", "FFJ4",
                      "MFJ0", "MFJ3", "MFJ4",
                      "RFJ0", "RFJ3", "RFJ4",
                      "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                      "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                      "WRJ1", "WRJ2" ]:
            hand_pub[joint] = rospy.Publisher('/sh_'+joint.lower()+'_mixed_position_velocity_controller/command', Float64, 
                                              latch = True)

        return hand_pub
   
    def hand_publish(self, pose):
        """
        Publishes the given pose to the correct controllers for the hand.
        The targets are converted in radians.
        """
        for joint in pose:
    
            self.hand_publishers[joint.joint_name].publish(math.radians(joint.joint_target))

    def fetch_target(self, name):
        """
        Fetch the targets for the etherCAT hand from the server parameter
        """
        target_name = []
    
        for x in ["FFJ0", "FFJ3", "FFJ4",
                  "MFJ0", "MFJ3", "MFJ4",
                  "RFJ0", "RFJ3", "RFJ4",
                  "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                  "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                  "WRJ1", "WRJ2" ]:
            target_name.append( joint(joint_name = x, 
                                      joint_target = rospy.get_param('/targets/'+name+'/'+x)) )
        return target_name

    def order_joint_states(self, data):
        """
        Fetch the joint state from a service call.
        Joints positions are then stored in the same way of the "joint" msg structure.
        This will help for comparing current and target position. 
        @ param data: joint state in the form of a "JointState" msg
        """
        ordered_joint = []
        
        # Create a dictionary that contains joint name and position 
        # starting from a JointState msg
        joints_dictionary = dict(zip(data.joint_state.name, data.joint_state.position))
        
        # helper variable        
        suffix = 'FJ0'        

        # For each joint name, look for the corresponding value in the joints_dictionary
        for key in ["FFJ0", "FFJ3", "FFJ4",
                    "MFJ0", "MFJ3", "MFJ4",
                    "RFJ0", "RFJ3", "RFJ4",
                    "LFJ0", "LFJ3", "LFJ4", "LFJ5",
                    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                    "WRJ1", "WRJ2" ]:
                
                # Check if the key contains "FJ0": "FFJ0", "MFJ0", "RFJ0", "LFJ0"
                # This has to be done because for convention ?FJO = ?FJ1 + ?FJ2
                if key[1:] == suffix:
                   ordered_joint.append( joint(joint_name = key,
                                         joint_target = joints_dictionary[key[:1]+"FJ1"] + joints_dictionary[key[:1]+"FJ2"]))       
                else: 
                    ordered_joint.append( joint(joint_name = key, 
                                                joint_target = joints_dictionary[key]) )       
        return ordered_joint

    def compute_joint_error_position(self, current_position, target_position):
        """
        Compute the norm of the error between the current joint position and the target joint position. 
        @ param current_position: joint current position
        @ param target_position: joint desired position   
        """
        
        # helper variables
        tmp_c = []
        tmp_t = []      
        
        for x in range(0,20):
            tmp_c.append(current_position[x].joint_target)
            tmp_t.append(math.radians (target_position[x].joint_target) )
        
        # Compute the norm of the error
        error = numpy.linalg.norm( numpy.array(tmp_c) - numpy.array(tmp_t) )
       
        #print error  

        return error
