#! /usr/bin/env python
# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import roslib; roslib.load_manifest('sr_counting_demo')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the CounterDemoAction, including the
# goal message and the result message.
import sr_counting_demo.msg

def counter_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (CounterDemoAction) to the constructor.
    client = actionlib.SimpleActionClient('counter_server_py', sr_counting_demo.msg.CounterDemoAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    # Ask to the user to select an integer number in the range [1-5] 
    print "Please enter a number that you would like the hand to count [1-5]"
    
    while(True):    

        # Enter a number from the keyboard
        num_selected = input()
      
        # Check if the number is in the range [1-5]
        if num_selected in range(1,6): 
            print "You have selected number", num_selected  	  	  
            break        
        else:
            print "Please enter a number between 1 and 5"
      
    # Creates a goal to send to the action server
    goal = sr_counting_demo.msg.CounterDemoGoal(target = num_selected)

    # Sends the goal to the action server
    client.send_goal(goal)

    # Waits for the server to finish performing the action
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterDemoResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('counter_client_py')
        
        # The counter demo continue to run if it is not stopped by the user 
        while(True):
            
            print "Please type s to start the demo or q to quit."
            key_type = raw_input()
	    
            if (key_type == "s"): 
                result = counter_client() 
                print "The Hand has counted up to", str(result.sequence)
        
            elif (key_type == "q"):
                print "Remember to type Ctrl-C if you desire to stop the server node too..."
                break
            
            else: print "Wrong key selected."
            
       
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


