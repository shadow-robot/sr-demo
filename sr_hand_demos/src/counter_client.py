#! /usr/bin/env python

import roslib; roslib.load_manifest('sr_hand_demos')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the counter_demoAction, including the
# goal message and the result message.
import sr_hand_demos.msg

def counter_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (counter_demoAction) to the constructor.
    client = actionlib.SimpleActionClient('counter_hand_demo', sr_hand_demos.msg.counter_demoAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    # Ask to the user to select an integer number in the range [1-5] 
    print "Please enter a number that you would like the hand to count [1-5]"
    
    loop = True
    
    while(loop):    

        # Enter a number from the keyboard
        num_selected = input()
      
        # Check if the number is in the range [1-5]
        if num_selected in range(1,6): 
            print "You have selected number", num_selected  	  	  
            loop = False        
        else:
            print "Please enter a number between 1 and 5"
      
        
        
    # Creates a goal to send to the action server
    goal = sr_hand_demos.msg.counter_demoGoal(target = num_selected)

    # Sends the goal to the action server
    client.send_goal(goal)

    # Waits for the server to finish performing the action
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A counter_demoResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('counter_client_py')
        result = counter_client()
        print "The Hand has counted up to", str(result.sequence)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


