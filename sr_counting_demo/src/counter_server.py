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

import roslib; roslib.load_manifest('sr_counting_demo')
import rospy
import time, mutex, math

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the CounterDemoAction, including the
# goal message and the result message.
import sr_counting_demo.msg

# This class contains some useful functions for controlling the shadow hand
import sr_etherCAT_hand_class 

class CounterDemoAction(object):
    # create messages that are used to publish feedback/result
    _feedback = sr_counting_demo.msg.CounterDemoFeedback()
    _result   = sr_counting_demo.msg.CounterDemoResult()

    def __init__(self, name):
        """
        The action client and server communicate over a set of topics, described in the actionlib protocol. 
        The action name (name) describes the namespace containing these topics, and the action specification message 
        (CounterDemoAction) describes what messages should be passed along these topics.
        """
    
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, sr_counting_demo.msg.CounterDemoAction, 
                                                execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
     
    def execute_cb(self, goal):  
        """
        This is the execute callback function that we'll run everytime a new goal is received. 
        The goal represents the number that will be counted by the hand.
        """
    
        # helper variables
        success = True
    
        #Initialization of the counterDemo class 
        count = sr_etherCAT_hand_class.sr_etherCAT_hand()

        #Target parameters
        one = count.fetch_target('one')
        two = count.fetch_target('two')
        three = count.fetch_target('three')
        four = count.fetch_target('four')
        five = count.fetch_target('five')
        fist_step1 = count.fetch_target('fist_step1')
        fist_step2 = count.fetch_target('fist_step2')
        hand_extended_pos = count.fetch_target('hand_extended_pos')
        numbers = [one, two, three, four, five]

        #Opens the the hand
        count.hand_publish(hand_extended_pos)        

        #Waiting for the hand has reached its rest position
        time.sleep(5)
    
        #Publish info to the console for the user
        rospy.loginfo('%s: Executing, the hand is going to count from 1 to %i' %(self._action_name, goal.target))
	 
        #The hand is going to assume the form of a fist before starting to count
    
        #Close the hand in two steps, first the thumb...
        count.hand_publish(fist_step1)        
        time.sleep(2)
	
        #...and then the other fingers
        count.hand_publish(fist_step2)
        time.sleep(5)

        # start executing the action: count!
        for i in xrange(0, goal.target):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
       
            count.hand_publish(numbers[i])
            time.sleep(3)					

            self._feedback.sequence = i+1

            # publish the feedback
            self._as.publish_feedback(self._feedback)
      
        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
    rospy.init_node('counter_hand_demo')
    CounterDemoAction(rospy.get_name())
    rospy.spin()


