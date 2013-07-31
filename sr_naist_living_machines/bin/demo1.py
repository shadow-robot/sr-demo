#!/usr/bin/env python
import roslib; roslib.load_manifest('sr_naist_living_machines')
import rospy
from sr_robot_msgs.msg import sendupdate, joint
from std_msgs.msg import Float64
import math


# Shadow hand lib doesn't know about the muscle hand on ethercat yet, so quick
# and dirty sendupdate is needed
# Note naist hand is only 2 fingers and a thumb
joint_names = [ "thj1", "thj2", "thj3", "thj4", "thj5", "ffj0", "ffj3", "ffj4", "rfj0", "rfj3", "rfj4", "wrj1", "wrj2" ]
joint_pubs = {}
for jname in joint_names:
    topic = "/sh_"+jname+"_muscle_position_controller/command"
    print topic
    joint_pubs[jname] = rospy.Publisher(topic, Float64, latch=True)

rospy.init_node('demo1')

def sendupdate(joints):
    for jname in joints:
        if not jname in joint_pubs:
            print "Joint %s not found"%jname
            return
        msg = Float64()
        msg.data = math.radians( float(joints[jname]) )
        print jname
        joint_pubs[jname].publish(msg)

start_pose = { 'ffj0': 27.0, 'ffj3': 0, 'ffj4': 0,
               'rfj0': 27.0, 'rfj3': 0, 'rfj4': 0,
               'wrj1': 0, 'wrj2': 0 }

print "Start"
sendupdate(start_pose)
rospy.sleep(10)

sendupdate({ 'wrj2': 10 })
rospy.sleep(4)

sendupdate({ 'wrj2': -20 })
rospy.sleep(6)

sendupdate({ 'wrj2': 0 })
rospy.sleep(6)

print "Sleeping"
for i in range(1,10):
    rospy.sleep(1)
