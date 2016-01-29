#!/usr/bin/env python
"""
See README.md
"""

import rospy
import tf2_ros
import threading
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_graspatron.gazebo_sim_support import initialize_gazebo_simulation, terminate_gazebo_simulation
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


def get_hand_to_object_transformation():
    rate = rospy.Rate(10)
    found = False
    hand_to_object_transformation = None
    i = 0
    while i < 3 and not found:
        try:
            hand_to_object_transformation = transformations_buffer.lookup_transform(hand_alvar_marker_name,
                                                                                    object_alvar_marker_name,
                                                                                    rospy.Time())
            found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
        i += 1

    if not found:
        raise Exception("Could not find transformation between hand and object markers " + hand_alvar_marker_name +
                        " and " + object_alvar_marker_name)
    return hand_to_object_transformation


def speech_recognition_callback(message):
    global recognized_message
    rospy.loginfo("Recognized => %s", message.data)
    recognized_message = message.data
    message_arrived_trigger_event.set()

if __name__ == "__main__":

    rospy.init_node("graspatron_demo2")

    if rospy.get_param("/settings/simulation", False):
        # TODO Add cube as an obstacle to moveit group
        initialize_gazebo_simulation()

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 0
    p.pose.position.y = 0
    # offset such that the box is below the dome
    p.pose.position.z = -0.12
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    scene.add_box("ground", p, (3, 3, 0.1))
    rospy.sleep(1)

    message_arrived_trigger_event = threading.Event()
    rospy.Subscriber("recognizer/output", String, speech_recognition_callback)

    sound_handle = SoundClient()
    rospy.sleep(1)

    command_execution_timeout = 20
    found = False

    while not found and not rospy.is_shutdown():
        recognized_message = ""
        message_arrived_trigger_event.clear()
        sound_handle.say("Waiting for command", "voice_kal_diphone")
        message_arrived_trigger_event.wait(command_execution_timeout)

        if any([item in ["grasp", "screw", "wrench", "washer"] for item in recognized_message.split(" ")]):
            found = True

    hand_commander = SrHandCommander()
    arm_commander = SrArmCommander()

    rospy.loginfo("Moving to initial position...")
    initial_state_name = rospy.get_param("/settings/initial_state_name", "gazebo_initial_state")
    hand_commander.move_to_named_target(initial_state_name, False)
    arm_commander.move_to_named_target(initial_state_name, True)

    rospy.loginfo("Moving close to object...")
    arm_commander.move_to_named_target("gazebo_move_pre_grasp", True)

    rospy.loginfo("Setting pre-grasp pose...")
    hand_commander.move_to_named_target("open", True)

    rospy.loginfo("Moving closer to object...")
    arm_commander.move_to_named_target("gazebo_move_grasp", True)
    hand_commander.move_to_named_target("gazebo_hand_grasp", True)
    # hand_commander.move_to_named_target("fingers_pack_thumb_open", True)

    rospy.loginfo("Moving object to air...")
    arm_commander.move_to_named_target("gazebo_move_up", True)

    found = False

    while not found and not rospy.is_shutdown():
        recognized_message = ""
        message_arrived_trigger_event.clear()
        sound_handle.say("Waiting for command", "voice_kal_diphone")
        message_arrived_trigger_event.wait(command_execution_timeout)

        if any([item in ["put", "screw", "wrench", "washer"] for item in recognized_message.split(" ")]):
            found = True

    rospy.loginfo("Releasing object...")
    hand_commander.move_to_named_target("gazebo_hand_release", True)

    rospy.loginfo("Moving to initial position...")
    hand_commander.move_to_named_target(initial_state_name, False)
    arm_commander.move_to_named_target(initial_state_name, True)

    if rospy.get_param("/settings/simulation", False):
        terminate_gazebo_simulation()

