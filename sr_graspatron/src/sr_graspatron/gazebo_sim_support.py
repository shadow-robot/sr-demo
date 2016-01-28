#!/usr/bin/env python
"""
See README.md
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetLinkState
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState


if __name__ == "__main__":

    rospy.init_node("gazebo_sim_support")

    hand_commander = SrHandCommander()
    arm_commander = SrArmCommander()

    group_id = rospy.get_param("/settings/arm_group_name")
    group = MoveGroupCommander(group_id)

    initial_pose = group.get_current_pose().pose
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 1.0
    group.clear_pose_targets()
    group.set_pose_target(initial_pose)
    plan = group.plan()
    group.execute(plan)
    rospy.sleep(4)

    rospy.loginfo("Waiting for warehouse services...")
    rospy.wait_for_service("moveit_warehouse_services/save_robot_state")  # TODO Check service name !!!
    save_robot_state_service = rospy.ServiceProxy("save_robot_state", SaveState)

    robot_state = RobotState()
    current_dict = {}
    current_dict = arm_commander.get_robot_state_bounded()
    robot_name = arm_commander.get_robot_name()
    robot_state.joint_state = JointState()
    robot_state.joint_state.name = current_dict.keys()
    robot_state.joint_state.position = current_dict.values()
    save_robot_state_service(rospy.get_param("/settings/initial_state_name", "gazebo_initial_state"), robot_name,
                             robot_state)
    rospy.loginfo("Saved initial state to database")

    rospy.loginfo("Starting broadcasting hand to object transformation...")
    rospy.wait_for_service("/gazebo/get_link_state")
    get_link_state_service = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    alvar_marker_prefix = "ar_marker_"
    hand_alvar_marker_name = alvar_marker_prefix + str(rospy.get_param("/settings/hand_alvar_marker_number", 0))
    object_alvar_marker_number = rospy.get_param("/settings/object_alvar_marker_number", 1)
    object_alvar_marker_name = alvar_marker_prefix + str(object_alvar_marker_number)

    rate = rospy.Rate(10)
    tf2_broadcaster = tf2_ros.TransformBroadcaster()
    hand_to_object_transformation = TransformStamped()
    hand_to_object_transformation.header.frame_id = object_alvar_marker_name
    hand_to_object_transformation.child_frame_id = hand_alvar_marker_name

    while not rospy.is_shutdown():
        hand_to_object_transformation.header.stamp = rospy.Time.now()
        # Publishing reverse transformation due to Gazebo unpredictable results
        my_object_relative_position = get_link_state_service("ur10srh::rh_wrist", "my_object::link")
        pose = my_object_relative_position.link_state.pose
        hand_to_object_transformation.transform.translation.x = pose.position.x
        hand_to_object_transformation.transform.translation.y = pose.position.y
        hand_to_object_transformation.transform.translation.z = pose.position.z
        hand_to_object_transformation.transform.rotation.x = pose.orientation.x
        hand_to_object_transformation.transform.rotation.y = pose.orientation.y
        hand_to_object_transformation.transform.rotation.z = pose.orientation.z
        hand_to_object_transformation.transform.rotation.w = pose.orientation.w
        tf2_broadcaster.sendTransform(hand_to_object_transformation)
        rate.sleep()
