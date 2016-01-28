#!/usr/bin/env python
"""
See README.md
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_graspatron.gazebo_sim_support import initialize_gazebo_simulation, terminate_gazebo_simulation


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

if __name__ == "__main__":

    rospy.init_node("graspatron_demo1")

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

    alvar_marker_prefix = "ar_marker_"
    initial_state_name = rospy.get_param("/settings/initial_state_name", "initial_state")
    hand_alvar_marker_name = alvar_marker_prefix + str(rospy.get_param("/settings/hand_alvar_marker_number", 0))
    object_alvar_marker_number = rospy.get_param("/settings/object_alvar_marker_number", 1)
    object_alvar_marker_name = alvar_marker_prefix + str(object_alvar_marker_number)

    hand_commander = SrHandCommander()
    arm_commander = SrArmCommander()

    rospy.loginfo("Moving to initial position...")
    hand_commander.move_to_named_target(initial_state_name, False)
    arm_commander.move_to_named_target(initial_state_name, True)

    rospy.loginfo("Searching transformation between hand and object...")
    transformations_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(transformations_buffer)

    hand_to_object_transformation = get_hand_to_object_transformation()

    rospy.loginfo("Moving closer to object...")
    group_id = rospy.get_param("/settings/arm_group_name")
    group = MoveGroupCommander(group_id)
    robot_link = group.get_end_effector_link()
    robot_frame = group.get_pose_reference_frame()

    pre_grasp_pose = group.get_current_pose().pose
    pre_grasp_delta = 0.3
    pre_grasp_pose.position.x += hand_to_object_transformation.transform.translation.x - pre_grasp_delta
    pre_grasp_pose.position.y += hand_to_object_transformation.transform.translation.y - pre_grasp_delta
    pre_grasp_pose.position.z += hand_to_object_transformation.transform.translation.z
    group.clear_pose_targets()
    group.set_pose_target(pre_grasp_pose)
    plan = group.plan()
    group.execute(plan)
    rospy.sleep(4)

    rospy.loginfo("Setting pre-grasp pose...")
    pre_grasp_pose_name = rospy.get_param("/settings/pre_grasp_pose_name", "pre_grasp")
    hand_commander.move_to_named_target(pre_grasp_pose_name, True)

    rospy.loginfo("Moving close to object...")
    hand_to_object_transformation = get_hand_to_object_transformation()
    grasp_pose = group.get_current_pose().pose
    grasp_delta = 0.1
    grasp_pose.position.x += hand_to_object_transformation.transform.translation.x - grasp_delta
    grasp_pose.position.y += hand_to_object_transformation.transform.translation.y
    grasp_pose.position.z += hand_to_object_transformation.transform.translation.z
    group.clear_pose_targets()
    group.set_pose_target(grasp_pose)
    plan = group.plan()
    group.execute(plan)
    rospy.sleep(4)

    rospy.loginfo("Grasping object...")
    grasp_pose_name = rospy.get_param("/settings/grasp_pose_name", "grasp")
    hand_commander.move_to_named_target(grasp_pose_name, True)

    rospy.loginfo("Picking the object up...")
    raise_height = 0.2
    grasp_pose = group.get_current_pose().pose
    grasp_pose.position.z += raise_height
    group.clear_pose_targets()
    group.set_pose_target(grasp_pose)
    plan = group.plan()
    group.execute(plan)
    rospy.sleep(4)

    rospy.loginfo("Putting object down...")
    raise_height = 0.3
    grasp_pose = group.get_current_pose().pose
    grasp_pose.position.z -= raise_height
    group.clear_pose_targets()
    group.set_pose_target(grasp_pose)
    plan = group.plan()
    group.execute(plan)
    rospy.sleep(4)

    rospy.loginfo("Setting pre-grasp pose...")
    hand_commander.move_to_named_target(pre_grasp_pose_name, True)

    rospy.loginfo("Moving to initial position...")
    hand_commander.move_to_named_target(initial_state_name, False)
    arm_commander.move_to_named_target(initial_state_name, True)

    if rospy.get_param("/settings/simulation", False):
        terminate_gazebo_simulation()

