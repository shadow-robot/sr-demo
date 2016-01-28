#!/usr/bin/env python
"""
See README.md
"""

import rospy
import tf2_ros
import roslib
from threading import Thread
from rospy.exceptions import ROSException
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetLinkState
from sr_robot_commander.sr_arm_commander import SrArmCommander
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from sr_graspatron.utils import *


broadcast_thread = None


class BroadcastThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.__stop = False
        rospy.wait_for_service("/gazebo/get_link_state")
        self.__get_link_state_service = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        alvar_marker_prefix = "ar_marker_"
        hand_alvar_marker_name = alvar_marker_prefix + str(rospy.get_param("/settings/hand_alvar_marker_number", 0))
        object_alvar_marker_number = rospy.get_param("/settings/object_alvar_marker_number", 1)
        object_alvar_marker_name = alvar_marker_prefix + str(object_alvar_marker_number)

        self.__tf2_broadcaster = tf2_ros.TransformBroadcaster()
        self.__hand_to_object_transformation = TransformStamped()
        self.__hand_to_object_transformation.header.frame_id = object_alvar_marker_name
        self.__hand_to_object_transformation.child_frame_id = hand_alvar_marker_name

        self.__object_to_world_transformation = TransformStamped()
        self.__object_to_world_transformation.header.frame_id = "world"
        self.__object_to_world_transformation.child_frame_id = object_alvar_marker_name

    def shutdown(self):
        self.__stop = True

    def run(self):
        rate = rospy.Rate(10)
        try:
            while not self.__stop and not rospy.is_shutdown():
                object_relative_position = self.__get_link_state_service("my_object::link", "world")
                wrist_relative_position = self.__get_link_state_service("ur10srh::rh_wrist", "world")

                object_pose = object_relative_position .link_state.pose
                self.__object_to_world_transformation.header.stamp = rospy.Time().now()
                # Publishing tf to world frame to avoid warning in the logs
                self.__object_to_world_transformation.transform.translation.x = object_pose.position.x
                self.__object_to_world_transformation.transform.translation.y = object_pose.position.y
                self.__object_to_world_transformation.transform.translation.z = object_pose.position.z
                self.__object_to_world_transformation.transform.rotation.x = object_pose.orientation.x
                self.__object_to_world_transformation.transform.rotation.y = object_pose.orientation.y
                self.__object_to_world_transformation.transform.rotation.z = object_pose.orientation.z
                self.__object_to_world_transformation.transform.rotation.w = object_pose.orientation.w
                self.__tf2_broadcaster.sendTransform(self.__object_to_world_transformation)

                wrist_pose = wrist_relative_position.link_state.pose
                self.__hand_to_object_transformation.header.stamp = rospy.Time.now()
                self.__hand_to_object_transformation.transform.translation.x = (object_pose.position.x -
                                                                                wrist_pose.position.x)
                self.__hand_to_object_transformation.transform.translation.y = (object_pose.position.y -
                                                                                wrist_pose.position.y)
                self.__hand_to_object_transformation.transform.translation.z = (object_pose.position.z -
                                                                                wrist_pose.position.z)
                self.__hand_to_object_transformation.transform.rotation.x = wrist_pose.orientation.x
                self.__hand_to_object_transformation.transform.rotation.y = wrist_pose.orientation.y
                self.__hand_to_object_transformation.transform.rotation.z = wrist_pose.orientation.z
                self.__hand_to_object_transformation.transform.rotation.w = wrist_pose.orientation.w
                self.__tf2_broadcaster.sendTransform(self.__hand_to_object_transformation)

                rate.sleep()
        except ROSException:
            pass


def add_or_update_model(existing_models, model_name, pose, model_type=None, filename=None, hard_reset=False):
    add_new_model = False
    if model_name in existing_models:
        rospy.loginfo("Updating {} position...".format(model_name))
        if hard_reset:
            delete_gazebo_model(model_name)
            rospy.sleep(2)
            add_new_model = True
        else:
            update_gazebo_model(model_name, pose)
    else:
        rospy.loginfo("Adding new {}...".format(model_name))
        add_new_model = True

    if add_new_model:
        if filename is not None:
            if model_type is not None:
                message = "Only one parameter should be provided model_type or filename"
                rospy.logerr(message)
                raise Exception(message)
            add_gazebo_model_from_sdf(model_name, filename, pose)
        elif model_type is not None:
            add_gazebo_model_from_database(model_name, model_type, pose)
        else:
            message = "Either model_type or filename should be provided"
            rospy.logerr(message)
            raise Exception(message)


def setup_gazebo_world():
    existing_models = get_gazebo_world_models_name()

    table_pose = get_pose(1.0, 0.7, 0.01)
    add_or_update_model(existing_models, "main_table", table_pose,
                        filename=roslib.packages.get_pkg_dir("sr_graspatron") + "/models/simple_box.sdf")

    rospy.sleep(1)

    object_position = get_pose(0.8, 0.7, 0.515)
    add_or_update_model(existing_models, "my_object", object_position,
                        filename=roslib.packages.get_pkg_dir("sr_graspatron") + "/models/glass_cleaner.sdf",
                        hard_reset=True)
    rospy.sleep(2)


def initialize_gazebo_simulation():

    global broadcast_thread

    rospy.loginfo("Waiting for warehouse services...")
    rospy.wait_for_service("save_robot_state")
    save_robot_state_service = rospy.ServiceProxy("save_robot_state", SaveState)

    rospy.loginfo("Moving to initial pose...")
    arm_commander = SrArmCommander()

    group_id = rospy.get_param("/settings/arm_group_name")
    group = MoveGroupCommander(group_id)

    group.clear_pose_targets()
    initial_pose = group.get_current_pose().pose
    initial_pose.position.x = 0.36643666952
    initial_pose.position.y = 0.0710018954239
    initial_pose.position.z = 0.897101685535
    initial_pose.orientation.x = -0.644286785844
    initial_pose.orientation.y = 0.123669678951
    initial_pose.orientation.z = -0.0423218963893
    initial_pose.orientation.w = 0.753531157406

    group.set_pose_target(initial_pose)
    plan = group.plan()
    group.execute(plan)
    rospy.sleep(4)

    robot_state = RobotState()
    current_dict = arm_commander.get_robot_state_bounded()
    robot_name = arm_commander.get_robot_name()
    robot_state.joint_state = JointState()
    robot_state.joint_state.name = current_dict.keys()
    robot_state.joint_state.position = current_dict.values()
    save_robot_state_service(rospy.get_param("/settings/initial_state_name", "gazebo_initial_state"), robot_name,
                             robot_state)
    rospy.loginfo("Saved initial state to database")

    rospy.loginfo("Setting up Gazebo world...")
    setup_gazebo_world()

    rospy.loginfo("Starting broadcasting hand to object transformation...")
    broadcast_thread = BroadcastThread()
    broadcast_thread.start()


def terminate_gazebo_simulation():
    if broadcast_thread is not None:
        broadcast_thread.shutdown()
