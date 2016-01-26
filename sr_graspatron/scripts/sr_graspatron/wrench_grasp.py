#!/usr/bin/env python

# This example demonstrates how you can send a partial trajectory to a joint or set of joints.
# The partial trajectory can then be run during an existing motion and define a new goal for
# the joints specified in this sub list

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("partial_traj_example", anonymous=True)
rospy.sleep(1)  # Do not start at time zero


def construct_trajectory_point(posture, duration):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(posture[key])
    return trajectory_point

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)

hand_mapping = hand_parameters.mapping[hand_serial]

# Hand joints are detected
joints = hand_finder.get_hand_joints()[hand_mapping]

open_hand = {'rh_FFJ1': 0.0, 'rh_FFJ2': 0.0, 'rh_FFJ3': 0.0, 'rh_FFJ4': 0.0,
             'rh_MFJ1': 0.0, 'rh_MFJ2': 0.0, 'rh_MFJ3': 0.0, 'rh_MFJ4': 0.0,
             'rh_RFJ1': 0.0, 'rh_RFJ2': 0.0, 'rh_RFJ3': 0.0, 'rh_RFJ4': 0.0,
             'rh_LFJ1': 0.0, 'rh_LFJ2': 0.0, 'rh_LFJ3': 0.0, 'rh_LFJ4': 0.0,
             'rh_LFJ5': 0.0, 'rh_THJ1': 0.0, 'rh_THJ2': 0.0, 'rh_THJ3': 0.0,
             'rh_THJ4': 0.0, 'rh_THJ5': 0.0}

keys = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4', 'rh_LFJ1', 'rh_LFJ2',
        'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5', 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3',
        'rh_MFJ4', 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4', 'rh_THJ1',
        'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5']


wrench_joints = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',
                 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5',
                 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4',
                 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4',
                 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5',
                 'rh_WRJ1', 'rh_WRJ2']

wrench_position_1 = [0.904356, 1.648319, 0.009656, -0.0065742,
                     0.7165546, 1.584021, -0.00508159, -0.052422, 0.102148,
                     0.840571, 1.5950723, -0.00178243, 0.00855392,
                     0.8478579, 1.6728142, 0.01159525, -0.0743803,
                     0.5327641, -0.02261, 0.13089, 1.17590, -0.685,
                     -0.020449, -0.03219032]

FINGER_BEND = 30
wrench_position_2 = [0.904356, 1.648319, FINGER_BEND*math.pi/180, 0,
                     0.7165546, 1.584021, FINGER_BEND*math.pi/180, 0, 0.102148,
                     0.840571, 1.5950723, FINGER_BEND*math.pi/180, 0,
                     0.8478579, 1.6728142, FINGER_BEND*math.pi/180, 0,
                     0.8290, -0.25771, -0.017149, 0.10925, -0.95748,
                     -0.020449, -0.03219032]

wrench_position_3 = [0.9264, 1.63434, 0.51665, -0.014886,
                     0.7723082, 1.50975, 0.37688, -0.016572, 0.06896,
                     0.854202, 1.57793, 0.512193, 0.00934,
                     0.942212, 1.58058, 0.5221, -0.00636,
                     0.678386, 0.13838, -0.18313, 0.028126, -0.4839,
                     -0.0173, -0.048744]

wrench_position_4 = [1.002645, 1.48946, FINGER_BEND*math.pi/180, 0,
                     0.99533, 1.220826, FINGER_BEND*math.pi/180, 0, 0.038,
                     0.83668, 1.5594, FINGER_BEND*math.pi/180, 0.0138,
                     0.984, 1.442, FINGER_BEND*math.pi/180, -0.014,
                     0.4618, 0.10727, -0.011157, 0.1119258, 0.17151,
                     -0.0192555, -0.0513]


grasp_wrench = dict(zip(wrench_joints, wrench_position_1))

# Adjust poses according to the hand loaded
start_pose = True
if start_pose:
    open_hand_current = dict([(i, open_hand[i]) for i in joints if i in open_hand])
    grasp_pose_current = dict([(i, grasp_wrench[i]) for i in joints if i in grasp_wrench])

    start_time = rospy.Time.now()

    # Move hand using move_to_joint_value_target_unsafe to 1st position
    hand_commander.move_to_joint_value_target_unsafe(open_hand_current, 1.0, True)

    rospy.sleep(2)

    # Move hand using run_joint_trajectory_unsafe to joint angles specified in 'position' list
    hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

    rospy.sleep(2)
grasp_wrench = dict(zip(wrench_joints, wrench_position_2))
grasp_pose_current = dict([(i, grasp_wrench[i]) for i in joints if i in grasp_wrench])
hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

rospy.sleep(2)
grasp_wrench = dict(zip(wrench_joints, wrench_position_3))
grasp_pose_current = dict([(i, grasp_wrench[i]) for i in joints if i in grasp_wrench])
hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

rospy.sleep(2)
grasp_wrench = dict(zip(wrench_joints, wrench_position_4))
grasp_pose_current = dict([(i, grasp_wrench[i]) for i in joints if i in grasp_wrench])
hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

trajectory_start_time = 2.0
joint_trajectory = JointTrajectory()
