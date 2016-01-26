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


spray_joints = ['rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',
                 'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5',
                 'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4',
                 'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4',
                 'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5',
                 'rh_WRJ1', 'rh_WRJ2']

spray_position_1 = [0.016279069852545514, 0.8796459430051421, -0.1070232968388109, -0.01392039843107048, 0.4794807306173311, 1.2737286276860171, 0.7636915764719582, 0.02413406894427405, 0.02406051774917024, 0.44897183146448894, 1.1102674037118474, 1.0692782480178558, 0.031157544182387153, 0.6158235599366355, 1.1339610370666329, 0.9676271157835811, -0.02194279346296921, 0.14020124636714484, -0.3310194810821593, 0.20755596472482504, 1.1448595729499487, -0.5045108495604419, -0.01729248420134635, -0.0466767631287554]
spray_position_2 = [0.016766029764377577, 0.8748127235380809, -0.011187150477680296, -0.008301534915409695, 0.5754738394770192, 1.1557485168694157, 0.8056534297486129, 0.027141968467538413, -0.03747079919525946, 0.4622836647424118, 1.1423973285781066, 1.0772584530854215, 0.041592992917566345, 0.6955905609066889, 1.0241863658136754, 1.1073777879368387, -0.026597718961556365, 0.501383442900569, 0.24833995140134102, 0.16658954308833213, 1.1999222711800483, 0.28165546644901374, -0.029519223823197372, -0.05155965127766162]

grasp_spray = dict(zip(spray_joints, spray_position_1))

# Adjust poses according to the hand loaded
open_hand_current = dict([(i, open_hand[i]) for i in joints if i in open_hand])
grasp_pose_current = dict([(i, grasp_spray[i]) for i in joints if i in grasp_spray])

start_time = rospy.Time.now()

# Move hand using move_to_joint_value_target_unsafe to 1st position
hand_commander.move_to_joint_value_target_unsafe(open_hand_current, 1.0, True)

rospy.sleep(2)

# Move hand using run_joint_trajectory_unsafe to joint angles specified in 'position' list
hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

rospy.sleep(2)

grasp_spray = dict(zip(spray_joints, spray_position_2))
grasp_pose_current = dict([(i, grasp_spray[i]) for i in joints if i in grasp_spray])

hand_commander.move_to_joint_value_target_unsafe(grasp_pose_current, 1.0, False)

trajectory_start_time = 2.0
joint_trajectory = JointTrajectory()
