#!/usr/bin/env python
"""
See README.md
"""

import rospy

# from moveit_commander import RobotCommander
# from moveit_commander import PlanningSceneInterface
# from moveit_commander import MoveGroupCommander
# from moveit_msgs.srv import ListRobotStatesInWarehouse as ListStates
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
# from moveit_msgs.srv import CheckIfRobotStateExistsInWarehouse as HasState

if __name__ == "__main__":

    rospy.init_node("graspatron_demo1")

    intial_state_name = rospy.get_param("initial_state_name", "initial_state")
    get_state_service = rospy.ServiceProxy("moveit_warehouse_services/get_robot_state", GetState)

