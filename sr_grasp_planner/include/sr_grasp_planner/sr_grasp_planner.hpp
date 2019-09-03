/*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.

* @file   sr_grasp_planner.hpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
* @date   Tue Mar 22 10:39:10 2011
*
* @brief  Plans grasps for an unknow object.
*
*
*/

#ifndef _SR_GRASP_PLANNER_H_
#define _SR_GRASP_PLANNER_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <boost/smart_ptr.hpp>

#include <Eigen/Core>

namespace shadowrobot
{
  class SrGraspPlanner
  {
  public:
    SrGraspPlanner();
    ~SrGraspPlanner();

    std::vector<object_manipulation_msgs::Grasp> compute_list_of_grasps(object_manipulation_msgs::ClusterBoundingBox bounding_box);

  protected:
    geometry_msgs::Pose compute_pose(unsigned int index_pose, bool is_vertical,
                                      object_manipulation_msgs::ClusterBoundingBox bounding_box);

    Eigen::Vector3d get_main_axis(object_manipulation_msgs::ClusterBoundingBox bbox);

    sensor_msgs::JointState pregrasp, grasp;

    static const double default_approach_distance;
    static const unsigned short default_number_of_computed_grasps;

    /* A transform listener */
    boost::shared_ptr<tf::TransformListener> tf_listener;
    /* A transform broadcaster to broadcast the object pose*/
    boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
