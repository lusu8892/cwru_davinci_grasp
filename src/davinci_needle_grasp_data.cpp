/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western Reserve University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Su Lu <sxl924@case.edu>
 * Description: Data class used by generating grasping
 */

#include <cwru_davinci_grasp/davinci_needle_grasp_data.h>

namespace cwru_davinci_grasp
{
DavinciNeedleGraspData::DavinciNeedleGraspData()
  : base_link_("/base_link")
{
  // left blank
}

bool DavinciNeedleGraspData::loadRobotGraspData(const ros::NodeHandle &nh,
                                                const std::string &end_effector)
{
  std::vector<std::string> joint_names;
  std::vector<double> pre_grasp_posture;
  std::vector<double> grasp_posture;
  double pregrasp_time_from_start;
  double grasp_time_from_start;
  std::string end_effector_name;
  std::string end_effector_parent_link;
  std::string end_effector_tool_tip_link;

  // needle grasping parameters
  std::vector<double> theta_normal;
  std::vector<double> theta_limits;
  std::vector<double> theta_resolution;


  if (!nh.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter `base_link` missing "
                             "from rosparam server. "
                             "Did you load your end effector's configuration "
                             "yaml file? Searching in namespace: "
                             << nh.getNamespace());
    return false;
  }
  nh.getParam("base_link", base_link_);

  if (!nh.hasParam("needle_radius"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter `needle_radius` missing "
                             "from rosparam server. "
                             "Did you load your end effector's configuration "
                             "yaml file? Searching in namespace: "
                             << nh.getNamespace());
    return false;
  }
  nh.getParam("needle_radius", needle_radius_);

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh, end_effector);

  // Load a param
  if (!child_nh.hasParam("pregrasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`pregrasp_time_from_start` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file? Searching in namespace: "
                             << child_nh.getNamespace());
    return false;
  }
  child_nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start);

  // Load a param
  if (!child_nh.hasParam("pregrasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`grasp_time_from_start` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file?");
    return false;
  }
  child_nh.getParam("grasp_time_from_start", grasp_time_from_start);

  // Load a param
  if (!child_nh.hasParam("end_effector_name"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter `end_effector_name` "
                             "missing from rosparam server. Did you load your "
                             "end effector's configuration yaml file?");
    return false;
  }
  child_nh.getParam("end_effector_name", end_effector_name);

  // Load a param
  if (!child_nh.hasParam("end_effector_parent_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`end_effector_parent_link` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file?");
    return false;
  }
  child_nh.getParam("end_effector_parent_link", end_effector_parent_link);

  // Load a param
  if (!child_nh.hasParam("end_effector_tool_tip_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`end_effector_tool_tip_link` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file?");
    return false;
  }
  child_nh.getParam("end_effector_tool_tip_link", end_effector_tool_tip_link);

  // Load a param
  if (!child_nh.hasParam("approach_retreat_desired_dist"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`approach_retreat_desired_dist` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file?");
    return false;
  }
  child_nh.getParam("approach_retreat_desired_dist", approach_retreat_desired_dist_);

  // Load a param
  if (!child_nh.hasParam("approach_retreat_min_dist"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`approach_retreat_min_dist` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file?");
    return false;
  }
  child_nh.getParam("approach_retreat_min_dist", approach_retreat_min_dist_);


  // Load a param
  if (!child_nh.hasParam("angle_resolution"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter "
                             "`angle_resolution` missing from rosparam "
                             "server. Did you load your end effector's "
                             "configuration yaml file?");
    return false;
  }
  child_nh.getParam("angle_resolution", angle_resolution_);

  // Load a param
  if (!child_nh.hasParam("joints"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader",
                           "Grasp configuration parameter `joints` missing "
                             "from rosparam server. Did you load your end "
                             "effector's configuration yaml file?");
    return false;
  }

  XmlRpc::XmlRpcValue joint_list;
  child_nh.getParam("joints", joint_list);
  if (joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int32_t i = 0; i < joint_list.size(); ++i)
    {
      ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      joint_names.push_back(static_cast<std::string>(joint_list[i]));
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("temp", "joint list type is not type array???");
  }

  if (child_nh.hasParam("pregrasp_posture"))
  {
    XmlRpc::XmlRpcValue preg_posture_list;
    child_nh.getParam("pregrasp_posture", preg_posture_list);

    ROS_ASSERT(preg_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < preg_posture_list.size(); ++i)
    {
      ROS_ASSERT(preg_posture_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      pre_grasp_posture.push_back(static_cast<double>(preg_posture_list[i]));
    }
  }

  ROS_ASSERT(child_nh.hasParam("grasp_posture"));
  XmlRpc::XmlRpcValue grasp_posture_list;
  child_nh.getParam("grasp_posture", grasp_posture_list);
  ROS_ASSERT(grasp_posture_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < grasp_posture_list.size(); ++i)
  {
    ROS_ASSERT(grasp_posture_list[i].getType() ==
               XmlRpc::XmlRpcValue::TypeDouble);
    grasp_posture.push_back(static_cast<double>(grasp_posture_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("theta_normal"));
  XmlRpc::XmlRpcValue theta_normal_list;
  child_nh.getParam("theta_normal", theta_normal_list);
  ROS_ASSERT(theta_normal_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < theta_normal_list.size(); ++i)
  {
    ROS_ASSERT(theta_normal_list[i].getType() ==
               XmlRpc::XmlRpcValue::TypeDouble);
    theta_normal.push_back(static_cast<double>(theta_normal_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("theta_limits"));
  XmlRpc::XmlRpcValue theta_limits_list;
  child_nh.getParam("theta_limits", theta_limits_list);
  ROS_ASSERT(theta_limits_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < theta_limits_list.size(); ++i)
  {
    ROS_ASSERT(theta_limits_list[i].getType() ==
               XmlRpc::XmlRpcValue::TypeDouble);
    theta_limits.push_back(static_cast<double>(theta_limits_list[i]));
  }

  ROS_ASSERT(child_nh.hasParam("theta_resolution"));
  XmlRpc::XmlRpcValue theta_resolution_list;
  child_nh.getParam("theta_resolution", theta_resolution_list);
  ROS_ASSERT(theta_resolution_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < theta_resolution_list.size(); ++i)
  {
    ROS_ASSERT(theta_resolution_list[i].getType() ==
               XmlRpc::XmlRpcValue::TypeDouble);
    theta_resolution.push_back(static_cast<double>(theta_resolution_list[i]));
  }

  // -------------------------------
  // Create pre-grasp posture if specified
  if (!pre_grasp_posture.empty())
  {
    pre_grasp_posture_.header.frame_id = base_link_;
    pre_grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    pre_grasp_posture_.joint_names = joint_names;
    // Position of joints
    pre_grasp_posture_.points.resize(1);
    pre_grasp_posture_.points[0].positions = pre_grasp_posture;
    pre_grasp_posture_.points[0].time_from_start =
      ros::Duration(pregrasp_time_from_start);
  }

  // -------------------------------
  // Create grasp posture
  grasp_posture_.header.frame_id = base_link_;
  grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = grasp_posture;
  grasp_posture_.points[0].time_from_start =
    ros::Duration(grasp_time_from_start);

  // -------------------------------
  // SRDF Info
  ee_parent_link_ = end_effector_parent_link;
  ee_tool_tip_link_ = end_effector_tool_tip_link;
  ee_group_ = end_effector_name;

  // -------------------------------
  // Nums
  //  approach_retreat_desired_dist_ = 0.01; // 0.3;
  //  approach_retreat_min_dist_ = 0.005;

  // TODO this grasp_depth_ variable may not be needed.
  //  // distance from center point of object to end effector
  //  grasp_depth_ = 0.06;// in negative or 0 this makes the grasps on the other
  //  side of the object! (like from below)

  // generate grasps at PI/angle_resolution increments
  //  angle_resolution_ = 16; // TODO parametrize this, or move to action interface

  grasp_theta_0_ = theta_normal_list[0];
  grasp_theta_1_ = theta_normal_list[1];
  grasp_theta_2_ = theta_normal_list[2];
  grasp_theta_3_ = theta_normal_list[3];

  grasp_theta_0_delta_ = theta_resolution_list[0];
  grasp_theta_1_delta_ = theta_resolution_list[1];
  grasp_theta_2_delta_ = theta_resolution_list[2];
  grasp_theta_3_delta_ = theta_resolution_list[3];

  grasp_theta_0_min_ = theta_limits_list[0];
  grasp_theta_0_max_ = theta_limits_list[1];
  grasp_theta_1_min_ = theta_limits_list[2];
  grasp_theta_1_max_ = theta_limits_list[3];
  grasp_theta_2_min_ = theta_limits_list[4];
  grasp_theta_2_max_ = theta_limits_list[5];
  grasp_theta_3_min_ = theta_limits_list[6];
  grasp_theta_3_max_ = theta_limits_list[7];

  //  needle_radius_ = 0.012;
  fillDataInGraspingParametersList();
  // Debug
  // moveit_simple_grasps::SimpleGrasps::printObjectGraspData(grasp_data);
  return true;
}

bool DavinciNeedleGraspData::setRobotStatePreGrasp(
  robot_state::RobotStatePtr &robot_state)
{
  return setRobotState(robot_state, pre_grasp_posture_);
}

bool DavinciNeedleGraspData::setRobotStateGrasp(
  robot_state::RobotStatePtr &robot_state)
{
  return setRobotState(robot_state, grasp_posture_);
}

bool DavinciNeedleGraspData::setRobotState(
  robot_state::RobotStatePtr &robot_state,
  const trajectory_msgs::JointTrajectory &posture)
{
  // do for every joint in end effector
  for (std::size_t i = 0; posture.joint_names.size(); ++i)
  {
    // Debug
    std::cout << "Setting joint " << posture.joint_names[i] << " to value "
              << posture.points[i].positions[0] << std::endl;

    // Set joint position
    robot_state->setJointPositions(posture.joint_names[i],
                                   posture.points[i].positions);
  }
}

void DavinciNeedleGraspData::print()
{
  ROS_WARN_STREAM_NAMED("davinci_needle_grasp_data", "Debug Grasp Data variable values:");
//  std::cout << "grasp_pose_to_eef_pose_: \n" <<grasp_pose_to_eef_pose_<<std::endl;
  std::cout << "pre_grasp_posture_: \n" << pre_grasp_posture_ << std::endl;
  std::cout << "grasp_posture_: \n" << grasp_posture_ << std::endl;
  std::cout << "base_link_: " << base_link_ << std::endl;
  std::cout << "ee_parent_link_: " << ee_parent_link_ << std::endl;
  std::cout << "ee_group_: " << ee_group_ << std::endl;
//  std::cout << "grasp_depth_: " <<grasp_depth_<<std::endl;
  std::cout << "angle_resolution_: " << angle_resolution_ << std::endl;
  std::cout << "approach_retreat_desired_dist_: " << approach_retreat_desired_dist_ << std::endl;
  std::cout << "approach_retreat_min_dist_: " << approach_retreat_min_dist_ << std::endl;
//  std::cout << "object_size_: " <<object_size_<<std::endl;
}


void DavinciNeedleGraspData::fillDataInGraspingParametersList()
{
  double theta_0 = grasp_theta_0_min_;
  grasp_theta_0_list_.clear();
  while (theta_0 >= grasp_theta_0_min_ && theta_0 <= grasp_theta_0_max_)
  {
    grasp_theta_0_list_.push_back(theta_0);
    theta_0 += grasp_theta_0_delta_;
  }

  double theta_1 = grasp_theta_1_min_;
  grasp_theta_1_list_.clear();
  while (theta_1 >= grasp_theta_1_min_ && theta_1 <= grasp_theta_1_max_)
  {
    grasp_theta_1_list_.push_back(theta_1);
    theta_1 += grasp_theta_1_delta_;
  }

  double theta_2 = grasp_theta_2_min_;
  grasp_theta_2_list_.clear();
  while (theta_2 >= grasp_theta_2_min_ && theta_2 <= grasp_theta_2_max_)
  {
    grasp_theta_2_list_.push_back(theta_2);
    theta_2 += grasp_theta_2_delta_;
  }

  double theta_3 = grasp_theta_3_min_;
  grasp_theta_3_list_.clear();
  while (theta_3 >= grasp_theta_3_min_ && theta_3 <= grasp_theta_3_max_)
  {
    grasp_theta_3_list_.push_back(theta_3);
    theta_3 += grasp_theta_3_delta_;
  }
}

}
