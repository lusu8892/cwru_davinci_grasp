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

#ifndef CWRU_DAVINCI_GRASP_DAVINCI_GRASP_DATA_H
#define CWRU_DAVINCI_GRASP_DAVINCI_GRASP_DATA_H

// Ros
#include <ros/node_handle.h>

// Msgs
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_state/robot_state.h>
namespace cwru_davinci_grasp
{
class DavinciNeedleGraspData
{
public:
  trajectory_msgs::JointTrajectory
      pre_grasp_posture_; // when the end effector is in "open" position
  trajectory_msgs::JointTrajectory
      grasp_posture_; // when the end effector is in "close" position

  std::string base_link_; // name of global frame ( with z axis pointing up), the value will be assigned by the value in MoveIt! package config/.srdf
  std::string ee_parent_link_; // the last link in the kinematic chain before
                               // the end effector
  std::string ee_tool_tip_link_;
  std::string ee_group_;       // the end effector name

  int angle_resolution_; // generate grasps at PI/angle_resolution increments

  double grasp_theta_0_; // the grasp parameter theta_0_ defined by needle
                         // grasping algorithm
  double grasp_theta_1_; // the grasp parameter theta_1_ defined by needle
                         // grasping algorithm
  double grasp_theta_2_; // the grasp parameter theta_2_ defined by needle
                         // grasping algorithm
  double grasp_theta_3_; // the grasp parameter theta_3_ defined by needle
                         // grasping algorithm

  double grasp_theta_0_delta_;

  double grasp_theta_1_delta_;

  double grasp_theta_2_delta_;

  double grasp_theta_3_delta_;

  double grasp_theta_0_min_;

  double grasp_theta_0_max_;

  double grasp_theta_1_min_;

  double grasp_theta_1_max_;

  double grasp_theta_2_min_;

  double grasp_theta_2_max_;

  double grasp_theta_3_min_;

  double grasp_theta_3_max_;

  double weight_0_;
  
  double weight_1_;
  
  double weight_2_;
  
  double weight_3_;

  std::vector<double> grasp_theta_0_list_;

  std::vector<double> grasp_theta_1_list_;

  std::vector<double> grasp_theta_2_list_;

  std::vector<double> grasp_theta_3_list_;

  double approach_desired_dist_; // how far back from the grasp position the pregrasp phase should be
  double approach_min_dist_;     // how far back from grasp position the pregrasp phase should be at minimum

  double retreat_desired_dist_;
  double retreat_min_dist_;

  double object_size_; // for visualization

  double needle_radius_; // for needle grasping
public:
  /**
   * @brief constructor
   * @return
   */
  DavinciNeedleGraspData();

  /**
   * @brief Loads grasp data from a yaml file (load from roslaunch)
   * @param node handle - allows for namespacing
   * @param end effector name - which side of a two handed robot to load data
   * for. should correspond to SRDF EE names
   * @return true on success
   */
  bool loadRobotGraspData(const ros::NodeHandle& nh,
                          const std::string& end_effector);

  /**
   * @brief Alter a robot state so that the end effector corresponding to this
   * grasp data is in pre-grasp state (OPEN)
   * @param joint state of robot
   * @return true on success
   */
  bool setRobotStatePreGrasp(robot_state::RobotStatePtr& robot_state);

  /**
   * @brief Alter a robot state so that the end effector corresponding to this
   * grasp data is in grasp state (CLOSED)
   * @param joint state of robot
   * @return true on success
   */
  bool setRobotStateGrasp(robot_state::RobotStatePtr& robot_state);

  /**
   * @brief Alter a robot state so that the end effector corresponding to this
   * grasp data is in a grasp posture
   * @param joint state of robot
   * @param posture - what state to set the end effector
   * @return true on success
   */
  bool setRobotState(robot_state::RobotStatePtr& robot_state,
                     const trajectory_msgs::JointTrajectory& posture);

  /**
   * @brief Debug data to console
   */
  void print();

private:
  void fillDataInGraspingParametersList();
};

} // namespace

#endif // CWRU_DAVINCI_GRASP_DAVINCI_GRASP_DATA_H
