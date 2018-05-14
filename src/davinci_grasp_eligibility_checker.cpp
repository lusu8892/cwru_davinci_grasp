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
 * Desc: Class that can be used to check for eligibility of grasping and un-grasping actions.
 *
 * The action message to be checked is of type grasp_execution_msgs/Grasp.action. Please also refer
 * to this message for documentation about the inputs / outputs.
 *
 * This class provides the option to subscribe to joint states in order to check eligibility criteria.
 *
 * Different implementations for handling the eligibility checks
 * can be achieved by deriving this class.
 * The default implementation in this base class only checks for the end effector pose and
 * ignores the joint states.
 */

#include <cwru_davinci_grasp/davinci_grasp_eligibility_checker.h>
#include <convenience_ros_functions/ROSFunctions.h>
#include <moveit_msgs/Grasp.h>

namespace cwru_davinci_grasp
{
  DavinciGraspEligibilityChecker::DavinciGraspEligibilityChecker(ros::NodeHandle &nodeHandle,
                                                                 const float &effector_pos_accuracy,
                                                                 const float &effector_ori_accuracy,
                                                                 const float &joint_angles_accuracy) :
    joint_states_sub_(nodeHandle),
    effector_pos_accuracy_(effector_pos_accuracy),
    effector_ori_accuracy_(effector_ori_accuracy),
    joint_angles_accuracy_(joint_angles_accuracy)
  {
    convenience_ros_functions::ROSFunctions::initSingleton();
  }

  DavinciGraspEligibilityChecker::~DavinciGraspEligibilityChecker()
  {
    joint_states_sub_.stop();
  }

  void DavinciGraspEligibilityChecker::connectSubscriber(const std::string &joint_states_topic)
  {
    if (!checkJointStates())
    {
      ROS_WARN_STREAM("Not connecting joint state subscriber because the "
                        << "implementation of this class does not support checking the joint states");
      return;
    }

    joint_states_sub_.start(joint_states_topic);
    joint_states_sub_.setActive(true);
  }

  bool DavinciGraspEligibilityChecker::checkJointStates()
  {
    return false;
  }

  bool DavinciGraspEligibilityChecker::goalJointStatesConsistent(const GraspGoalT &graspGoal,
                                                                 const float use_joint_angles_accuracy) const
  {
    const moveit_msgs::Grasp mgrasp = graspGoal.grasp.grasp;

    const trajectory_msgs::JointTrajectory traj = graspGoal.grasp_trajectory;

    if (traj.points.empty())
    {
      ROS_ERROR("Trajectory for the grasp is empty");
      return false;
    }

    sensor_msgs::JointState last_traj_point;
    if (!convenience_ros_functions::ROSFunctions::getJointStateAt(traj.points.size() - 1, traj, last_traj_point))
    {
      ROS_ERROR("Could not get the last joint state of grasp trajectory");
      return false;
    }

    trajectory_msgs::JointTrajectory temp_traj = graspGoal.is_grasp ? mgrasp.grasp_posture : mgrasp.pre_grasp_posture;

    sensor_msgs::JointState grasp_state;

    if (!convenience_ros_functions::ROSFunctions::getJointStateAt(temp_traj.points.size() - 1, temp_traj, grasp_state))
    {
      ROS_ERROR("Could not get the grasp state");
      return false;
    }

    int ret = convenience_ros_functions::ROSFunctions::equalJointPositions(last_traj_point,
                                                                           grasp_state,
                                                                           use_joint_angles_accuracy);

    if (ret > 0) return true;

    if (ret == -2)
    {
      ROS_ERROR("Grasp pose joint state and last trajectory point do not intersect");
    }
    else if (ret == -3)
    {
      ROS_ERROR("Consistency: intersection of grasp state and last trajectory point not of same size");
    }
    else if (ret == -1)
    {
      ROS_ERROR("Last trajectory point and grasp pose are not similar enough");
    }

    return false;
  }

  bool DavinciGraspEligibilityChecker::executionEligible(const GraspGoalT &graspGoal)
  {
    const cwru_davinci_grasp::GraspData &grasp = graspGoal.grasp;
    const moveit_msgs::Grasp &mgrasp = grasp.grasp;
    const geometry_msgs::PoseStamped &grasp_pose = mgrasp.grasp_pose;
    const std::string & effector_link_name = grasp.effector_link_name;

    float use_pos_acc = effector_pos_accuracy_;
    float use_ori_acc = effector_ori_accuracy_;
    float use_joint_acc = joint_angles_accuracy_;

    if (graspGoal.use_custom_tolerances)
    {
      use_pos_acc = graspGoal.effector_pos_tolerance;
      use_ori_acc = graspGoal.effector_angle_tolerance;
      use_joint_acc = graspGoal.joint_angles_tolerance;
    }

    if (!goalJointStatesConsistent(graspGoal, use_joint_acc))
    {
      ROS_ERROR("Joint states in trajectory and in grasp are not conform");
      return false;
    }

    if (graspGoal.is_grasp || !graspGoal.ignore_effector_pose_ungrasp)
    {
      if (!this -> graspExecutionEligible(grasp.effector_link_name, grasp_pose, use_pos_acc, use_ori_acc))
      {
        return false;
      }
    }

    return true;
  }

  bool DavinciGraspEligibilityChecker::graspExecutionEligible(const std::string &effector_link_frame,
                                                              const geometry_msgs::PoseStamped &grasp_pose,
                                                              const float &effector_pos_accuracy,
                                                              const float &effector_ori_accuracy)
  {
    bool print_errors = true;
    float mas_wait_transform = 2.0;

    // check if the end effector is at the same pose as it is expected to be.
    // First, get the transform from the robot pose frame to the effector frame. This transform
    // corresponds to the absolute effector pose (expressed in the robot position frame).
    geometry_msgs::PoseStamped effector_pose;

    int trans_ret = convenience_ros_functions::ROSFunctions::Singleton()->getTransform(grasp_pose.header.frame_id,
                                                                                       effector_link_frame,
                                                                                       effector_pose.pose,
                                                                                       ros::Time(0),
                                                                                       mas_wait_transform,
                                                                                       print_errors);

    if (trans_ret != 0)
    {
      ROS_ERROR_STREAM("DavinciGraspEligibilityChecker: Cannot get transform from the grasp pose frame ("
                         << graspPose.header.frame_id << ") to the effector link (" << effectorLinkFrame
                         << "), so can't determine effector pose, and not do the grasp");
      return false;
    }



  }

//  bool DavinciGraspEligibilityChecker::convertJointTrajToJointState(const trajectory_msgs::JointTrajectory &joint_traj,
//                                                                    sensor_msgs::JointState &joint_state)
//  {
//    joint_state.name.clear();
//    joint_state.position.clear();
//    joint_state.effort.clear();
//    joint_state.velocity.clear();
//
//    if (!joint_traj.points.empty())
//    {
//      for (int i = 0; i < joint_traj.points.size(); ++i)
//      {
//        sensor_msgs::JointState temp_joint_state;
//        if (convenience_ros_functions::ROSFunctions::getJointStateAt(i, joint_traj, temp_joint_state))
//        {
//
//        }
//      }
//
//      return true;
//    }
//    else
//    {
//      ROS_ERROR("The input joint trajectory has no points");
//      return false;
//    }
//
//
//
//
//
//  }
}  // namespace
