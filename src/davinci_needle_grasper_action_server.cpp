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
   Desc:   Main function of action server to do needle grasping
*/


#include <cwru_davinci_grasp/davinci_needle_grasper_action_server.h>

namespace cwru_davinci_grasp
{

DavinciNeedleGrasperActionServer::DavinciNeedleGrasperActionServer(const ros::NodeHandle &node_handle,
                                                                   const ros::NodeHandle &node_handle_priv,
                                                                   const std::string &which_arm,
                                                                   const std::string &object_name) :
  nh_(node_handle), needle_grasper_action_server_(nh_, "needle_grasper_action_service",
                                                  boost::bind(&DavinciNeedleGrasperActionServer::executeCB, this, _1),
                                                  false)
{

  needleGrasper_.reset(new DavinciSimpleNeedleGrasper(node_handle, node_handle_priv, which_arm, object_name));
  needle_grasper_action_server_.start();
}


void DavinciNeedleGrasperActionServer::executeCB(
  const actionlib::SimpleActionServer<
    cwru_davinci_grasp::NeedleGraspAction>::GoalConstPtr &goal)
{
  sendResultToActionClient(goal);
}


void DavinciNeedleGrasperActionServer::sendResultToActionClient(
  const actionlib::SimpleActionServer<
    cwru_davinci_grasp::NeedleGraspAction>::GoalConstPtr &goal)
{
  geometry_msgs::TransformStamped grasp_pose;
  const std::string which_arm = goal->which_arm;
  const std::string action = goal->action;
  const std::string way_to_pick = goal->way_to_grasp;
  const std::string object_name = goal->object_name;

  int return_val = executeNeedleMove(which_arm, action, way_to_pick, object_name);
  switch (return_val)
  {
    case 0:
      result_.result = NeedleGraspGoal::GRASPING_FAILED;
      break;
    case 1:
      result_.result = NeedleGraspGoal::GRASPING_SUCCESSED_DEFINED_GRASP;
      result_.grasp_transform = needleGrasper_->getGraspTransform();
      break;
    case 2:
      result_.result = NeedleGraspGoal::GRASPING_SUCCESSED_RANDOM_GRASP;
      result_.grasp_transform = needleGrasper_->getGraspTransform();
      break;
    case 3:
      result_.result = NeedleGraspGoal::RELEASING_SUCCESSED;
      break;
    case 4:
      result_.result = NeedleGraspGoal::RELEASING_FAILED;
      break;
  }
  if (return_val < 5 && return_val >= 0)
  {
    ROS_INFO("Sending results back to needle grasper action client");
    needle_grasper_action_server_.setSucceeded(result_);
  }
  else
  {
    ROS_WARN("Informing needle extraction action client of aborted goal");

    // tell the client we have given up on this goal; send the result message as well
    needle_grasper_action_server_.setAborted(result_);
  }
}


int DavinciNeedleGrasperActionServer::executeNeedleMove(
  const std::string &which_arm,
  const std::string &action,
  const std::string &way_to_pick,
  const std::string &object_name)
{
  int able_to_execute = 0;
  needleGrasper_->changePlanningGroup(which_arm);
  if (action == "pick")
  {
    if (way_to_pick == "DEFINED")
      if (needleGrasper_->pickNeedle(NeedlePickMode::DEFINED, object_name))
      {
        able_to_execute = 1;
        return able_to_execute;
      }
      else
      {
        ROS_INFO("NeedleGrasperActionServer: failed to perform DEFINED needle pick up");
        return able_to_execute;
      }
    else if (way_to_pick == "RANDOM")
    {
      if (needleGrasper_->pickNeedle(NeedlePickMode::RANDOM, object_name))
      {
        able_to_execute = 2;
        return able_to_execute;
      }
      else
      {
        ROS_INFO("NeedleGrasperActionServer: failed to perform RANDOM needle pick up");
        return able_to_execute;
      }
    }
  }
  else if (action == "release")
  {
    if (needleGrasper_->releaseNeedle(object_name))
    {
      able_to_execute = 3;
      return able_to_execute;
    }
    else
    {
      ROS_INFO("NeedleGrasperActionServer: failed to perform release grasping of needle");
      able_to_execute = 4;
      return able_to_execute;
    }
  }
  else
  {
    throw std::invalid_argument("The selected arm's gripper can NOT do the action you provided.");
  }

  return 0;
}

}