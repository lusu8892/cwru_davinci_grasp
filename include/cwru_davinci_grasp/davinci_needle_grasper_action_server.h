/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Case Western Reserve University
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
   Desc:   Action server to do needle grasping
*/


#ifndef CWRU_DAVINCI_GRASP_DAVINCI_NEEDLE_GRASPER_ACTION_SERVER_H
#define CWRU_DAVINCI_GRASP_DAVINCI_NEEDLE_GRASPER_ACTION_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

#include <cwru_davinci_grasp/NeedleGraspAction.h>
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

namespace cwru_davinci_grasp
{
class DavinciNeedleGrasperActionServer
{
public:
  DavinciNeedleGrasperActionServer(const ros::NodeHandle &node_handle,
                                   const ros::NodeHandle &node_handle_priv,
                                   const std::string &object_name,
                                   const std::string &which_arm);

  /**
   * Callback function to get the goal messages from action clients.
   * @param goal
   */
  void executeCB(
    const actionlib::SimpleActionServer
      <cwru_davinci_grasp::NeedleGraspAction>::GoalConstPtr &goal);

  /**
   * Help method sending results back to action client
   * @param goal
   */
  void sendResultToActionClient(
    const actionlib::SimpleActionServer
      <cwru_davinci_grasp::NeedleGraspAction>::GoalConstPtr &goal);

  /**
   * Helper Method: Given the arm, try to pick or place the needle
   * @param which_arm "psm_one" or "psm_two"
   * @param action "pick" or "place"
   * @return true if needle is picked or placed successfully
   */
  int executeNeedleMove(const std::string &which_arm,
                        const std::string &action,
                        const std::string &way_to_pick,
                        const std::string &object_name);

private:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer
    <cwru_davinci_grasp::NeedleGraspAction> needle_grasper_action_server_;


  cwru_davinci_grasp::NeedleGraspGoal goal_;

  cwru_davinci_grasp::NeedleGraspFeedback feedback_;

  cwru_davinci_grasp::NeedleGraspResult result_;

  DavinciSimpleNeedleGrasperPtr needleGrasper_;
};
}  // namespace

#endif //CWRU_DAVINCI_GRASP_DAVINCI_NEEDLE_GRASPER_ACTION_SERVER_H
