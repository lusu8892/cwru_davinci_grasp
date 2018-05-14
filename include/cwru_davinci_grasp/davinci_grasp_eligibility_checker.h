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


#ifndef CWRU_DAVINCI_GRASP_DAVINCI_GRASP_ELIGIBILITY_CHECKER_H
#define CWRU_DAVINCI_GRASP_DAVINCI_GRASP_ELIGIBILITY_CHECKER_H

#include <cwru_davinci_grasp/GraspAction.h>
#include <convenience_ros_functions/TypedSubscriber.h>

namespace cwru_davinci_grasp
{
  class DavinciGraspEligibilityChecker
  {
  protected:
    typedef cwru_davinci_grasp::GraspGoal GraspGoalT;

  public:

    DavinciGraspEligibilityChecker(ros::NodeHandle &nodeHandle, const float &effector_pos_accuracy,
                                   const float &effector_ori_accuracy, const float &joint_angles_accuracy);

    virtual ~DavinciGraspEligibilityChecker();

    /**
     *
     * @return true if the implementaiton also checks for correct joint states
     * as precondition to the grasp/ungrasp. If it returns false, only the
     * end effector pose is checked to be as expected.
     */
    virtual bool checkJointStates();

    /**
     * connects subscriber to receive most recent joint states. This only make
     * sense if checkJointStates() returns true, and if the the Grasp actions
     * sent to not already contain the most recent joint state (i.e. whichever
     * node generates the grasp action requests, does leave the field
     * Grasp::curr_joint_state uninitialized
     * @param joint_states_topic
     */
    void connectSubscriber(const std::string& joint_states_topic);

    /**
     *
     * @param graspGoal
     * @return true for action eligible, false action not eligible because arm not in required state
     */
    virtual bool executionEligible(const GraspGoalT& graspGoal);

  protected:

    /**
     * Uses /tf to determine eligibility. The pose of @param effector_link_frame (in /tf)
     * is compared to @param grasp_pose
     * @param effector_link_frame
     * @param grasp_pose the expected pose the end-effector (frame @param effector_link_frame)
     *                                  has to be at to perform grasp
     * @param effector_pos_accuracy
     * @param effector_ori_accuracy
     * @return
     */
    bool graspExecutionEligible(const std::string &effector_link_frame, const geometry_msgs::PoseStamped &grasp_pose,
                                const float &effector_pos_accuracy, const float &effector_ori_accuracy);


    /**
     * Checks for consistency in the grasp goal. In particular, checks wheter the jointTrajectory (grasp_trajectory)
     * relating to the grasp conforms to the fields in the moveit_msgs::Grasp, i.e. the last trajectory point
     * has to be the same as the moveit_msgs::Grasp::pre_grasp_posture.
     * @param graspGoal
     * @param use_joint_angles_accuracy the tolerance to use for comparing joint states
     * @return
     */
    bool goalJointStatesConsistent(const GraspGoalT& graspGoal, const float use_joint_angles_accuracy) const;

    // default tolerance for the effector target pose and actual effector pose to be
    // similar enough to accept a grasp action.
    float effector_pos_accuracy_;

    // default tolerance for the effector target orientation and actual effector orientation to be
    // similar enough to accept a grasp action (in rad).
    float effector_ori_accuracy_;

    // default tolerance for the joints (in rad) when checking against joint state targets
    float joint_angles_accuracy_;

    typedef convenience_ros_functions::TypedSubscriber<sensor_msgs::JointState> jointStateSubscriberT;

    jointStateSubscriberT joint_states_sub_;

//    bool convertJointTrajToJointState(const trajectory_msgs::JointTrajectory &joint_traj,
//                                      sensor_msgs::JointState &joint_state);
  };  // class

}  // namespace


#endif //CWRU_DAVINCI_GRASP_DAVINCI_GRASP_ELIGIBILITY_CHECKER_H
