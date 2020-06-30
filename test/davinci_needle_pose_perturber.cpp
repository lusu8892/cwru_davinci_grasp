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
 * Description: A needle pose perturber for testing needle pose perturbation in gazebo.
 * This node is to perturb the needle pose.
 */

#include <cwru_davinci_grasp/davinci_needle_pose_publisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_needle_pose_perturber");

  ros::NodeHandle nh;
  ros::Rate loopRate(0.1);
  DummyNeedleTracker needleTracker(nh);

  // grasp_pose
  // part_id:0
  // graspParamInfo
  // param_0_index:1
  // param_1_index:0
  // param_2_index:1
  // param_3_index:0
  // grasp_id:56
  // param_0:0.0030000000000000001
  // param_1:0
  // param_2:-2.0943399999999999
  // param_3:0.16667000000000001
  // theta_diff_avg:0.054442750000000005

  // grasp_pose
  // part_id:0
  // graspParamInfo
  // param_0_index:0
  // param_1_index:0
  // param_2_index:0
  // param_3_index:0
  // grasp_id:0
  // param_0:0.002
  // param_1:0
  // param_2:-3.1415000000000002
  // param_3:0.16667000000000001
  // theta_diff_avg:0.080871750000000006

  cwru_davinci_grasp::GraspInfo graspInfo1;
  graspInfo1.graspParamInfo.grasp_id = 56;

  graspInfo1.graspParamInfo.param_0_index = 1;
  graspInfo1.graspParamInfo.param_1_index = 0;
  graspInfo1.graspParamInfo.param_2_index = 1;
  graspInfo1.graspParamInfo.param_3_index = 0;

  graspInfo1.graspParamInfo.param_0 = 0.003000000000000000;
  graspInfo1.graspParamInfo.param_1 = 0.0;
  graspInfo1.graspParamInfo.param_2 = -2.0943399999999999;
  graspInfo1.graspParamInfo.param_3 = 0.16667000000000001;
  graspInfo1.theta_diff_avg = 0.054442750000000005;

  cwru_davinci_grasp::GraspInfo graspInfo2;
  graspInfo2.graspParamInfo.grasp_id = 0;

  graspInfo2.graspParamInfo.param_0_index = 0;
  graspInfo2.graspParamInfo.param_1_index = 0;
  graspInfo2.graspParamInfo.param_2_index = 0;
  graspInfo2.graspParamInfo.param_3_index = 0;

  graspInfo2.graspParamInfo.param_0 = 0.002;
  graspInfo2.graspParamInfo.param_1 = 0.0;
  graspInfo2.graspParamInfo.param_2 = -3.1415000000000002;
  graspInfo2.graspParamInfo.param_3 = 0.16667000000000001;
  graspInfo2.theta_diff_avg = 0.054442750000000005;
  while (ros::ok())
  {
    needleTracker.perturbNeedlePose(0.1, graspInfo2);
    ros::spinOnce();
    ros::Duration(2.0).sleep();

    needleTracker.perturbNeedlePose(-0.1, graspInfo2);
    ros::spinOnce();
    ros::Duration(2.0).sleep();
  }
  return 0;
}