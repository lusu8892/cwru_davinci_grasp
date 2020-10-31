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
#include <cwru_davinci/uv_control/psm_interface.h>
#include <std_srvs/SetBool.h>
#include <memory>

class UniformRandGenerator
{
public:
  UniformRandGenerator(double min, double max) : m_UniformRealDistribution(min, max){};

  double rand() {return m_UniformRealDistribution(m_Generator);}
private:
  std::default_random_engine m_Generator;
  std::uniform_real_distribution<double> m_UniformRealDistribution;
};

void showStats
(
const std::vector<double>& diffVec
)
{
  double max = *std::max_element(diffVec.begin(), diffVec.end());
  double min = *std::min_element(diffVec.begin(), diffVec.end());
  double accum = std::accumulate(diffVec.begin(), diffVec.end(), 0.0);
  double mean = accum / diffVec.size();
  std::for_each(diffVec.begin(), diffVec.end(), [&](const double d)
  {
    accum += (d - mean) * (d - mean);
  });
  double stdev = sqrt(accum / (diffVec.size() - 1));
  printf("Average Change of Radian: %f \n", mean);
  printf("Maximum Change of Radian: %f \n", max);
  printf("Minimum Change of Radian: %f \n", min);
  printf("Standard Dev Change of Radian: %f \n", stdev);
}

void radianOfChange
(
double& radOfChange,
const Eigen::Quaterniond& q1,
const Eigen::Quaterniond& q2
)
{
  // q_w_ri, q_w_rp, q_ri_rp = q_ri_w * q_w_rp = transpose(q_w_ri) * q_w_rp;
  const Eigen::Quaterniond q = q1.inverse() * q2;
  radOfChange = 2 * acos(q.w());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "needle_perturbation_statistics");

  ros::NodeHandle nh;
  ros::NodeHandle nhPriv("~");
  ros::Rate loopRate(0.1);
  DummyNeedleModifier needleTracker(nh, nhPriv);

  ros::ServiceClient stickyFingerClient = nh.serviceClient<std_srvs::SetBool>("sticky_finger/PSM1_tool_wrist_sca_ee_link_1");

  PSMInterfacePtr pSupportArmGroup = std::make_unique<psm_interface>("psm_one", nh);

//   cwru_davinci_grasp::GraspInfo graspInfo1;
//   graspInfo1.graspParamInfo.grasp_id = 56;

//   graspInfo1.graspParamInfo.param_0_index = 1;
//   graspInfo1.graspParamInfo.param_1_index = 0;
//   graspInfo1.graspParamInfo.param_2_index = 1;
//   graspInfo1.graspParamInfo.param_3_index = 0;

//   graspInfo1.graspParamInfo.param_0 = 0.003000000000000000;
//   graspInfo1.graspParamInfo.param_1 = 0.0;
//   graspInfo1.graspParamInfo.param_2 = -2.0943399999999999;
//   graspInfo1.graspParamInfo.param_3 = 0.16667000000000001;
//   graspInfo1.theta_diff_avg = 0.054442750000000005;

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

  Eigen::Affine3d idealNeedlePose(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0));
  idealNeedlePose.translation().x() = 0.0;
  idealNeedlePose.translation().y() = 0.0;
  idealNeedlePose.translation().z() = 0.18;

  int num = 10;
  std::vector<std::vector<double> > vec;
  std::vector<double> vecDiff;
  vec.resize(num);
  vecDiff.reserve(num);

  std::array<double,4>                                   m_Intervals{{-0.31, -0.29, 0.29, 0.31}};
  std::array<double,3>                                   m_Weights{{50.0, 1.0, 50.0}};
  std::piecewise_constant_distribution<double>           m_PiecewiseDistribution(m_Intervals.begin(), m_Intervals.end(), m_Weights.begin());
  std::random_device                                     m_RandSeed;

  double perturbRadian = m_PiecewiseDistribution(m_RandSeed);

  for (std::size_t i = 0; i < num; ++i)
  {
    perturbRadian = m_PiecewiseDistribution(m_RandSeed);
    vec[i].reserve(2);
    // open jaw;
    pSupportArmGroup->control_jaw(0.3, 0.5);
    // turn off stick finger
    std_srvs::SetBool graspCommand;
    graspCommand.request.data = false;
    stickyFingerClient.call(graspCommand);

    // perturbe needle pose
    needleTracker.perturbNeedlePose(perturbRadian, graspInfo2, idealNeedlePose, true);

    ros::Duration(0.5).sleep();
    // calculate the net of radian of perturbed needle pose
    Eigen::Affine3d perturbedNeedlePose;
    if(!needleTracker.getCurrentNeedlePose(perturbedNeedlePose))
      return false;
    double radOfChange1;
    radianOfChange(radOfChange1,
                   Eigen::Quaterniond(idealNeedlePose.linear()),
                   Eigen::Quaterniond(perturbedNeedlePose.linear()));

    // turn on stick finger
    graspCommand.request.data = true;
    stickyFingerClient.call(graspCommand);
    // close jaw
    pSupportArmGroup->control_jaw(0.0, 0.05);
    ros::Duration(0.5).sleep();

    // calculate the net of radian change after jaw close
    Eigen::Affine3d needlePoseAfterCloseJaw;
    if(!needleTracker.getCurrentNeedlePose(needlePoseAfterCloseJaw))
      return false;
    double radOfChange2;
    radianOfChange(radOfChange2,
                   Eigen::Quaterniond(idealNeedlePose.linear()),
                   Eigen::Quaterniond(needlePoseAfterCloseJaw.linear()));
    double diff = fabs(radOfChange2 - radOfChange1);
    vecDiff.push_back(diff);
    vec[i].push_back(radOfChange1);
    vec[i].push_back(radOfChange2);
  }
//   ros::spinOnce();
//   ros::Duration(2.0).sleep();
  showStats(vecDiff);
  return 0;
}