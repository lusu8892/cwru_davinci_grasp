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

#include <cwru_davinci_grasp/davinci_needle_grasper_base.h>

using namespace cwru_davinci_grasp;

DavinciNeedleGrasperBase::DavinciNeedleGrasperBase(const ros::NodeHandle &nh_priv,
                                                   const std::string &planning_group_name,
                                                   const std::string &ee_group_name)
                                                   : nh_priv_(nh_priv)
{
  nh_priv_.param("planning_group_name", planning_group_name_, planning_group_name);
  nh_priv_.param("ee_group_name", ee_group_name_, ee_group_name);

  if (!needleGraspData_.loadRobotGraspData(nh_priv_, ee_group_name_))
  {
    ros::shutdown();
  }

  simpleNeedleGraspGenerator_.reset(new cwru_davinci_grasp::DavinciSimpleGraspGenerator());
}

std::vector<GraspInfo> DavinciNeedleGrasperBase::getAllPossibleNeedleGrasps(bool sort)
{
  simpleNeedleGraspGenerator_->graspGeneratorHelper(needleGraspData_, possible_grasps_, sort);
  return possible_grasps_;
}
