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
 * Description: The needle grasper base class for grasp transformation only
 */

#ifndef CWRU_DAVINCI_GRASP_DAVINCI_NEEDLE_GRASPER_BASE_H
#define CWRU_DAVINCI_GRASP_DAVINCI_NEEDLE_GRASPER_BASE_H

// ROS
#include <ros/ros.h>

// Grasp generation
#include <cwru_davinci_grasp/davinci_simple_grasp_generator.h>
#include <cwru_davinci_grasp/davinci_needle_grasp_data.h>
namespace cwru_davinci_grasp
{

class DavinciNeedleGrasperBase
{
public:
  DavinciNeedleGrasperBase(const ros::NodeHandle &nh_priv,
                           const std::string &planning_group_name,
                           const std::string &ee_group_name);

  ~DavinciNeedleGrasperBase(){};

  std::vector<GraspInfo> getAllPossibleNeedleGrasps(bool sort = false);
protected:
  ros::NodeHandle nh_priv_;

  std::string ee_group_name_;  // end-effector group

  std::string planning_group_name_;  // the planning group normally it is the parent group of end-effector

  DavinciNeedleGraspData needleGraspData_;

  DavinciSimpleGraspGeneratorPtr simpleNeedleGraspGenerator_;

  std::vector<GraspInfo> possible_grasps_;
};

typedef boost::shared_ptr<DavinciNeedleGrasperBase> DavinciNeedleGrasperBasePtr;
typedef boost::shared_ptr<const DavinciNeedleGrasperBase> DavinciNeedleGrasperBaseConstPtr;

}

#endif //CWRU_DAVINCI_GRASP_DAVINCI_NEEDLE_GRASPER_BASE_H
