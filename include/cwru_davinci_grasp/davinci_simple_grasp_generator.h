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
   Desc:   Grasp generator for generating simple grasping trajectory
*/

#ifndef CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_GRASP_GENERATOR_H
#define CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_GRASP_GENERATOR_H

#include <convenience_ros_functions/ROSFunctions.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/Grasp.h>
#include <cwru_davinci_grasp/davinci_needle_grasp_data.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace cwru_davinci_grasp
{

struct GraspInfo
{
  Eigen::Affine3d grasp_pose;
  int part_id;
};

/**
 * helper functions to generator simple Grasp Message objects.
 */
class DavinciSimpleGraspGenerator
{
public:
  DavinciSimpleGraspGenerator(moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                              bool verbose = false);
  ~DavinciSimpleGraspGenerator();

  /**
   * @brief generate simple possible needle grasps list
   * @param needle_pose
   * @param needleGraspData
   * @param possible_grasp_msgs
   * @return
   */
  bool generateSimpleNeedleGrasps(const geometry_msgs::PoseStamped &needle_pose,
                                  const DavinciNeedleGraspData &needleGraspData,
                                  std::vector<moveit_msgs::Grasp> &possible_grasp_msgs);



  /**
   * @brief generate a simple needle grasp with user defined grasping parameters
   * @param needle_pose
   * @param needleGraspData
   * @param possible_grasps
   * @return
   */
  bool generateDefinedSimpleNeedleGrasp(const geometry_msgs::PoseStamped &needle_pose,
                                        const DavinciNeedleGraspData &needleGraspData,
                                        moveit_msgs::Grasp &possible_grasp_msg,
                                        GraspInfo &grasp_pose,
                                        bool has_grasp_pose = false);

  void graspGeneratorHelper(const geometry_msgs::PoseStamped &needle_pose,
                            const DavinciNeedleGraspData &needleGraspData,
                            std::vector<GraspInfo> &grasp_pose);
private:

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // display more output both in console
  bool verbose_;

  // Transform from frame of needle to global frame(base frame)
  Eigen::Affine3d needle_pose_wrt_base_frame_;


//  static trajectory_msgs::JointTrajectory simpleGrasp(const std::vector<std::string> joint_names, float pos, float effort=0);

  /**
   * Compute the desired needle pose wrt to gripper frame by providing a struct type variable
   * @param grasping_parameters
   * @param needle_geometry_parameters
   * @return A 4-by-4 matrix represents desired needle pose wrt to gripper frame
   */
  void calNeedleToGripperPose(const double (&grasping_parameters)[4],
                              const double &needle_radius,
                              GraspInfo &grasp_info);


  /**
   * Compute pre-grasping gripper pose w/rt needle frame
   * @param grasping_parameters
   * @param needle_radius
   * @param pose_above_object
   * @return A 4-by-4 matrix represents pre-grasping gripper pose w/rt needle frame
   */
  static Eigen::Affine3d calPreGraspGripperPose(const double grasping_parameters[],
                                                const double needle_radius,
                                                const float pose_above_needle_dist);

};

typedef boost::shared_ptr<DavinciSimpleGraspGenerator> DavinciSimpleGraspGeneratorPtr;
typedef boost::shared_ptr<const DavinciSimpleGraspGenerator> DavinciSimpleGraspGeneratorConstPtr;

}  // namespace

#endif  // CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_GRASP_GENERATOR_H
