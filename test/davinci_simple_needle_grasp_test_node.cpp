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
 * Description: A simple needle grasping test
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_simple_needle_grasp_test_node");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");

  ros::AsyncSpinner spinner(1);
  ros::Duration(5.0).sleep();
  spinner.start();

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("world"));
  visual_tools->deleteAllMarkers();

  // Rviz provides many types of markers, in this demo we will use text,
  // cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();

  text_pose.translation().z() = 1.75; // a magic number need to be changed later

  visual_tools->publishText(text_pose, "davinci Simple Needle Grasping Testing",
                            rviz_visual_tools::WHITE,
                            rviz_visual_tools::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to
  // Rviz for large visualizations
  visual_tools->trigger();

  cwru_davinci_grasp::DavinciSimpleNeedleGrasper needleGrasper(node_handle, node_handle_priv, "psm_one",
                                           "/get_planning_scene", "/planning_scene", "/updated_needle_pose");

  geometry_msgs::PoseStamped needle_pose;
  needle_pose.header.stamp = ros::Time::now();
  needle_pose.header.frame_id = "davinci_endo_cam_l";
  needle_pose.pose.position.x = -0.248;
  needle_pose.pose.position.y = 0.0;
  needle_pose.pose.position.z = 0.45;
  needle_pose.pose.orientation.w = 1;
  needle_pose.pose.orientation.x = 0;
  needle_pose.pose.orientation.y = 0;
  needle_pose.pose.orientation.z = 0;

  cwru_davinci_grasp::GraspInfo empty_grasp_pose;
  needleGrasper.pickNeedle(needle_pose, "needle_r", cwru_davinci_grasp::NeedlePickMode::DEFINED,
                           empty_grasp_pose, false, false);


  while (ros::ok())
  {
    ros::spinOnce();
  }
}

