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
 * Description: The main program executing needle grasping and placing
 */

#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace cwru_davinci_grasp;
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << "<which_arm> <needle_name> [place]"
              << std::endl;
    return 1;
  }

  std::string which_arm = argv[1];
  std::string needle_name = argv[2];

  bool is_place = false;
  if (argc > 3)
  {
    std::string arg = argv[3];
    if (arg == "place")
    {
      ROS_INFO("Place %s", needle_name.c_str());
      is_place = true;
    }
  }
  else
  {
    ROS_INFO("Pick %s", needle_name.c_str());
  }

  ros::init(argc, argv, "davinci_simple_needle_grasp_main_node");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");

  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
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

  DavinciSimpleNeedleGrasper needleGrasper(node_handle,
                                           node_handle_priv,
                                           which_arm,
                                           needle_name);

  if (!is_place)
  {
    int pick_mode = 0;

    std::cout << "How do you want to pick needle 0 for DEFINED pick, 1 for OPTIMAL pick, 2 for FINDGOOD pick, other number is for RANDOM pick: ";
    std::cin >> pick_mode;
    if (pick_mode == 0)
    {
      // defined needle pick up
      if(!needleGrasper.pickNeedle(NeedlePickMode::DEFINED, needle_name))
      {
        ROS_INFO("Main function: failed to perform DEFINED needle pick up");
        ros::shutdown();
        return 0;
      }
      ROS_INFO("Main function: successfully performed DEFINED needle pick up");
    }
    else if(pick_mode == 1)
    {
      // defined needle pick up
      if(!needleGrasper.pickNeedle(NeedlePickMode::OPTIMAL, needle_name))
      {
        ROS_INFO("Main function: failed to perform OPTIMAL needle pick up");
        ros::shutdown();
        return 0;
      }
      ROS_INFO("Main function: successfully performed OPTIMAL needle pick up");
      ROS_INFO("Main function: the selected grasp index is: %d",
               needleGrasper.getSelectedGraspInfo().graspParamInfo.grasp_id);
    }
    else if(pick_mode == 2)
    {
      // defined needle pick up
      if(!needleGrasper.pickNeedle(NeedlePickMode::FINDGOOD, needle_name))
      {
        ROS_INFO("Main function: failed to perform FINDGOOD needle pick up");
        ros::shutdown();
        return 0;
      }
      ROS_INFO("Main function: successfully performed FINDGOOD needle pick up");
      ROS_INFO("Main function: the selected grasp index is: %d",
               needleGrasper.getSelectedGraspInfo().graspParamInfo.grasp_id);
    }
    else
    {
      // random needle pick up
      if (!needleGrasper.pickNeedle(NeedlePickMode::RANDOM, needle_name))
      {
        ROS_INFO("Main function: failed to perform RANDOM needle pick up");
        ros::shutdown();
        return 0;
      }
      ROS_INFO("Main function: successfully performed RANDOM needle pick up");
      ROS_INFO("Main function: the selected grasp index is: %d",
               needleGrasper.getSelectedGraspInfo().graspParamInfo.grasp_id);
    }
  }
  else
  {
    std::cout << "Where do you want to place the needle wrt world frame \n";
    std::cout << "Please type in three numbers for position: ";
    double x, y, z;

    std::cin >> x;
    std::cin >> y;
    std::cin >> z;

    geometry_msgs::Pose needle_pose_goal;

    needle_pose_goal.position.x = x;
    needle_pose_goal.position.y = y;
    needle_pose_goal.position.z = z;

    if(!needleGrasper.placeNeedle(needle_pose_goal, needle_name))
    {
      ROS_INFO("Main function: failed to place needle");
      ros::shutdown();
      return 0;
    }
    ROS_INFO("Main function: successfully placed needle to a new location");
  }

  ros::Duration(3.0).sleep();
  ros::shutdown();
  return 0;
}
