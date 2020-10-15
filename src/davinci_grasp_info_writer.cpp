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
 * Description: The main program writes grasp info array to files
 */

#include <cwru_davinci_grasp/davinci_needle_grasper_base.h>
#include <fstream>

using namespace cwru_davinci_grasp;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_grasp_info_writer");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");

  std::string packPath;
  std::string planning_group_name;
  std::string ee_group_name;
  node_handle_priv.getParam("packPath", packPath);
  node_handle_priv.getParam("planning_group_name", planning_group_name);
  node_handle_priv.getParam("ee_group_name", ee_group_name);
  DavinciNeedleGrasperBasePtr pNeedleGrasper(new DavinciNeedleGrasperBase(node_handle_priv,
                                                                          planning_group_name,
                                                                          ee_group_name));

  const std::vector<GraspInfo> graspInfoArray = pNeedleGrasper->getAllPossibleNeedleGrasps();

  std::ofstream outFile(packPath +"/../" + "AllNeedleGraspTransformations.txt");
  // the important part
  for (size_t i = 0; i < graspInfoArray.size(); ++i)
  {
    outFile << "grasp index " << i << "\n";
    outFile << "grasp part " << graspInfoArray[i].part_id << "\n";
    outFile << "grasp translation: " << "\n";
    outFile << "x: " << graspInfoArray[i].grasp_pose.translation().x() << "\n";
    outFile << "y: " << graspInfoArray[i].grasp_pose.translation().y() << "\n";
    outFile << "z: " << graspInfoArray[i].grasp_pose.translation().z() << "\n";
    outFile << "grasp orientation: " << "\n";
    outFile << graspInfoArray[i].grasp_pose.rotation() << "\n";
    outFile << "-------------------------------" << "\n";
  }
  
  outFile.close();
  return 0;
}
