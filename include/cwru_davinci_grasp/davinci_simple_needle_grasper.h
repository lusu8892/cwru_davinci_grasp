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
 * Description: A need grasping execution routine which uses MoveIt motion
 * planner
 * to control da Vinci PSMs to grasp needle with a defined needle grasping
 * algorithm.
 */

#ifndef CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_NEEDLE_GRASPER_H
#define CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_NEEDLE_GRASPER_H

// ROS
#include <ros/ros.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Grasp generation
#include <cwru_davinci_grasp/davinci_simple_grasp_generator.h>
#include <cwru_davinci_grasp/davinci_needle_grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasp
#include <moveit_msgs/AttachedCollisionObject.h>

#include <cwru_davinci_moveit_object_handling/davinci_grasped_object_handler.h>

namespace cwru_davinci_grasp
{

struct NeedleModel
{
  std::string name;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
};

//  scoped enum definition for user to choose needle pick mode
enum class NeedlePickMode { RANDOM, DEFINED };

class DavinciSimpleNeedleGrasper
{
private:

  typedef std::map<std::string, moveit_msgs::CollisionObject> ObjsCheckMap;
  typedef std::map<std::string, moveit_msgs::AttachedCollisionObject> AttachedObjsCheckMap;

public:
  DavinciSimpleNeedleGrasper(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_priv,
                             const std::string &planning_group_name,
                             const std::string &needle_name,
                             const std::string &get_planning_scene_service = "get_planning_scene",
                             const std::string &set_planning_scene_topic = "planning_scene",
                             const std::string &updated_needle_pose_topic = "updated_needle_pose");

  ~DavinciSimpleNeedleGrasper();

  /**
   * @brief pick needle by using member variable @var needle_pose_
   * @param needle_name
   * @param mode
   * @return
   */
  bool pickNeedle(const std::string &needle_name, const NeedlePickMode &mode);


  /**
   * @brief pick up needle @param needle_name from planning scene
   * @param needle_pose
   * @param needle_name
   * @return
   */
  bool pickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                  const std::string &needle_name,
                  const NeedlePickMode &mode);

  /**
   * @brief place needle
   * @param needle_pose
   * @param needle_name
   * @return
   */
  bool placeNeedle(const geometry_msgs::Pose &needle_goal_pose,
                         const std::string &needle_name);

  /**
   * @brief get all possible needle grasps messages
   * @param needle_pose
   * @param needleGraspData
   * @return
   */
  std::vector<moveit_msgs::Grasp> getAllPossibleNeedleGrasps(const geometry_msgs::PoseStamped &needle_pose,
                                                             const DavinciNeeldeGraspData &needleGraspData);

  /**
   * @brief get the defind needle grasp message
   * @param needle_pose
   * @param needleGraspData
   * @return
   */
  moveit_msgs::Grasp getDefinedNeedleGrasp(const geometry_msgs::PoseStamped &needle_pose,
                                           const DavinciNeeldeGraspData &needleGraspData);

private:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::string ee_group_name_;  // end-effector group
  std::string planning_group_name_;  // the planning group normally it is the parent group of end-effector
  std::string needle_name_;

  ros::NodeHandle nh_;

  ros::NodeHandle nh_priv_;

  ros::ServiceClient moveit_planning_scene_diff_client_;
  ros::Publisher moveit_planning_scene_diff_publisher_;

  ros::Subscriber needle_pose_sub_;

  geometry_msgs::PoseStamped needle_pose_;

  moveit_msgs::CollisionObject needle_collision_model_;

  // interface to MoveIt
  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  boost::scoped_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  ObjsCheckMap objsCheckMap_;

  AttachedObjsCheckMap attachedObjsCheckMap_;

  davinci_moveit_object_handling::DavinciMoveitGraspedObjectHandlerPtr davinciNeedleHandler_;

  DavinciNeeldeGraspData needleGraspData_;

  DavinciSimpleGraspGeneratorPtr simpleNeedleGraspGenerator_;

  std::vector<moveit_msgs::Grasp> possible_grasps_;

  moveit_msgs::Grasp defined_grasp_;


  /**
   * @brief pick up needle by using possible grasping poses list
   * @param needle_pose
   * @param needle_name
   * @return
   */
  moveit_msgs::MoveItErrorCodes randomPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                                 const std::string &needle_name);

  /**
   * @brief pick up needle by user defined grasping parameter
   * @param needle_pose
   * @param needle_name
   * @return
   */
  moveit_msgs::MoveItErrorCodes definedPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                                  const std::string &needle_name);

  /**
   * @brief helper function to place needle
   * @param needle_goal_pose
   * @param needle_name
   * @return
   */
  moveit_msgs::MoveItErrorCodes placeNeedleHelper(const geometry_msgs::Pose &needle_goal_pose,
                                                  const std::string &needle_name);

  /**
   * @brief to add a needle collision model in planning scene
   * @param needle_origin
   * @param needle_name
   * @return
   */
  bool addNeedleToPlanningScene(const geometry_msgs::PoseStamped &needle_origin,
                                const std::string &needle_name);

  /**
   * @brief to remove needle @param needle_name from planning scene.
   *        This will also detach needle @param needle_name from robot attached objects list
   * @param needle_name
   * @return
   */
  bool removeNeedleFromPlanningScene(const std::string &needle_name);

  /**
   * @brief to get needle collision model
   * @return
   */
  bool getNeedleCollisionModel();

  /**
   * @brief generate needle collision model based on its origin
   * @param needle_origin
   * @param needle_name
   * @return true if needle able to be generated
   */
  bool generateNeedleCollisionModel(const geometry_msgs::PoseStamped &needle_origin,
                                    const std::string &needle_name);

//  /**
//   * @brief updates pose data, leaves all other fields untouched
//   */
//  void updatePose(const geometry_msgs::PoseStamped& new_needle_origin,
//                  moveit_msgs::CollisionObject& obj);

  /**
   * @brief receive latest needle origin pose
   * @param needle_pose
   */
  void needlePoseCallBack(const geometry_msgs::PoseStamped& needle_pose);
};

}

#endif // CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_NEEDLE_GRASPER_H
