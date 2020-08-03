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

#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <std_srvs/SetBool.h>
#include <uv_msgs/pf_grasp.h>

using namespace davinci_moveit_object_handling;

namespace cwru_davinci_grasp
{
DavinciSimpleNeedleGrasper::DavinciSimpleNeedleGrasper
(
const ros::NodeHandle &nh,
const ros::NodeHandle &nh_priv,
const std::string &planning_group_name,
const std::string &needle_name,
const std::string &get_planning_scene_service,
const std::string &set_planning_scene_topic,
const std::string &updated_needle_pose_topic
)
: DavinciNeedleGrasperBase(nh_priv, planning_group_name), nh_(nh), needle_name_(needle_name)
{
  ROS_INFO_STREAM_NAMED("DavinciSimpleNeedleGrasper", "Starting Simpling Needle Grasping");

  // Re-reate MoveGroup for one of the planning groups
  move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
  move_group_->setPlanningTime(10.0);

  if(ee_group_name_.empty())
    ee_group_name_ = move_group_->getEndEffector();

  if (!needleGraspData_.loadRobotGraspData(nh_priv_, ee_group_name_))
  {
    ros::shutdown();
  }

  ROS_INFO_STREAM_NAMED("moveit_blocks", "End Effector: " << ee_group_name_);
  ROS_INFO_STREAM_NAMED("moveit_blocks", "Planning Group: " << planning_group_name_);

  // Load grasp generator
  davinciNeedleHandler_.reset(
    new davinci_moveit_object_handling::DavinciMoveitGraspedObjectHandler(nh,
                                                                          (new moveit::planning_interface::MoveGroupInterface(
                                                                            ee_group_name_))->getLinkNames(),
                                                                          get_planning_scene_service,
                                                                          set_planning_scene_topic));

  // Load the Robot Viz Tools for publishing to rviz
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(needleGraspData_.base_link_));

  simpleNeedleGraspGenerator_.reset(new cwru_davinci_grasp::DavinciSimpleGraspGenerator(visual_tools_));

  moveit_planning_scene_diff_publisher_ =
    nh_.advertise<moveit_msgs::PlanningScene>(set_planning_scene_topic, 1);

  moveit_planning_scene_diff_client_ =
    nh_.serviceClient<moveit_msgs::GetPlanningScene>(get_planning_scene_service);

  pf_grasp_client_ =
    nh_.serviceClient<uv_msgs::pf_grasp>("/pf_grasp");

  needle_pose_sub_ =
    nh_.subscribe(updated_needle_pose_topic, 1,
                  &DavinciSimpleNeedleGrasper::needlePoseCallBack, this);

  if(needle_pose_sub_.getNumPublishers() < 1)
  {
    ros::Duration(5.0).sleep();
  }

  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface);

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  planning_scene_monitor_->startStateMonitor();

  ros::Duration(1.0).sleep();
}

bool DavinciSimpleNeedleGrasper::pickNeedle
(
const std::string& needle_name,
const NeedlePickMode mode
)
{
  return pickNeedle(needle_name, needle_pose_, mode, false);
}

bool DavinciSimpleNeedleGrasper::pickNeedle
(
const std::string& needle_name,
const geometry_msgs::PoseStamped& needle_pose,
const NeedlePickMode mode,
bool plan_only
)
{
  bool able_to_pick = false;

  if (hasObject(needle_name, planning_scene_interface_ -> getAttachedObjects()))
  {
    if(!move_group_->detachObject(needle_name))
    {
      ROS_INFO("Needle: %s is already grasped and can not be detached from group %s", needle_name.c_str(),
               move_group_->getName().c_str());
      return able_to_pick;
    }
  }

  objsCheckMap_ = planning_scene_interface_ -> getObjects();

  if (!updateNeedleModel(needle_name, hasObject(needle_name, objsCheckMap_)))
  {
    return able_to_pick;
  }

  // turnOnStickyFinger();
  switch (mode)
  {
    case NeedlePickMode::RANDOM:
    {
      ROS_INFO("Robot is going to grasp needle in RANDOM mode");
      able_to_pick = randomPickNeedle(needle_name,
                                      needle_pose,
                                      possible_grasps_msgs_,
                                      plan_only,
                                      false);

      if (able_to_pick && !plan_only)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle in RANDOM mode");
        return able_to_pick;
      }
    }

    case NeedlePickMode::DEFINED:
    {
      ROS_INFO("Robot is going to grasp needle in DEFINED mode");
      able_to_pick = definedPickNeedle(needle_name,
                                       needle_pose,
                                       defined_grasp_msg_,
                                       plan_only);

      if (able_to_pick && !plan_only)
      {
        selected_grasp_info_ = defined_grasp_info_;
        ROS_INFO("Grasp Planning succeeded at %d th grasp pose", (int)selected_grasp_info_.graspParamInfo.grasp_id);

        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle in DEFINED mode");
        return able_to_pick;
      }
    }

    case NeedlePickMode::OPTIMAL:
    {
      ROS_INFO("Robot is going to grasp needle in OPTIMAL mode");
      able_to_pick = optimalPickNeedle(needle_name,
                                       needle_pose,
                                       possible_grasps_msgs_,
                                       plan_only);

      if (able_to_pick && !plan_only)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle in OPTIMAL mode");
        return able_to_pick;
      }
    }

    case NeedlePickMode::FINDGOOD:
    {
      ROS_INFO("Robot is going to grasp needle in FINDGOOD mode");
      able_to_pick = tryPickNeedle(needle_name,
                                   needle_pose,
                                   possible_grasps_msgs_,
                                   plan_only);

      if (able_to_pick && !plan_only)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle in FINDGOOD mode");
        return able_to_pick;
      }
    }
  }
}

bool DavinciSimpleNeedleGrasper::placeNeedle
(
const std::string& needle_name,
const geometry_msgs::Pose& needle_goal_pose,
bool plan_only
)
{
  std::vector<moveit_msgs::PlaceLocation> empty_place_locations;
  bool able_to_place = planPlacePath(needle_name, needle_goal_pose, empty_place_locations);
  if (able_to_place && !plan_only)
  {
    able_to_place = executePlaceTraj();
    ROS_INFO("Object has been placed");
    return able_to_place;
  }
  else
  {
    ROS_INFO("Can not place needle");
    return able_to_place;
  }
}

void DavinciSimpleNeedleGrasper::changePlanningGroup
(
const std::string& planning_group
)
{
  if(planning_group_name_ != planning_group)
  {
    nh_priv_.param("planning_group_name", planning_group_name_, planning_group);  // set planning_group_name_ == planning_group
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
    move_group_->setPlanningTime(20.0);

    ee_group_name_ = move_group_->getEndEffector();

    if (!needleGraspData_.loadRobotGraspData(nh_priv_, ee_group_name_))
    {
      ros::shutdown();
    }
  }
}

bool DavinciSimpleNeedleGrasper::releaseNeedle
(
const std::string &needle_name,
bool plan_only
)
{
  bool able_to_place = false;

  moveit::planning_interface::MoveGroupInterface eef_group(ee_group_name_);
  std::map< std::string, double > variable_values;
  variable_values.insert(std::pair<std::string, double>(needleGraspData_.grasp_posture_.joint_names.front(),
                                                        needleGraspData_.pre_grasp_posture_.points.front().positions.front()));

  eef_group.setJointValueTarget(variable_values);

  ROS_INFO("Object is able to be released");
  moveit_msgs::MoveItErrorCodes error_code = eef_group.move();
  if (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    move_group_->detachObject(needle_name);
    removeNeedleFromPlanningScene(needle_name);
    return true;
  }
  else
  {
    ROS_INFO("Can not place needle and the error code is %d", error_code.val);
    return false;
  }
}

const GraspInfo& DavinciSimpleNeedleGrasper::getDefinedGraspInfo
(
) const
{
  return defined_grasp_info_;
}

const GraspInfo& DavinciSimpleNeedleGrasper::getSelectedGraspInfo
(
) const
{
  return selected_grasp_info_;
}

geometry_msgs::PoseStamped DavinciSimpleNeedleGrasper::getNeedlePose
(
)
{
  fresh_needle_pose_ = false;
  updateNeedlePose();
  return needle_pose_;
}

geometry_msgs::Transform DavinciSimpleNeedleGrasper::getGraspTransform
(
)
{
  updateNeedlePose();

  tf::Pose needle_pose;
  tf::poseMsgToTF(needle_pose_.pose, needle_pose);
  tf::Pose tip_pose;
  tf::poseMsgToTF(move_group_->getCurrentPose(needleGraspData_.ee_tool_tip_link_).pose, tip_pose);

  tf::Transform grasp_tf = tip_pose.inverse() * needle_pose;
  geometry_msgs::Transform grasp_tf_msg;
  tf::transformTFToMsg(grasp_tf, grasp_tf_msg);
  return grasp_tf_msg;
}

bool DavinciSimpleNeedleGrasper::addNeedleToPlanningScene
(
const std::string& needle_name,
const geometry_msgs::PoseStamped& needle_origin
)
{
  bool is_added = false;

  if (!generateNeedleCollisionModel(needle_origin, needle_name))
  {
    ROS_ERROR("DavinciSimpleNeedleGrasper: Can't add needle to planning scene "
              "because no needle collision model generated");
    return is_added;
  }

  needle_collision_model_.operation = needle_collision_model_.ADD;
  ROS_INFO("Adding the needle into the planning scene world.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.clear(); // make sure there is no other object will be added
  planning_scene.world.collision_objects.push_back(needle_collision_model_);
  planning_scene.is_diff = true;
  moveit_planning_scene_diff_publisher_.publish(planning_scene);

  is_added = true;
  return is_added;
}

bool DavinciSimpleNeedleGrasper::removeNeedleFromPlanningScene
(
const std::string &needle_name
)
{
  bool is_removed = false;

  if(!davinciNeedleHandler_ -> detachObjectFromRobot(needle_name))
  {
    ROS_INFO("Trying to detach needle from robot, but there is no needle %s attached to robot", needle_name.c_str());
  }

  if (moveit_planning_scene_diff_publisher_.getNumSubscribers() < 1)
  {
    ROS_WARN("DavinciSimpleNeedleGrasper: No node subscribed to planning scene "
             "publisher.");
    return is_removed;
  }

  objsCheckMap_ = planning_scene_interface_ -> getObjects();
  ObjsCheckMap::iterator objs_it = objsCheckMap_ .find(needle_name);

  if (objs_it  == objsCheckMap_.end())
  {
    ROS_INFO("Needle %s is not in the planning scene world", needle_name.c_str());
    is_removed = true;
    return is_removed;
  }

  needle_collision_model_.operation = needle_collision_model_.REMOVE;

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.is_diff = true; // always set to be true either in attach or detach case
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(needle_collision_model_
  );
  moveit_planning_scene_diff_publisher_.publish(planning_scene);

  is_removed = true;
  return is_removed;
}

bool DavinciSimpleNeedleGrasper::randomPickNeedle
(
const std::string& needle_name,
const geometry_msgs::PoseStamped& needle_pose,
std::vector<moveit_msgs::Grasp>& possible_grasps_msgs,
bool plan_only,
bool sort,
const NeedlePickMode pickMode
)
{
  possible_grasps_msgs.clear();
  // Pick grasp
  simpleNeedleGraspGenerator_->generateSimpleNeedleGrasps(needle_pose, needleGraspData_, possible_grasps_ ,possible_grasps_msgs, sort);
  needleGraspData_.print();

  return planGraspPath(needle_name, possible_grasps_msgs, pickMode);
}

bool DavinciSimpleNeedleGrasper::definedPickNeedle
(
const std::string& needle_name,
const geometry_msgs::PoseStamped& needle_pose,
moveit_msgs::Grasp& defined_grasp_msgs,
bool plan_only
)
{
  // Pick grasp
  simpleNeedleGraspGenerator_->generateDefinedSimpleNeedleGrasp(needle_pose, needleGraspData_, defined_grasp_msgs,
                                                                defined_grasp_info_);
  needleGraspData_.print();
  std::vector<moveit_msgs::Grasp> defined_grasp(1, defined_grasp_msgs);
  return planGraspPath(needle_name, defined_grasp, NeedlePickMode::DEFINED);
}

bool DavinciSimpleNeedleGrasper::optimalPickNeedle
(
const std::string& needle_name,
const geometry_msgs::PoseStamped& needle_pose,
std::vector<moveit_msgs::Grasp>& possible_grasps_msgs,
bool plan_only
)
{
  return randomPickNeedle(needle_name, needle_pose, possible_grasps_msgs, plan_only, true, NeedlePickMode::OPTIMAL);
}

bool DavinciSimpleNeedleGrasper::tryPickNeedle
(
const std::string& needle_name,
const geometry_msgs::PoseStamped& needle_pose,
std::vector<moveit_msgs::Grasp>& possible_grasps_msgs,
bool plan_only
)
{
  return randomPickNeedle(needle_name, needle_pose, possible_grasps_msgs, plan_only, false, NeedlePickMode::FINDGOOD);
}

bool DavinciSimpleNeedleGrasper::selectPickNeedle
(
const std::string& needle_name,
GraspInfo& selected_grasp_info,
bool plan_only
)
{
  bool able_to_pick = false;

  if (hasObject(needle_name, planning_scene_interface_ -> getAttachedObjects()))
  {
    if(!move_group_->detachObject(needle_name))
    {
      ROS_INFO("Needle: %s is already grasped and can not be detached from group %s", needle_name.c_str(),
               move_group_->getName().c_str());
      return able_to_pick;
    }
  }

  objsCheckMap_ = planning_scene_interface_ -> getObjects();

  if (!updateNeedleModel(needle_name, hasObject(needle_name, objsCheckMap_)))
  {
    return able_to_pick;
  }
  // Pick grasp
  moveit_msgs::Grasp grasp_msg;
  simpleNeedleGraspGenerator_->generateDefinedSimpleNeedleGrasp(needle_pose_, needleGraspData_, grasp_msg,
                                                                selected_grasp_info, true);
  needleGraspData_.print();
  std::vector<moveit_msgs::Grasp> defined_grasp(1, grasp_msg);
  able_to_pick = planGraspPath(needle_name, defined_grasp, NeedlePickMode::SELECT);

  if (able_to_pick && !plan_only)
  {
    selected_grasp_info_ = selected_grasp_info;
    ROS_INFO("Grasp Planning succeeded at %d th grasp pose", (int)selected_grasp_info_.graspParamInfo.grasp_id);

    able_to_pick = executePickupTraj();
    m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
    ROS_INFO("Object has been picked up");
    return able_to_pick;
  }

  ROS_INFO("Can not pick needle in SELECT mode");
  return able_to_pick;
}

bool DavinciSimpleNeedleGrasper::planGraspPath
(
const std::string& needle_name,
std::vector<moveit_msgs::Grasp>& possible_grasps_msgs,
const NeedlePickMode pickMode
)
{
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getRobotModel()));
  pPickPlace_.reset(new pick_place::PickPlace(planning_pipeline));

  std::vector<std::string> allowed_touch_objects;
  allowed_touch_objects.push_back(needle_name);
  planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor::LockedPlanningSceneRO lockedPS(planning_scene_monitor_);

  switch (pickMode)
  {
    case NeedlePickMode::OPTIMAL :
      // continue to DEFINED
    case NeedlePickMode::FINDGOOD :
      // continue to DEFINED
    case NeedlePickMode::SELECT :
      // conttnue to DEFINED
    case NeedlePickMode::DEFINED :
      for (std::size_t i = 0; i < possible_grasps_msgs.size(); ++i)
      {
        possible_grasps_msgs[i].allowed_touch_objects = allowed_touch_objects;
        moveit_msgs::PickupGoal pickupGoal;
        constructPickupGoal(needle_name, std::vector<moveit_msgs::Grasp>(1, possible_grasps_msgs[i]), pickupGoal);

        pick_place::PickPlanPtr pPickPlan = pPickPlace_->planPick(lockedPS, pickupGoal);
        if (!pPickPlan || pPickPlan->getSuccessfulManipulationPlans().empty())
        {
          continue;
        }

        pick_place::ManipulationPlanPtr pGoodPlan = pPickPlan->getSuccessfulManipulationPlans()[0];
        if (!pGoodPlan)
        {
          continue;
        }

        pPickPlace_->displayComputedMotionPlans(true);
        pPickPlace_->visualizePlan(pGoodPlan);
        std::vector<plan_execution::ExecutableTrajectory> graspPath = pGoodPlan->trajectories_;
        convertPathToTrajectory(graspPath, graspTrajectories_);
        if (possible_grasps_msgs.size() == 1 && (pickMode == NeedlePickMode::DEFINED || pickMode == NeedlePickMode::SELECT))
        {
          return true;
        }

        selected_grasp_info_ = possible_grasps_[i];
        ROS_INFO("Grasp Planning succeeded at %d th grasp pose", (int)selected_grasp_info_.graspParamInfo.grasp_id);
        ROS_INFO("Prepared to call debugger");
        ros::Duration(1.0).sleep();
        return true;
      }
      return false;
    case NeedlePickMode::RANDOM :
      for (std::size_t i = 0; i < possible_grasps_msgs.size(); ++i)
      {
        possible_grasps_msgs[i].allowed_touch_objects = allowed_touch_objects;
      }
      moveit_msgs::PickupGoal pickupGoal;
      constructPickupGoal(needle_name, possible_grasps_msgs, pickupGoal);
      pick_place::PickPlanPtr pPickPlan = pPickPlace_->planPick(lockedPS, pickupGoal);
      if (!pPickPlan || pPickPlan->getSuccessfulManipulationPlans().empty())
      {
        return false;
      }

      pick_place::ManipulationPlanPtr pGoodPlan = pPickPlan->getSuccessfulManipulationPlans()[0];
      if(!pGoodPlan)
      {
        return false;
      }

      pPickPlace_->displayComputedMotionPlans(true);
      pPickPlace_->visualizePlan(pGoodPlan);
      std::vector<plan_execution::ExecutableTrajectory> graspPath = pGoodPlan->trajectories_;
      convertPathToTrajectory(graspPath, graspTrajectories_);
      selected_grasp_info_ = possible_grasps_[pPickPlan->getSuccessfulManipulationPlans()[0]->id_];

      ROS_INFO("Grasp Planning succeeded at %d th grasp pose", selected_grasp_info_.graspParamInfo.grasp_id);
      ROS_INFO("Prepared to call debugger");
      ros::Duration(1.0).sleep();
      return true;
  }
}

void DavinciSimpleNeedleGrasper::convertPathToTrajectory
(
const std::vector<plan_execution::ExecutableTrajectory>& planedPath,
std::vector<trajectory_msgs::JointTrajectory>& executableTrajectory
)
{
  executableTrajectory.clear();

  for (std::size_t i = 0; i < planedPath.size(); ++i)
  {
    moveit_msgs::RobotTrajectory ithTraj;
    planedPath[i].trajectory_->getRobotTrajectoryMsg(ithTraj);
    executableTrajectory.push_back(ithTraj.joint_trajectory);
    double t = 0;
    double time = 0.05;
    for (std::size_t j = 0; j < executableTrajectory[i].points.size(); ++j)
    {
      t += time;
      executableTrajectory[i].points[j].velocities.clear();
      executableTrajectory[i].points[j].accelerations.clear();
      executableTrajectory[i].points[j].time_from_start = ros::Duration(t);
    }
  }
}

bool DavinciSimpleNeedleGrasper::planPlacePath
(
const std::string& needle_name,
const geometry_msgs::Pose& needle_goal_pose,
std::vector<moveit_msgs::PlaceLocation>& place_location_msgs
)
{
  ROS_WARN_STREAM_NAMED("place","Placing '"<< needle_name << "'");
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(planning_scene_monitor_->getRobotModel()));
  pPickPlace_.reset(new pick_place::PickPlace(planning_pipeline));

  std::vector<std::string> allowed_touch_objects;
  allowed_touch_objects.push_back(needle_name);
  planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor::LockedPlanningSceneRO lockedPS(planning_scene_monitor_);

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = needleGraspData_.base_link_;
  pose_stamped.header.stamp = ros::Time::now();

  moveit_msgs::PlaceGoal placeGoal;
  if(place_location_msgs.empty())
  {
    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2*M_PI; angle += M_PI/10)
    {
      pose_stamped.pose.position = needle_goal_pose.position;

      // Orientation
      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
      pose_stamped.pose.orientation.x = quat.x();
      pose_stamped.pose.orientation.y = quat.y();
      pose_stamped.pose.orientation.z = quat.z();
      pose_stamped.pose.orientation.w = quat.w();

      // Create new place location
      moveit_msgs::PlaceLocation place_loc;

      place_loc.place_pose = pose_stamped;

      // Approach
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = needleGraspData_.approach_desired_dist_; // The distance the origin of a robot link needs to travel
      pre_place_approach.min_distance = needleGraspData_.approach_min_dist_; // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = needleGraspData_.ee_tool_tip_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = 1; // Approach direction (positive z axis)
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      post_place_retreat.desired_distance = needleGraspData_.retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      post_place_retreat.min_distance = needleGraspData_.retreat_min_dist_; // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = needleGraspData_.ee_tool_tip_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = -1; // Retreat direction (negative z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = needleGraspData_.pre_grasp_posture_;

      place_loc.allowed_touch_objects = allowed_touch_objects;
      place_location_msgs.push_back(place_loc);
    }

    constructPlaceGoal(needle_name, place_location_msgs, placeGoal);
    pick_place::PlacePlanPtr pPlacePlan = pPickPlace_->planPlace(lockedPS, placeGoal);
    if (!pPlacePlan || pPlacePlan->getSuccessfulManipulationPlans().empty())
    {
      return false;
    }

    pick_place::ManipulationPlanPtr pGoodPlan = pPlacePlan->getSuccessfulManipulationPlans()[0];
    if(!pGoodPlan)
    {
      return false;
    }

    pPickPlace_->displayComputedMotionPlans(true);
    pPickPlace_->visualizePlan(pGoodPlan);
    std::vector<plan_execution::ExecutableTrajectory> placePath = pGoodPlan->trajectories_;
    convertPathToTrajectory(placePath, placeTrajectories_);

    ROS_INFO("Placeing Planning succeeded");
    ROS_INFO("Prepared to call debugger");
    ros::Duration(10.0).sleep();
    return true;
  }
  else
  {
    constructPlaceGoal(needle_name, place_location_msgs, placeGoal);
    pick_place::PlacePlanPtr pPlacePlan = pPickPlace_->planPlace(lockedPS, placeGoal);
    if (!pPlacePlan || pPlacePlan->getSuccessfulManipulationPlans().empty())
    {
      return false;
    }

    pick_place::ManipulationPlanPtr pGoodPlan = pPlacePlan->getSuccessfulManipulationPlans()[0];
    if(!pGoodPlan)
    {
      return false;
    }

    pPickPlace_->displayComputedMotionPlans(true);
    pPickPlace_->visualizePlan(pGoodPlan);
    std::vector<plan_execution::ExecutableTrajectory> placePath = pGoodPlan->trajectories_;
    convertPathToTrajectory(placePath, placeTrajectories_);

    ROS_INFO("Placeing Planning succeeded");
    ROS_INFO("Prepared to call debugger");
    ros::Duration(10.0).sleep();
    return true;
  }
}

bool DavinciSimpleNeedleGrasper::generateNeedleCollisionModel
(
const geometry_msgs::PoseStamped& needle_origin,
const std::string& needle_name
)
{

  bool able_to_generate = false;

  Eigen::Vector3d scale_vec(1.0, 1.0, 1.0);
  shapes::Mesh* m =
      shapes::createMeshFromResource(needleGraspData_.needle_mesh_model_path_,
                                     scale_vec);
  ROS_INFO("needle mesh loaded");
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  if (!shapes::constructMsgFromShape(m, mesh_msg))
  {
    ROS_ERROR("DavinciSimpleNeedleGrasper is unable to generate needle "
              "collision model");
    return able_to_generate;
  }
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  needle_collision_model_.meshes.resize(1);
  needle_collision_model_.mesh_poses.resize(1);
  needle_collision_model_.meshes[0] = mesh;
  needle_collision_model_.header.frame_id = needle_origin.header.frame_id;
  needle_collision_model_.id = needle_name;

  needle_collision_model_.mesh_poses[0].position.x =
      needle_origin.pose.position.x;

  needle_collision_model_.mesh_poses[0].position.y =
      needle_origin.pose.position.y;

  needle_collision_model_.mesh_poses[0].position.z =
      needle_origin.pose.position.z;

  needle_collision_model_.mesh_poses[0].orientation.w =
      needle_origin.pose.orientation.w;

  needle_collision_model_.mesh_poses[0].orientation.x =
      needle_origin.pose.orientation.x;

  needle_collision_model_.mesh_poses[0].orientation.y =
      needle_origin.pose.orientation.y;

  needle_collision_model_.mesh_poses[0].orientation.z =
      needle_origin.pose.orientation.z;

  //  needle_mesh.operation = needle_mesh.ADD;  // Add object to the planning
  //  scene, object will be replaced if it existed
  able_to_generate = true;
  return able_to_generate;
}

void DavinciSimpleNeedleGrasper::needlePoseCallBack
(
const geometry_msgs::PoseStamped& needle_pose
)
{
  fresh_needle_pose_ = true;
  needle_pose_ = needle_pose;
}

void DavinciSimpleNeedleGrasper::updateNeedlePose
(
)
{
  while (!fresh_needle_pose_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  fresh_needle_pose_ = false;  // reset to false
  return;
}

bool DavinciSimpleNeedleGrasper::hasObject
(
const std::string &name,
const AttachedObjsCheckMap &attachedObjsCheckMap
)
{
  AttachedObjsCheckMap::const_iterator attached_it = attachedObjsCheckMap.find(name);

  if (attached_it != attachedObjsCheckMap.end())
  {
    return true;
  }
  return false;
}

bool DavinciSimpleNeedleGrasper::hasObject
(
const std::string &name,
const ObjsCheckMap &objsCheckMap
)
{
  ObjsCheckMap::const_iterator objs_it = objsCheckMap.find(name);

  if (objs_it  != objsCheckMap_.end())
  {
    return true;
  }
  return false;
}

bool DavinciSimpleNeedleGrasper::updateNeedleModel
(
const std::string &needle_name,
bool in_planning_scene
)
{
  if (in_planning_scene && !removeNeedleFromPlanningScene(needle_name))
  {
    return false;
  }

  updateNeedlePose();
  if(!addNeedleToPlanningScene(needle_name, needle_pose_))
  {
    return false;
  }

  return true;
}

bool DavinciSimpleNeedleGrasper::turnOnStickyFinger
(
)
{
  ros::ServiceClient stickyFingerClient;
  (planning_group_name_ == "psm_one") ?
  stickyFingerClient = nh_.serviceClient<std_srvs::SetBool>("/sticky_finger/PSM1_tool_wrist_sca_ee_link_1") :
  stickyFingerClient = nh_.serviceClient<std_srvs::SetBool>("/sticky_finger/PSM2_tool_wrist_sca_ee_link_1");

  std_srvs::SetBool graspCommand;
  graspCommand.request.data = true;
  stickyFingerClient.call(graspCommand);
}

bool DavinciSimpleNeedleGrasper::executePickupTraj
(
)
{
  m_pSupportArmGroup.reset(new psm_interface(planning_group_name_, nh_));
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  for (std::size_t i = 0; i < graspTrajectories_.size(); ++i)
  {
    if (i == 3)
    {
      turnOnStickyFinger();
    }
    if (!m_pSupportArmGroup->execute_trajectory(graspTrajectories_[i]))
    {
      return false;
    }
    if (i == 2)
    {
      char answer = 'n';
      // std::cout << "Do you want to move tool tip down (y/n)? ";
      // std::cin >> answer;
      double okToMove = (answer == 'y') ? true : false;
      while (okToMove)
      {
        double moveDist = 0.0;
        double time = 0.0;
        std::cout << "How much further and time? ";
        std::cin >> moveDist >> time;
        if (!compensationLinearMove(moveDist, time))
        {
          return false;
        }
        std::cout << "Do you want to move tool tip down further (y/n)? ";
        std::cin >> answer;
        okToMove = (answer == 'y') ? true : false;
      }
    }
  }
  move_group_->attachObject(needle_name_);
  planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  planning_scene_monitor_->updateFrameTransforms();

  uv_msgs::pf_grasp pf_grasp_srv;
  pf_grasp_srv.request.psm = m_pSupportArmGroup->get_psm();
  // geometry_msgs::Pose grasp_pose = possible_grasps_msgs_[selected_grasp_info_.graspParamInfo.grasp_id].grasp_pose.pose;

  geometry_msgs::Pose grasp_pose;
  tf::poseEigenToMsg(selected_grasp_info_.grasp_pose, grasp_pose);
  pf_grasp_srv.request.grasp_transform.rotation.w = grasp_pose.orientation.w;
  pf_grasp_srv.request.grasp_transform.rotation.x = grasp_pose.orientation.x;
  pf_grasp_srv.request.grasp_transform.rotation.y = grasp_pose.orientation.y;
  pf_grasp_srv.request.grasp_transform.rotation.z = grasp_pose.orientation.z;
  pf_grasp_srv.request.grasp_transform.translation.x = grasp_pose.position.x;
  pf_grasp_srv.request.grasp_transform.translation.y = grasp_pose.position.y;
  pf_grasp_srv.request.grasp_transform.translation.z = grasp_pose.position.z;

  if(!pf_grasp_client_.call(pf_grasp_srv))
  {
    ROS_WARN("Failed to call pf_grasp service.");
    ros::spinOnce();
  }

  return true;
  // return compensationLinearMove(-0.015, 5.0);
}

bool DavinciSimpleNeedleGrasper::executePlaceTraj
(
)
{
  m_pSupportArmGroup.reset(new psm_interface(planning_group_name_, nh_));
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  for (std::size_t i = 0; i < placeTrajectories_.size(); ++i)
  {
    if (!m_pSupportArmGroup->execute_trajectory(placeTrajectories_[i]))
    {
      return false;
    }
    if (i == 2)
    {
      char answer = 'n';
      std::cout << "Do you want to move tool tip down (y/n)? ";
      std::cin >> answer;
      bool okToMove = (answer == 'y') ? true : false;
      while (okToMove)
      {
        double moveDist = 0.0;
        double time = 0.0;
        std::cout << "How much further and time? ";
        std::cin >> moveDist >> time;
        if (!compensationLinearMove(moveDist, time))
        {
          return false;
        }
        std::cout << "Do you want to move tool tip down further (y/n)? ";
        std::cin >> answer;
        okToMove = (answer == 'y') ? true : false;
      }
    }
  }
  move_group_->detachObject(needle_name_);

  uv_msgs::pf_grasp pf_grasp_srv;
  pf_grasp_srv.request.psm = 0;
  geometry_msgs::Pose grasp_pose = possible_grasps_msgs_[selected_grasp_info_.graspParamInfo.grasp_id].grasp_pose.pose;
  pf_grasp_srv.request.grasp_transform.rotation.w = 1.0;
  pf_grasp_srv.request.grasp_transform.rotation.x = 0;
  pf_grasp_srv.request.grasp_transform.rotation.y = 0;
  pf_grasp_srv.request.grasp_transform.rotation.z = 0;
  pf_grasp_srv.request.grasp_transform.translation.x = 0;
  pf_grasp_srv.request.grasp_transform.translation.y = 0;
  pf_grasp_srv.request.grasp_transform.translation.z = 0;

  // if(!pf_grasp_client_.call(pf_grasp_srv))
  // {
  //   ROS_WARN("Failed to call pf_grasp service.");
  //   ros::spinOnce();
  // }

  // return compensationLinearMove(-0.015, 5.0);
  return true;
}

bool DavinciSimpleNeedleGrasper::compensationLinearMove
(
double z_dist,
double time
)
{
  const robot_state::RobotStatePtr pRobotState(new robot_state::RobotState(move_group_->getRobotModel()));
  pRobotState->setToDefaultValues();
  const moveit::core::LinkModel* pTipLink = pRobotState->getJointModelGroup(planning_group_name_)->getOnlyOneEndEffectorTip();
  std::vector<double> jointPosition;
  m_pSupportArmGroup->get_fresh_position(jointPosition);
  pRobotState->setJointGroupPositions(planning_group_name_, jointPosition);
  pRobotState->update();
  Eigen::Affine3d toolTipPose = pRobotState->getGlobalLinkTransform(pTipLink);

  toolTipPose.translation().z() += z_dist;
  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pRobotState->computeCartesianPath(pRobotState->getJointModelGroup(planning_group_name_),
                                                                traj,
                                                                pTipLink,
                                                                toolTipPose,
                                                                true,
                                                                0.001,
                                                                0.0);

  if (!((foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  std::vector<std::vector<double> > jntTrajectory(traj.size());

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    traj[i]->copyJointGroupPositions(planning_group_name_, jntTrajectory[i]);
  }

  double jaw = 0.0; m_pSupportArmGroup->get_gripper_fresh_position(jaw);

  if (!m_pSupportArmGroup->execute_trajectory_t(jntTrajectory, jaw, time))
  {
    return false;
  }

  return true;
}

void DavinciSimpleNeedleGrasper::constructPickupGoal
(
const std::string& object,
const std::vector<moveit_msgs::Grasp>& possible_grasps_msgs,
moveit_msgs::PickupGoal& pickupGoal
)
{
  pickupGoal.target_name = object;
  pickupGoal.group_name = planning_group_name_;
  pickupGoal.end_effector = ee_group_name_;
  pickupGoal.allowed_planning_time = move_group_->getPlanningTime();
  pickupGoal.planner_id = move_group_->getPlannerId();
  pickupGoal.allow_gripper_support_collision = true;

  pickupGoal.possible_grasps = possible_grasps_msgs;
  pickupGoal.planning_options.plan_only = true;
  pickupGoal.planning_options.look_around = false;
  pickupGoal.planning_options.replan = false;
  pickupGoal.planning_options.replan_delay = 2.0;
  pickupGoal.planning_options.planning_scene_diff.is_diff = true;
  pickupGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;
}

void DavinciSimpleNeedleGrasper::constructPlaceGoal
(
const std::string& object,
const std::vector<moveit_msgs::PlaceLocation>& place_location_msgs,
moveit_msgs::PlaceGoal& placeGoal
)
{
  placeGoal.attached_object_name = object;
  placeGoal.group_name = planning_group_name_;
  placeGoal.allowed_planning_time = move_group_->getPlanningTime();
  placeGoal.planner_id = move_group_->getPlannerId();
  placeGoal.allow_gripper_support_collision = true;

  placeGoal.place_locations = place_location_msgs;
  placeGoal.planning_options.plan_only = true;
  placeGoal.planning_options.look_around = false;
  placeGoal.planning_options.replan = false;
  placeGoal.planning_options.replan_delay = 2.0;
  placeGoal.planning_options.planning_scene_diff.is_diff = true;
  placeGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;
}

}  // namespace
