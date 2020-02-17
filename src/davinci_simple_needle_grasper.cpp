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

using namespace davinci_moveit_object_handling;

namespace cwru_davinci_grasp
{
DavinciSimpleNeedleGrasper::DavinciSimpleNeedleGrasper(
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nh_priv,
  const std::string &planning_group_name,
  const std::string &needle_name,
  const std::string &get_planning_scene_service,
  const std::string &set_planning_scene_topic,
  const std::string &updated_needle_pose_topic)
  : DavinciNeedleGrasperBase(nh_priv, planning_group_name), nh_(nh), needle_name_(needle_name)
{
  ROS_INFO_STREAM_NAMED("DavinciSimpleNeedleGrasper", "Starting Simpling Needle Grasping");

  // Re-reate MoveGroup for one of the planning groups
  move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
  move_group_->setPlanningTime(30.0);

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
    nh_.serviceClient<moveit_msgs::GetPlanningScene>(
      get_planning_scene_service);

  needle_pose_sub_ =
    nh_.subscribe(updated_needle_pose_topic, 1,
                  &DavinciSimpleNeedleGrasper::needlePoseCallBack, this);
  if(needle_pose_sub_.getNumPublishers() < 1)
  {
    ros::Duration(5.0).sleep();
  }

  pick_up_action_sub_ = nh_.subscribe("/pickup/result", 100,
                                      &DavinciSimpleNeedleGrasper::pickupActionCallBack, this);

  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface);

  ros::Duration(1.0).sleep();
}

bool DavinciSimpleNeedleGrasper::pickNeedle(const NeedlePickMode &mode, const std::string &needle_name)
{
  return pickNeedle(needle_pose_, needle_name, mode, defined_grasp_);
}

bool DavinciSimpleNeedleGrasper::pickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                            const std::string &needle_name,
                                            const NeedlePickMode &mode,
                                            GraspInfo &grasp_pose,
                                            bool has_grasp_pose,
                                            bool plan_only)
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

  turnOnStickyFinger();
  switch (mode)
  {
    case NeedlePickMode::RANDOM:
    {
      ROS_INFO("Robot is going to grasp needle with random grasping parameters");
      moveit_msgs::MoveItErrorCodes error_code = randomPickNeedle(needle_pose,
                                                                  needle_name,
                                                                  possible_grasps_msgs_,
                                                                  plan_only);
      if (error_code.val == error_code.SUCCESS)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle and the error code is %d", error_code.val);
        return able_to_pick;
      }
    }

    case NeedlePickMode::DEFINED:
    {
      ROS_INFO("Robot is going to grasp needle with user defined grasping parameters");
      moveit_msgs::MoveItErrorCodes error_code = definedPickNeedle(needle_pose,
                                                                   needle_name,
                                                                   defined_grasp_msg_,
                                                                   grasp_pose,
                                                                   has_grasp_pose,
                                                                   plan_only);
      if (error_code.val == error_code.SUCCESS)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle and the error code is %d", error_code.val);
        return able_to_pick;
      }
    }

    case NeedlePickMode::OPTIMAL:
    {
      ROS_INFO("Robot is going to grasp needle with user defined grasping parameters");
      moveit_msgs::MoveItErrorCodes error_code = optimalPickNeedle(needle_pose,
                                                                   needle_name,
                                                                   possible_grasps_msgs_,
                                                                   plan_only);
      if (error_code.val == error_code.SUCCESS)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle and the error code is %d", error_code.val);
        return able_to_pick;
      }
    }

    case NeedlePickMode::FINDGOOD:
    {
      ROS_INFO("Robot is going to grasp needle with user defined grasping parameters");
      moveit_msgs::MoveItErrorCodes error_code = tryPickNeedle(needle_pose,
                                                               needle_name,
                                                               possible_grasps_msgs_,
                                                               plan_only);
      if (error_code.val == error_code.SUCCESS)
      {
        able_to_pick = executePickupTraj();
        m_pSupportArmGroup->get_fresh_position(graspedJointPosition_);
        ROS_INFO("Object has been picked up");
        return able_to_pick;
      }
      else
      {
        ROS_INFO("Can not pick needle and the error code is %d", error_code.val);
        return able_to_pick;
      }
    }
  }
}

bool DavinciSimpleNeedleGrasper::placeNeedle(const geometry_msgs::Pose &needle_goal_pose,
                                             const std::string &needle_name)
{
  bool able_to_place = false;
  std::vector<moveit_msgs::PlaceLocation> empty_place_locations;
  moveit_msgs::MoveItErrorCodes error_code = placeNeedleHelper(needle_goal_pose, needle_name, empty_place_locations);
  if (error_code.val == error_code.SUCCESS)
  {
    ROS_INFO("Object is able to be placed");
    able_to_place = true;
    return able_to_place;
  }
  else
  {
    ROS_INFO("Can not place needle and the error code is %d", error_code.val);
    return able_to_place;
  }
}

void DavinciSimpleNeedleGrasper::changePlanningGroup(const std::string& planning_group)
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

bool DavinciSimpleNeedleGrasper::releaseNeedle(const std::string &needle_name,
                                               bool plan_only)
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

const GraspInfo& DavinciSimpleNeedleGrasper::getDefinedGraspInfo() const
{
  return defined_grasp_;
}

const GraspInfo& DavinciSimpleNeedleGrasper::getSelectedGraspInfo() const
{
  return selected_grasp_;
}

geometry_msgs::PoseStamped DavinciSimpleNeedleGrasper::getNeedlePose()
{
  fresh_needle_pose_ = false;
  updateNeedlePose();
  return needle_pose_;
}

geometry_msgs::Transform DavinciSimpleNeedleGrasper::getGraspTransform()
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

bool DavinciSimpleNeedleGrasper::addNeedleToPlanningScene(
    const geometry_msgs::PoseStamped& needle_origin,
    const std::string& needle_name)
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

bool DavinciSimpleNeedleGrasper::removeNeedleFromPlanningScene(const std::string &needle_name)
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

moveit_msgs::MoveItErrorCodes DavinciSimpleNeedleGrasper::randomPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                                                           const std::string &needle_name,
                                                                           std::vector<moveit_msgs::Grasp> &possible_grasps_msgs,
                                                                           bool plan_only, bool sort)
{
  possible_grasps_msgs.clear();
  // Pick grasp
  simpleNeedleGraspGenerator_->generateSimpleNeedleGrasps(needle_pose, needleGraspData_, possible_grasps_msgs, sort);
  needleGraspData_.print();

  // Allow blocks to be touched by end effector
  {
    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(needle_name);

    // Add this list to all grasps
    for (std::size_t i = 0; i < possible_grasps_msgs.size(); ++i)
    {
      possible_grasps_msgs[i].allowed_touch_objects = allowed_touch_objects;
    }
  }

  return move_group_->pick(needle_name, possible_grasps_msgs, plan_only);
}

moveit_msgs::MoveItErrorCodes DavinciSimpleNeedleGrasper::tryPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                                                        const std::string &needle_name,
                                                                        std::vector<moveit_msgs::Grasp> &possible_grasps_msgs,
                                                                        bool plan_only, bool sort)
{
  possible_grasps_msgs.clear();
  // Pick grasp
  simpleNeedleGraspGenerator_->generateSimpleNeedleGrasps(needle_pose, needleGraspData_, possible_grasps_msgs, sort);
  needleGraspData_.print();

  // Allow blocks to be touched by end effector
  moveit_msgs::MoveItErrorCodes errorCode;
  {
    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(needle_name);

    // Add this list to all grasps
    for (std::size_t i = 0; i < possible_grasps_msgs.size(); ++i)
    {
      possible_grasps_msgs[i].allowed_touch_objects = allowed_touch_objects;
      errorCode = move_group_->pick(needle_name, possible_grasps_msgs[i], plan_only);
      ros::spinOnce();
      if (errorCode.val == errorCode.SUCCESS)
      {
        selected_grasp_.graspParamInfo.grasp_id = i;
        return errorCode;
      }
    }
  }

  errorCode.val = errorCode.FAILURE;
  return errorCode;
}

moveit_msgs::MoveItErrorCodes DavinciSimpleNeedleGrasper::definedPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                                                            const std::string &needle_name,
                                                                            moveit_msgs::Grasp &defined_grasp_msgs,
                                                                            GraspInfo &grasp_pose,
                                                                            bool has_grasp_pose,
                                                                            bool plan_only)
{
  // Pick grasp
  simpleNeedleGraspGenerator_->generateDefinedSimpleNeedleGrasp(needle_pose, needleGraspData_, defined_grasp_msgs,
                                                                grasp_pose, has_grasp_pose);
  needleGraspData_.print();
  // Allow blocks to be touched by end effector
  {
    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(needle_name);

    // Add this list to all grasps
    defined_grasp_msgs.allowed_touch_objects = allowed_touch_objects;
  }

  return move_group_->pick(needle_name, defined_grasp_msgs, plan_only);
}

moveit_msgs::MoveItErrorCodes
DavinciSimpleNeedleGrasper::optimalPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                              const std::string &needle_name,
                                              std::vector<moveit_msgs::Grasp> &possible_grasps_msgs,
                                              bool plan_only, bool sort)
{
  return randomPickNeedle(needle_pose, needle_name, possible_grasps_msgs, plan_only, sort);
}

moveit_msgs::MoveItErrorCodes DavinciSimpleNeedleGrasper::placeNeedleHelper(const geometry_msgs::Pose &needle_goal_pose,
                                                                            const std::string &needle_name,
                                                                            std::vector<moveit_msgs::PlaceLocation>& place_locations,
                                                                            bool plan_only)
{
  ROS_WARN_STREAM_NAMED("place","Placing '"<< needle_name << "'");

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = needleGraspData_.base_link_;
  pose_stamped.header.stamp = ros::Time::now();

  if(place_locations.empty())
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

//    visual_tools_->publishCuboid(place_loc.place_pose.pose, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE, rviz_visual_tools::BLUE);

      // Approach
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = needleGraspData_.approach_desired_dist_; // The distance the origin of a robot link needs to travel
      pre_place_approach.min_distance = needleGraspData_.approach_min_dist_; // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = needleGraspData_.ee_tool_tip_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = 1; // Approach direction (negative z axis)  // TODO: document this assumption
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      post_place_retreat.desired_distance = needleGraspData_.retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      post_place_retreat.min_distance = needleGraspData_.retreat_min_dist_; // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = needleGraspData_.ee_tool_tip_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = -1; // Retreat direction (pos z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = needleGraspData_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
    }

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    move_group_->setPlannerId("RRTConnectkConfigDefault");
    return move_group_->place(needle_name, place_locations, plan_only);
  }
  else
  {
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    return move_group_->place(needle_name, place_locations, plan_only);
  }

}

bool DavinciSimpleNeedleGrasper::generateNeedleCollisionModel(
    const geometry_msgs::PoseStamped& needle_origin,
    const std::string& needle_name)
{

  bool able_to_generate = false;

  Eigen::Vector3d scale_vec(0.0254, 0.0254, 0.0254);
  shapes::Mesh* m =
      shapes::createMeshFromResource("package://sim_gazebo/"
                                     "props/needle_r/mesh/needle_r4.dae",
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

void DavinciSimpleNeedleGrasper::needlePoseCallBack(const geometry_msgs::PoseStamped& needle_pose)
{
  fresh_needle_pose_ = true;
  needle_pose_ = needle_pose;
}

void DavinciSimpleNeedleGrasper::pickupActionCallBack(const moveit_msgs::PickupActionResult& pickupResult)
{
  pickupTrajectories_.clear();
  for (std::size_t i = 0; i < pickupResult.result.trajectory_stages.size(); ++i)
  {
    pickupTrajectories_.push_back(pickupResult.result.trajectory_stages[i].joint_trajectory);
    if (i == 0 || i == 2)
    {
      double t = 0;
      double time = 0.2;
      for (std::size_t j = 0; j < pickupTrajectories_[i].points.size(); ++j)
      {
        t += time;
        pickupTrajectories_[i].points[j].velocities.clear();
        pickupTrajectories_[i].points[j].accelerations.clear();
        pickupTrajectories_[i].points[j].time_from_start = ros::Duration(t);
      }
    }
  }

  fresh_pickup_traj_ = true;
}

void DavinciSimpleNeedleGrasper::updateNeedlePose()
{
  while (!fresh_needle_pose_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  fresh_needle_pose_ = false;  // reset to false
  return;
}

void DavinciSimpleNeedleGrasper::updatePickupTraj()
{
  while (!fresh_pickup_traj_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  fresh_pickup_traj_ = false;  // reset to false
  return;
}

bool DavinciSimpleNeedleGrasper::hasObject(const std::string &name,
                                           const AttachedObjsCheckMap &attachedObjsCheckMap)
{
  AttachedObjsCheckMap::const_iterator attached_it = attachedObjsCheckMap.find(name);

  if (attached_it != attachedObjsCheckMap.end())
  {
    return true;
  }
  return false;
}

bool DavinciSimpleNeedleGrasper::hasObject(const std::string &name,
                                           const ObjsCheckMap &objsCheckMap)
{
  ObjsCheckMap::const_iterator objs_it = objsCheckMap.find(name);

  if (objs_it  != objsCheckMap_.end())
  {
    return true;
  }
  return false;
}

bool DavinciSimpleNeedleGrasper::updateNeedleModel(const std::string &needle_name,
                                                   bool in_planning_scene)
{
  if (in_planning_scene && !removeNeedleFromPlanningScene(needle_name))
  {
    return false;
  }

  updateNeedlePose();
  if(!addNeedleToPlanningScene(needle_pose_, needle_name))
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
  updatePickupTraj();
  m_pSupportArmGroup.reset(new psm_interface(planning_group_name_, nh_));
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  for (std::size_t i = 0; i < pickupTrajectories_.size(); ++i)
  {
    if (!m_pSupportArmGroup->execute_trajectory(pickupTrajectories_[i]))
    {
      return false;
    }
  }
  move_group_->attachObject(needle_name_);

  const robot_state::RobotStatePtr pRobotState(new robot_state::RobotState(move_group_->getRobotModel()));
  pRobotState->setToDefaultValues();
  const moveit::core::LinkModel* pTipLink = pRobotState->getJointModelGroup(planning_group_name_)->getOnlyOneEndEffectorTip();
  std::vector<double> jointPosition;
  m_pSupportArmGroup->get_fresh_position(jointPosition);
  pRobotState->setJointGroupPositions(planning_group_name_, jointPosition);
  pRobotState->update();
  Eigen::Affine3d toolTipPose = pRobotState->getGlobalLinkTransform(pTipLink);

  toolTipPose.translation().z() += -0.01;
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

  if (!m_pSupportArmGroup->execute_trajectory(jntTrajectory, jaw, 5.0))
  {
    return false;
  }

  return true;
}

}  // namespace
