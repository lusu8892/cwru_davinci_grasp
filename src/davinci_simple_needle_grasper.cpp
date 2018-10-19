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

using namespace davinci_moveit_object_handling;

namespace cwru_davinci_grasp
{

DavinciSimpleNeedleGrasper::DavinciSimpleNeedleGrasper(
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nh_priv,
  const std::string &needle_name,
  const std::string &planning_group_name,
  const std::string &get_planning_scene_service,
  const std::string &set_planning_scene_topic,
  const std::string &updated_needle_pose_topic)
  : nh_(nh), nh_priv_(nh_priv), needle_name_(needle_name)
{
  ROS_INFO_STREAM_NAMED("DavinciSimpleNeedleGrasper", "Starting Simpling Needle Grasping");

  // Get arm info from param server
//  nh_priv_.param("ee_group_name", ee_group_name_, std::string("psm_one_gripper"));
  nh_priv_.param("planning_group_name", planning_group_name_, planning_group_name);

  ROS_INFO_STREAM_NAMED("moveit_blocks", "End Effector: " << ee_group_name_);
  ROS_INFO_STREAM_NAMED("moveit_blocks", "Planning Group: " << planning_group_name_);

  // Re-reate MoveGroup for one of the planning groups
  move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
  move_group_->setPlanningTime(30.0);

  ee_group_name_ = move_group_->getEndEffector();

  // Load grasp generator
  davinciNeedleHandler_.reset(
    new davinci_moveit_object_handling::DavinciMoveitGraspedObjectHandler(nh,
                                                                          (new moveit::planning_interface::MoveGroupInterface(
                                                                            ee_group_name_))->getLinkNames(),
                                                                          get_planning_scene_service,
                                                                          set_planning_scene_topic));
  if (!needleGraspData_.loadRobotGraspData(nh_priv_, ee_group_name_))
  {
    ros::shutdown();
  }

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

  planning_scene_.reset(new moveit::planning_interface::PlanningSceneInterface);

  // Let everything load
  ros::Duration(1.0).sleep();

}

DavinciSimpleNeedleGrasper::~DavinciSimpleNeedleGrasper()
{
  // blank
  convenience_ros_functions::ROSFunctions::destroySingleton();
}

bool DavinciSimpleNeedleGrasper::pickNeedle(const std::string &needle_name, const NeedlePickMode &mode)
{
  return pickNeedle(needle_pose_, needle_name, mode);
}

bool DavinciSimpleNeedleGrasper::pickNeedle(
  const geometry_msgs::PoseStamped &needle_pose, const std::string &needle_name, const NeedlePickMode &mode)
{
  bool able_to_pick = false;

  if (hasObject(needle_name, planning_scene_ -> getAttachedObjects()))
  {
    if(!move_group_->detachObject(needle_name))
    {
      ROS_INFO("Needle: %s is already grasped and can not be detached from group %s", needle_name.c_str(),
               move_group_->getName().c_str());
      return able_to_pick;
    }
  }

  objsCheckMap_ = planning_scene_ -> getObjects();

  if (hasObject(needle_name, objsCheckMap_))
  {
    updateNeedleModel(needle_name);
  }
  else
  {
    updateNeedlePose();
    if(!addNeedleToPlanningScene(needle_pose, needle_name))
    {
      return able_to_pick;
    };
  }

  switch (mode)
  {
    case NeedlePickMode::RANDOM:
    {
      ROS_INFO("Robot is going to grasp needle with random grasping parameters");
      moveit_msgs::MoveItErrorCodes error_code = randomPickNeedle(needle_pose, needle_name, possible_grasps_);
      if (error_code.val == error_code.SUCCESS)
      {
//        if (!davinciNeedleHandler_ -> attachObjectToRobot(needle_name, needleGraspData_.ee_tool_tip_link_))
//        {
//          ROS_INFO("Object is able to be picked up, but failed to be attached to link %s",
//                   needleGraspData_.ee_tool_tip_link_.c_str());
//          return able_to_pick;
//        }
        ROS_INFO("Object has been picked up");
        able_to_pick = true;
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
      moveit_msgs::MoveItErrorCodes error_code = definedPickNeedle(needle_pose, needle_name, defined_grasp_);
      if (error_code.val == error_code.SUCCESS)
      {
//        if (!davinciNeedleHandler_ -> attachObjectToRobot(needle_name, needleGraspData_.ee_tool_tip_link_))
//        {
//          ROS_INFO("Object is able to be picked up, but failed to be attached to link %s",
//                   needleGraspData_.ee_tool_tip_link_.c_str());
//          return able_to_pick;
//        }
        ROS_INFO("Object has been picked up");
        able_to_pick = true;
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
  moveit_msgs::MoveItErrorCodes error_code = placeNeedleHelper(needle_goal_pose, needle_name);
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


std::vector<moveit_msgs::Grasp> DavinciSimpleNeedleGrasper::getAllPossibleNeedleGraspsList() const
{
  return possible_grasps_list_;
}

std::vector<GraspInfo> DavinciSimpleNeedleGrasper::getAllPossibleNeedleGrasps() const
{
  return possible_grasps_;
}

//moveit_msgs::Grasp DavinciSimpleNeedleGrasper::getDefinedNeedleGrasp()
//{
//  return defined_grasp_.grasp;
//}

GraspInfo DavinciSimpleNeedleGrasper::getDefinedNeedleGrasp()
{
  return defined_grasp_;
}


geometry_msgs::PoseStamped DavinciSimpleNeedleGrasper::getNeedlePose() const
{
  return needle_pose_;
}


bool DavinciSimpleNeedleGrasper::addNeedleToPlanningScene(
    const geometry_msgs::PoseStamped& needle_origin,
    const std::string& needle_name)
{
  bool is_added = false;

//  moveit_msgs::GetPlanningScene pscene_srv;
//  pscene_srv.request.components.components =
//      moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
//      moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
//
//  if (!moveit_planning_scene_diff_client_.call(pscene_srv))
//  {
//    ROS_ERROR("DavinciSimpleNeedleGrasper: Can't obtain planning scene "
//              "in order to remove needle.");
//    return is_added;
//  }
//
//  if (DavinciMoveitGraspedObjectHandler::hasObject(needle_name,
//                                                   pscene_srv.response.scene.world.collision_objects,
//                                                   needle_collision_model_))
//  {
//    ROS_INFO("Needle %s already exists in planning scene", needle_name.c_str());
//    is_added = true;
//    return is_added;
//  }
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

//  moveit_msgs::GetPlanningScene pscene_srv;
//  pscene_srv.request.components.components =
//      moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
//      moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
//
//  if (!moveit_planning_scene_diff_client_.call(pscene_srv))
//  {
//    ROS_ERROR("DavinciSimpleNeedleGrasper: Can't obtain planning scene "
//              "in order to remove needle.");
//    return is_removed;
//  }
//
//  // check if the needle is in the planning scene world
//  bool is_in_world = DavinciMoveitGraspedObjectHandler::hasObject(needle_name,
//                                                                  pscene_srv.response.scene.world.collision_objects,
//                                                                  needle_collision_model_);
//
//  if (!is_in_world)
//  {
//    ROS_INFO("Needle %s is not in the planning scene world", needle_name.c_str());
//    is_removed = true;
//    return is_removed;
//  }

  objsCheckMap_ = planning_scene_ -> getObjects();
  ObjsCheckMap::iterator objs_it = objsCheckMap_ .find(needle_name);

  if (objs_it  == objsCheckMap_.end())
  {
    ROS_INFO("Needle %s is not in the planning scene world", needle_name.c_str());
    return is_removed;
  }

  needle_collision_model_.operation = needle_collision_model_.REMOVE;

  moveit_msgs::PlanningScene planning_scene;
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
                                                                           std::vector<GraspInfo>& possible_grasps)
{
  possible_grasps.clear();
  // std::vector<moveit_msgs::Grasp> possible_grasps;
  // Pick grasp
  simpleNeedleGraspGenerator_->generateSimpleNeedleGrasps(needle_pose, needleGraspData_, possible_grasps);

  needleGraspData_.print();
  // Visualize them
  //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
  moveit::planning_interface::MoveGroupInterface davinci_gripper_group(needleGraspData_.ee_group_);

  const robot_state::JointModelGroup* davinci_eef_jmg =
    davinci_gripper_group.getCurrentState()->getJointModelGroup(needleGraspData_.ee_group_);

  //visual_tools_->publishGrasps(possible_grasps, davinci_eef_jmg);

  // Prevent collision with table
  //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

  // Allow blocks to be touched by end effector
  {
    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(needle_name);

    // Add this list to all grasps
    for (std::size_t i = 0; i < possible_grasps.size(); ++i)
    {
      possible_grasps[i].grasp.allowed_touch_objects = allowed_touch_objects;
    }
  }

  possible_grasps_list_.clear();
  for (int i = 0; i < possible_grasps.size(); i++)
    possible_grasps_list_.push_back(possible_grasps[i].grasp);

  //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << possible_grasps[0]);
  return move_group_->pick(needle_name, possible_grasps_list_);
}

moveit_msgs::MoveItErrorCodes DavinciSimpleNeedleGrasper::definedPickNeedle(const geometry_msgs::PoseStamped &needle_pose,
                                                                            const std::string &needle_name,
                                                                            GraspInfo& defined_grasp)
{
  // moveit_msgs::Grasp defined_grasp;
  // Pick grasp
  simpleNeedleGraspGenerator_->generateDefinedSimpleNeedleGrasp(needle_pose, needleGraspData_, defined_grasp);

  needleGraspData_.print();
  // Visualize them
  //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
  moveit::planning_interface::MoveGroupInterface davinci_gripper_group(needleGraspData_.ee_group_);

  const robot_state::JointModelGroup* davinci_eef_jmg =
    davinci_gripper_group.getCurrentState()->getJointModelGroup(needleGraspData_.ee_group_);

//  visual_tools_->publishGrasps(possible_grasps, davinci_eef_jmg);

  // Prevent collision with table
  //move_group_->setSupport0SurfaceName(SUPPORT_SURFACE3_NAME);

  // Allow blocks to be touched by end effector
  {
    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(needle_name);

    // Add this list to all grasps
    defined_grasp.grasp.allowed_touch_objects = allowed_touch_objects;
  }

  //ROS_INFO_STREAM_NAMED("","Grasp 0\n" << possible_grasps[0]);
  return move_group_->pick(needle_name, defined_grasp.grasp);
}

moveit_msgs::MoveItErrorCodes DavinciSimpleNeedleGrasper::placeNeedleHelper(
  const geometry_msgs::Pose &needle_goal_pose,
  const std::string &needle_name)
{
  ROS_WARN_STREAM_NAMED("place","Placing '"<< needle_name << "'");

  std::vector<moveit_msgs::PlaceLocation> place_locations;

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = needleGraspData_.base_link_;
  pose_stamped.header.stamp = ros::Time::now();

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
    pre_place_approach.desired_distance = needleGraspData_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
    pre_place_approach.min_distance = needleGraspData_.approach_retreat_min_dist_; // half of the desired? Untested.
    pre_place_approach.direction.header.frame_id = needleGraspData_.base_link_;
    pre_place_approach.direction.vector.x = 0;
    pre_place_approach.direction.vector.y = 0;
    pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
    place_loc.pre_place_approach = pre_place_approach;

    // Retreat
    moveit_msgs::GripperTranslation post_place_retreat;
    post_place_retreat.direction.header.stamp = ros::Time::now();
    post_place_retreat.desired_distance = needleGraspData_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
    post_place_retreat.min_distance = needleGraspData_.approach_retreat_min_dist_; // half of the desired? Untested.
    post_place_retreat.direction.header.frame_id = needleGraspData_.base_link_;
    post_place_retreat.direction.vector.x = 0;
    post_place_retreat.direction.vector.y = 0;
    post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
    place_loc.post_place_retreat = post_place_retreat;

    // Post place posture - use same as pre-grasp posture (the OPEN command)
    place_loc.post_place_posture = needleGraspData_.pre_grasp_posture_;

    place_locations.push_back(place_loc);
  }

  // Prevent collision with table
  //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

  move_group_->setPlannerId("RRTConnectkConfigDefault");

  return move_group_->place(needle_name, place_locations);
}

bool DavinciSimpleNeedleGrasper::generateNeedleCollisionModel(
    const geometry_msgs::PoseStamped& needle_origin,
    const std::string& needle_name)
{

  bool able_to_generate = false;

  Eigen::Vector3d scale_vec(0.025, 0.025, 0.025);
  shapes::Mesh* m =
      shapes::createMeshFromResource("package://cwru_davinci_geometry_models/"
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

//void DavinciSimpleNeedleGrasper::updatePose(
//    const geometry_msgs::PoseStamped& new_needle_origin,
//    moveit_msgs::CollisionObject& obj)
//{
//
//  return;
//}

void DavinciSimpleNeedleGrasper::needlePoseCallBack(
    const geometry_msgs::PoseStamped& needle_pose)
{
  fresh_needle_pose_ = true;
  needle_pose_ = needle_pose;
}

void DavinciSimpleNeedleGrasper::updateNeedlePose()
{
  while (!fresh_needle_pose_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
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

bool DavinciSimpleNeedleGrasper::updateNeedleModel(const std::string &needle_name)
{
  bool updated = false;
  if(!removeNeedleFromPlanningScene(needle_name))
  {
    return updated;
  }

  updateNeedlePose();

  if(!addNeedleToPlanningScene(needle_pose_, needle_name))
    return updated;

  updated = true;
  return updated;
}

}  // namespace
