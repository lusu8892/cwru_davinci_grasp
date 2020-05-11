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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

using namespace cwru_davinci_grasp;


moveit_msgs::AttachedCollisionObject generateNeedleCollisionModel(const geometry_msgs::PoseStamped& needle_origin, const std::string& needle_name)
{
  Eigen::Vector3d scale_vec(0.025, 0.025, 0.025);
  shapes::Mesh* m =
      shapes::createMeshFromResource("package://sim_gazebo/"
                                     "props/needle/mesh/needle_thin.dae",
                                     scale_vec);
  ROS_INFO("needle mesh loaded");
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  if (!shapes::constructMsgFromShape(m, mesh_msg))
  {
    ROS_ERROR("DavinciSimpleNeedleGrasper is unable to generate needle "
              "collision model");
  }
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::AttachedCollisionObject needle_collision_model_;
  needle_collision_model_.object.meshes.resize(1);
  needle_collision_model_.object.mesh_poses.resize(1);
  needle_collision_model_.object.meshes[0] = mesh;
  needle_collision_model_.object.header.frame_id = needle_origin.header.frame_id;
  needle_collision_model_.object.id = needle_name;

  needle_collision_model_.object.mesh_poses[0].position.x =
      needle_origin.pose.position.x;

  needle_collision_model_.object.mesh_poses[0].position.y =
      needle_origin.pose.position.y;

  needle_collision_model_.object.mesh_poses[0].position.z =
      needle_origin.pose.position.z;

  needle_collision_model_.object.mesh_poses[0].orientation.w =
      needle_origin.pose.orientation.w;

  needle_collision_model_.object.mesh_poses[0].orientation.x =
      needle_origin.pose.orientation.x;

  needle_collision_model_.object.mesh_poses[0].orientation.y =
      needle_origin.pose.orientation.y;

  needle_collision_model_.object.mesh_poses[0].orientation.z =
      needle_origin.pose.orientation.z;

  needle_collision_model_.object.operation = needle_collision_model_.object.ADD;
  //  scene, object will be replaced if it existed


  return needle_collision_model_;
}

int main(int argc, char **argv)
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

  ros::init(argc, argv, "davinci_test_grasping_main");

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

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ROS_INFO_NAMED("tutorial", "Frame in which the transforms for this model are computed: %s",
                 robot_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr rstate(new robot_state::RobotState(robot_model));
  rstate->setToDefaultValues();

  planning_scene::PlanningScene planning_scene(robot_model);
  planning_scene::PlanningScenePtr planning_scene_ptr;
  planning_scene_ptr.reset(new planning_scene::PlanningScene(robot_model));

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }

  DavinciSimpleNeedleGrasper needleGrasper(node_handle,
                                           node_handle_priv,
                                           which_arm,
                                           needle_name);

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_pose = needleGrasper.getAllPossibleNeedleGrasps(false);

  std::vector<double> needle_pose_translation;
  std::vector<double> needle_pose_orientation;
  int grasp_pose_index;

  if (node_handle_priv.hasParam("needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (!node_handle_priv.hasParam("grasp_pose_index"))
  {
    ROS_ERROR_STREAM_NAMED("davinci_test_grasping_main",
                           "Grasp configuration parameter "
                             "`grasp_pose_index` missing from rosparam "
                             "server. Did you load your grasping_and_needle_pose.yaml file?"
                             << node_handle_priv.getNamespace());

    return false;
  }
  node_handle_priv.getParam("grasp_pose_index", grasp_pose_index);

  geometry_msgs::PoseStamped needle_pose;
  needle_pose.header.frame_id = "/davinci_endo_cam_l";
  needle_pose.header.stamp = ros::Time::now();
  needle_pose.pose.position.x = needle_pose_translation[0];
  needle_pose.pose.position.y = needle_pose_translation[1];
  needle_pose.pose.position.z = needle_pose_translation[2];
  needle_pose.pose.orientation.x = needle_pose_orientation[0];
  needle_pose.pose.orientation.y = needle_pose_orientation[1];
  needle_pose.pose.orientation.z = needle_pose_orientation[2];
  needle_pose.pose.orientation.w = needle_pose_orientation[3];

  GraspInfo selected_grasp_pose = grasp_pose[grasp_pose_index];

  Eigen::Affine3d object_pose_affine;  // object pose w/rt base frame
  tf::poseMsgToEigen(needle_pose.pose, object_pose_affine);

  Eigen::Affine3d grasp_pose_affine = selected_grasp_pose.grasp_pose;
  Eigen::Affine3d tool_tip_pose = object_pose_affine * grasp_pose_affine.inverse();

  const std::string selected_group_name = which_arm;
  const std::string rest_group_name = (selected_group_name == "psm_one") ? "psm_two" : "psm_one";

  const robot_state::JointModelGroup *selected_joint_model_group = rstate->getJointModelGroup(selected_group_name);
  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = rstate->setFromIK(selected_joint_model_group, tool_tip_pose, attempts, timeout);

  if (found_ik)
  {
    const robot_state::JointModelGroup *rest_joint_model_group = rstate->getJointModelGroup(rest_group_name);
    rstate->setToDefaultValues(rest_joint_model_group, rest_group_name + "_home");

    std::string rest_group_eef_name = rest_joint_model_group->getAttachedEndEffectorNames()[0];
    const robot_state::JointModelGroup *rest_joint_model_group_eef = rstate->getJointModelGroup(rest_group_eef_name);
    rstate->setToDefaultValues(rest_joint_model_group_eef, rest_group_eef_name + "_home");
//    rstate->copyJointGroupPositions(selected_joint_model_group, hybrid_state->JointVariables().values);
    rstate->update();
  }
  else
  {
    printf("Did not find IK solution");
  }

  moveit_msgs::AttachedCollisionObject attached_needle = generateNeedleCollisionModel(needle_pose, needle_name);
  attached_needle.link_name = selected_joint_model_group->getOnlyOneEndEffectorTip()->getName();

  const robot_state::JointModelGroup *eef_group = robot_model->getJointModelGroup(
    selected_joint_model_group->getAttachedEndEffectorNames()[0]);
  attached_needle.touch_links = eef_group->getLinkModelNames();
  attached_needle.detach_posture.header.stamp = ros::Time::now();
  attached_needle.detach_posture.header.frame_id = robot_model->getModelFrame().c_str();
  attached_needle.detach_posture.joint_names = eef_group->getVariableNames();
  attached_needle.detach_posture.points.resize(3);
  attached_needle.detach_posture.points[0].positions.push_back(-1.0);
  attached_needle.detach_posture.points[1].positions.push_back(-1.0);
  attached_needle.detach_posture.points[2].positions.push_back(-1.0);

  moveit_msgs::PlanningScene planning_scene_msgs;

  planning_scene_msgs.world.collision_objects.clear();
  moveit::core::robotStateToRobotStateMsg(*rstate, planning_scene_msgs.robot_state);
  planning_scene_msgs.robot_state.attached_collision_objects.push_back(attached_needle);
  planning_scene_diff_publisher.publish(planning_scene_msgs);

  moveit::core::robotStateMsgToRobotState(planning_scene_msgs.robot_state, *rstate);
  planning_scene_ptr->setCurrentState(*rstate);
  collision_detection::CollisionRequest collision_request_simple_;
  collision_request_simple_.contacts = true;
  collision_request_simple_.max_contacts = 1000;
  collision_detection::CollisionResult res;
  planning_scene_ptr->checkCollisionUnpadded(collision_request_simple_, res, *rstate);
//  kstate->clearAttachedBody(object_name_);
  std::vector<double> selected_jt_values;
  planning_scene_ptr->getCurrentState().copyJointGroupPositions(selected_group_name, selected_jt_values);

  ROS_INFO("\n");
  ROS_INFO("selected group %s joint values: ", selected_group_name.c_str());
  for(int i = 0; i < selected_jt_values.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, selected_jt_values[i]);
  }

  std::vector<double> rest_jt_values;
  planning_scene_ptr->getCurrentState().copyJointGroupPositions(rest_group_name, rest_jt_values);

  ROS_INFO("rest group %s joint values: ", rest_group_name.c_str());
  for(int i = 0; i < rest_jt_values.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, rest_jt_values[i]);
  }

  const std::string rest_group = (rest_group_name == "psm_two") ? "PSM2" : "PSM1";
  Eigen::Affine3d rest_group_tip_pose = planning_scene_ptr->getCurrentState().getGlobalLinkTransform(rest_group +"_tool_tip_link");
  ROS_INFO_STREAM("rest_group_tip_pose_wrt_planning_frame \n" << rest_group_tip_pose.matrix());

  if(res.collision)
  {
    ROS_INFO("Invalid State: Robot state is in collision with planning scene. \n");
    collision_detection::CollisionResult::ContactMap contactMap = res.contacts;
    for(collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin(); it != contactMap.end(); ++it)
    {
      ROS_INFO("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
    }
  }

  ros::Duration(20.0).sleep();
  ros::shutdown();
  return 0;
}
