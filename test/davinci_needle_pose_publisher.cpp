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
 * Description: A needle pose publisher for testing simple needle grasping.
 * This node is to publish updated needle pose.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_state_plugins/GazeboObjectInfo.h>

class DummyNeedleTracker : public gazebo::GazeboObjectInfo
{
public :
  DummyNeedleTracker();

  virtual ~DummyNeedleTracker(){};

  void setNeedlePose();

  void getNeedlePose();
};

DummyNeedleTracker::DummyNeedleTracker() : gazebo::GazeboObjectInfo()
{}

void DummyNeedleTracker::setNeedlePose()
{
  return;
}

void DummyNeedleTracker::getNeedlePose()
{
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_needle_pose_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle node_handle_priv("~");

  ros::ServiceClient needle_model_pose_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::Publisher updated_needle_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/updated_needle_pose", 1000);
  ros::Rate loop_rate(1.0);

  std::vector<double> needle_pose_translation;
  std::vector<double> needle_pose_orientation;

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

  gazebo_msgs::GetModelState needle_model_pose;
  while (ros::ok())
  {
    needle_model_pose.request.model_name = "needle_r";
    needle_model_pose.request.relative_entity_name = "davinci_endo_cam_l";
    needle_model_pose_client.call(needle_model_pose);

    needle_pose.pose = needle_model_pose.response.pose;
    updated_needle_pose_pub.publish(needle_pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
