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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_simple_needle_grasp_test_node");

  ros::NodeHandle nh;

  ros::Publisher updated_needle_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("updated_needle_pose", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::PoseStamped needle_pose;
  needle_pose.header.stamp = ros::Time::now();
  needle_pose.header.frame_id = "world";
  needle_pose.pose.position.x = -0.248;
  needle_pose.pose.position.y = 0.0;
  needle_pose.pose.position.z = 0.45;
  needle_pose.pose.orientation.w = 1;
  needle_pose.pose.orientation.x = 0;
  needle_pose.pose.orientation.y = 0;
  needle_pose.pose.orientation.z = 0;

  while (ros::ok())
  {
    updated_needle_pose_pub.publish(needle_pose);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}


