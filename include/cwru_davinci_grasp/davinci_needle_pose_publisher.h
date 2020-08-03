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
#include <gazebo_msgs/SetModelState.h>

#include <cwru_davinci_grasp/davinci_simple_grasp_generator.h>

class DummyNeedleModifier
{
public :
  DummyNeedleModifier(){}

  DummyNeedleModifier
  (
  const ros::NodeHandle &nh
  );

  bool perturbNeedlePose
  (
  double perturbRadian,
  const cwru_davinci_grasp::GraspInfo& selectedGrasp,
  const Eigen::Affine3d& idealNeedlePose = Eigen::Affine3d(),
  bool useIdealNdlPose = false,
  char rotAxis = 'z',
  bool randomRotAxis = true
  );

  bool getCurrentNeedlePose
  (
  Eigen::Affine3d& perturbedNeedlePose
  );

  bool setNeedlePose
  (
  double x,
  double y,
  double z,
  double qw,
  double qx,
  double qy,
  double qz
  );

  void radianOfChange
  (
  double& radOfChange,
  const Eigen::Affine3d& t1,
  const Eigen::Affine3d& t2
  );

protected:
  virtual void initialize();

protected:
  ros::NodeHandle               m_NodeHandle;
  ros::ServiceClient            m_NeedlePoseClient;
  ros::ServiceClient            m_NeedlePoseModifier;

  geometry_msgs::Pose           m_PerturbedNeedlePose;
  gazebo_msgs::GetModelState    m_NeedleState;
  gazebo_msgs::SetModelState    m_PerturbedNeedleState;

  std::bernoulli_distribution   m_Distribution;
  std::default_random_engine    m_Generator;
  std::random_device            m_RandSeed;
};

DummyNeedleModifier::DummyNeedleModifier
(
const ros::NodeHandle &nh
) 
: m_NodeHandle(nh)
{
  initialize();
}

void DummyNeedleModifier::initialize
(
)
{
  m_NeedlePoseClient = m_NodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  m_NeedlePoseModifier = m_NodeHandle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  m_NeedleState.request.model_name = "needle_r";
  m_NeedleState.request.relative_entity_name = "davinci_endo_cam_l";

  m_PerturbedNeedleState.request.model_state.model_name = "needle_r";
  m_PerturbedNeedleState.request.model_state.reference_frame = "davinci_endo_cam_l";
}

bool DummyNeedleModifier::getCurrentNeedlePose
(
Eigen::Affine3d& currentNeedlePose
)
{
  if (!m_NeedlePoseClient.call(m_NeedleState))
  {
    ROS_WARN("DummyNeedleModifier: Failed getting neeedle pose from gazebo");
    return false;
  }
  tf::poseMsgToEigen(m_NeedleState.response.pose, currentNeedlePose);

  return true;
}

bool DummyNeedleModifier::perturbNeedlePose
(
double perturbRadian,
const cwru_davinci_grasp::GraspInfo& selectedGrasp,
const Eigen::Affine3d& idealNeedlePose,
bool useIdealNdlPose,
char rotAxis,
bool randomRotAxis
)
{
  Eigen::Affine3d needlePose;
  if (useIdealNdlPose)
  {
    needlePose = idealNeedlePose;
  }
  else
  {
    if (!m_NeedlePoseClient.call(m_NeedleState))
    {
      ROS_WARN("DummyNeedleModifier: Failed getting neeedle pose from gazebo");
      return false;
    }
    tf::poseMsgToEigen(m_NeedleState.response.pose, needlePose);
  }

  double needle_radius = 0.0125;  // TODO
  double radius = selectedGrasp.graspParamInfo.param_3;
  double x = needle_radius * cos(radius);
  double y = needle_radius * sin(radius);

  // Transform vector into world frame
  Eigen::Vector3d vecFromNeedleOriginToGraspPointWrtWorldFrame = needlePose.linear() * Eigen::Vector3d(x, y, 0.0);
  vecFromNeedleOriginToGraspPointWrtWorldFrame.normalize();

  // Transform point into world frame
  Eigen::Vector3d graspPointLocationWrtWorldFrame = needlePose * Eigen::Vector3d(x, y, 0.0);

  Eigen::Vector3d needleFrameZaxisProjectedOnWorldFrame = needlePose.rotation().col(2);

  Eigen::Vector3d perturbAxisOfRotWrtWorldFrame = needleFrameZaxisProjectedOnWorldFrame;

  if (!randomRotAxis)
  {
    switch (rotAxis)
    {
      case 'z' :
        break;
      case 't' :
        perturbAxisOfRotWrtWorldFrame = vecFromNeedleOriginToGraspPointWrtWorldFrame.cross(needleFrameZaxisProjectedOnWorldFrame);
        ROS_INFO("DummyNeedleModifier: Perturbation is around Tangential Axis");
        // perturbRadian -= 0.08;  // compensation, this is a number from statistics
        break;
      case 'r' :
        perturbAxisOfRotWrtWorldFrame = vecFromNeedleOriginToGraspPointWrtWorldFrame;
        ROS_INFO("DummyNeedleModifier: Perturbation is around Radial Axis");
        // perturbRadian -= 0.03;  // compensation, this is a number from statistics
        break;
    }
  }
  else
  {
    if (m_Distribution(m_RandSeed))
    {
      if(m_Distribution(m_RandSeed))
      {
        rotAxis = 't';
        perturbAxisOfRotWrtWorldFrame = vecFromNeedleOriginToGraspPointWrtWorldFrame.cross(needleFrameZaxisProjectedOnWorldFrame);
        ROS_INFO("DummyNeedleModifier: Perturbation is around Tangential Axis");
        // perturbRadian -= 0.08;  // compensation, this is a number from statistics
      }
      else
      {
        rotAxis = 'r';
        perturbAxisOfRotWrtWorldFrame = vecFromNeedleOriginToGraspPointWrtWorldFrame;
        ROS_INFO("DummyNeedleModifier: Perturbation is around Radial Axis");
        // perturbRadian -= 0.03;  // compensation, this is a number from statistics
      }
    }
  }

  if (rotAxis == 'z')
    ROS_INFO("DummyNeedleModifier: Perturbation is around Vertical Axis");

  Eigen::AngleAxisd perturbRotationMatWrtWorldFrame(perturbRadian, perturbAxisOfRotWrtWorldFrame);

  Eigen::Vector3d needleLocationWrtWorldFrame = needlePose.translation();
  // Eigen::Vector3d graspPointLocationWrtWorldFrame = needleLocationWrtWorldFrame + vecFromNeedleOriginToGraspPointWrtWorldFrame;

  Eigen::Affine3d TM1(Eigen::Affine3d::Identity());     TM1.translation() = graspPointLocationWrtWorldFrame;

  Eigen::Affine3d TM2(perturbRotationMatWrtWorldFrame);

  Eigen::Affine3d TM3(Eigen::Affine3d::Identity());     TM3.translation() = -graspPointLocationWrtWorldFrame;

  Eigen::Affine3d perturbedNeedlePose = TM1 * TM2 * TM3 * needlePose;

  tf::poseEigenToMsg(perturbedNeedlePose, m_PerturbedNeedleState.request.model_state.pose);
  if (!m_NeedlePoseModifier.call(m_PerturbedNeedleState))
  {
    ROS_WARN("DummyNeedleModifier: Failed setting perturbed neeedle pose to gazebo");
    return false;
  }

  return true;
}

bool DummyNeedleModifier::setNeedlePose
(
double x,
double y,
double z,
double qw,
double qx,
double qy,
double qz
)
{
  m_PerturbedNeedleState.request.model_state.pose.position.x = x;
  m_PerturbedNeedleState.request.model_state.pose.position.y = y;
  m_PerturbedNeedleState.request.model_state.pose.position.z = z;

  m_PerturbedNeedleState.request.model_state.pose.orientation.w = qw;
  m_PerturbedNeedleState.request.model_state.pose.orientation.x = qx;
  m_PerturbedNeedleState.request.model_state.pose.orientation.y = qy;
  m_PerturbedNeedleState.request.model_state.pose.orientation.z = qz;
  if (!m_NeedlePoseModifier.call(m_PerturbedNeedleState))
  {
    ROS_WARN("DummyNeedleModifier: Failed to set needle pose");
    return false;
  }

  return true;
}

void DummyNeedleModifier::radianOfChange
(
double& radOfChange,
const Eigen::Affine3d& t1,
const Eigen::Affine3d& t2
)
{
  // q_w_ri, q_w_rp, q_ri_rp = q_ri_w * q_w_rp = transpose(q_w_ri) * q_w_rp;
  const Eigen::Matrix3d rot = t1.linear().inverse() * t2.linear();
  radOfChange = acos( (rot.trace() - 1) / 2 );
}

class DummyNeedleTracker : public DummyNeedleModifier
{
public :
  DummyNeedleTracker
  (
  const ros::NodeHandle &nh
  );

  bool publishNeedlePose();

  bool publishNeedlePoseWithNose();
protected:
  bool getNeedlePose();

protected:
  ros::Publisher                m_NeedlePosePub;

  geometry_msgs::PoseStamped    m_StampedNeedlePose;
};

DummyNeedleTracker::DummyNeedleTracker
(
const ros::NodeHandle &nh
)
 : DummyNeedleModifier(nh)
{
  m_NeedlePosePub = m_NodeHandle.advertise<geometry_msgs::PoseStamped>("/updated_needle_pose", 1000);
}

bool DummyNeedleTracker::publishNeedlePose()
{
  if (!getNeedlePose())
    return false;

  m_NeedlePosePub.publish(m_StampedNeedlePose);
  return true;
}

bool DummyNeedleTracker::publishNeedlePoseWithNose()
{
  if (!getNeedlePose())
    return false;

  // G_wn
  // G_wn * (Rot_o/n, T == 0)
  // (I, T.z + std::normal_distribution(0.002, 0.0001)) * G_wn
  // (I, T.z + 0.002) * G_wn *(Rot_o/n, T == 0)

  // adding depth error camera's z axis
  // adding rotational error needle's z axis

  // m_StampedNeedlePose.pose.position.z += 
  // m_NeedlePosePub.publish(m_StampedNeedlePose);
  return true;
}

bool DummyNeedleTracker::getNeedlePose
(
)
{
  if (!m_NeedlePoseClient.call(m_NeedleState))
  {
    ROS_WARN("DummyNeedleModifier: Failed getting neeedle pose from gazebo");
    return false;
  }

  m_StampedNeedlePose.header.frame_id = m_NeedleState.response.header.frame_id;
  m_StampedNeedlePose.header.stamp = ros::Time::now();
  m_StampedNeedlePose.pose = m_NeedleState.response.pose;
  return true;
}
