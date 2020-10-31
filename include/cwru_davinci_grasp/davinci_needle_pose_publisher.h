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
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nhPriv
  );

  virtual ~DummyNeedleModifier(){}

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

  bool getCurrentNeedlePose
  (
  geometry_msgs::Pose& currentNeedlePose
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
  ros::NodeHandle               m_PrivNodeHandle;
  ros::ServiceClient            m_NeedlePoseClient;
  ros::ServiceClient            m_NeedlePoseModifier;

  geometry_msgs::Pose           m_PerturbedNeedlePose;
  gazebo_msgs::GetModelState    m_NeedleState;
  gazebo_msgs::SetModelState    m_PerturbedNeedleState;

  std::bernoulli_distribution   m_BernoulliDistribution;
  std::default_random_engine    m_Generator;
  std::random_device            m_RandSeed;
};

class DummyNeedleTracker : public DummyNeedleModifier
{
public :
  DummyNeedleTracker
  (
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nhPriv
  );

  virtual ~DummyNeedleTracker(){}

  virtual bool publishNeedlePose();

protected:
  bool getNeedlePose();

protected:
  ros::Publisher                        m_NeedlePosePub;

  geometry_msgs::PoseStamped            m_StampedNeedlePose;

  Eigen::Affine3d                       m_NeedlePoseWithNoise;
  Eigen::Affine3d                       m_IdealNeedlePose;
};

class DummyNeedleTrackerWithDepthNoise : public DummyNeedleTracker
{
public :
  DummyNeedleTrackerWithDepthNoise
  (
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nhPriv
  );

  virtual ~DummyNeedleTrackerWithDepthNoise(){}

  virtual bool publishNeedlePose() override;

protected:
  std::normal_distribution<double>      m_TransNoise;

  Eigen::Vector3d                       m_NoiseVec = Eigen::Vector3d::Zero();
};

class DummyNeedleTrackerWithRotationNoise : public DummyNeedleTracker
{
public :
  DummyNeedleTrackerWithRotationNoise
  (
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nhPriv
  );

  virtual ~DummyNeedleTrackerWithRotationNoise(){}

  virtual bool publishNeedlePose() override;

protected:
  std::normal_distribution<double>      m_OrientNoise;

  Eigen::Vector3d                       m_Zaxis = Eigen::Vector3d::UnitZ();
  Eigen::Matrix3d                       m_NoiseMat;
};

class DummyNeedleTrackerWithRandRotNoise : public DummyNeedleTrackerWithRotationNoise
{
public :
  DummyNeedleTrackerWithRandRotNoise
  (
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nhPriv
  );

  virtual ~DummyNeedleTrackerWithRandRotNoise(){}

  virtual bool publishNeedlePose() override;

protected:
  Eigen::Vector3d                         m_RandAxis;
  std::uniform_real_distribution<double>  m_UniformRealDistribution;
};