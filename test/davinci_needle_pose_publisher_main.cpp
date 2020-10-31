/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Case Western Reserve University
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

#include <cwru_davinci_grasp/davinci_needle_pose_publisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_needle_pose_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle nhPriv("~");
  ros::Rate loopRate(2);

  // pubMode == 0: accurate tracking
  // pubMode == 1: error on camera's z axis
  // pubMode == 2: error on needle's orientation
  DummyNeedleTracker * needleTracker = nullptr;

  int pubMode = 0;
  if (argc == 2)
  {
    pubMode = atoi(argv[1]);
  }

  switch (pubMode)
  {
    case 0:
      needleTracker = new DummyNeedleTracker(nh, nhPriv);
      break;
    case 1:
      needleTracker = new DummyNeedleTrackerWithDepthNoise(nh, nhPriv);
      break;
    case 2:
      needleTracker = new DummyNeedleTrackerWithRotationNoise(nh, nhPriv);
      break;
    case 3:
      needleTracker = new DummyNeedleTrackerWithRandRotNoise(nh, nhPriv);
      break;
  }

  if (!needleTracker)
  {
    ROS_ERROR("davinci_needle_pose_publisher: No needle tracker is instantiated");
    return 0;
  }

  while (ros::ok())
  {
    if (!needleTracker->publishNeedlePose())
      break;
    loopRate.sleep();
  }

  delete needleTracker;
  return 0;
}
