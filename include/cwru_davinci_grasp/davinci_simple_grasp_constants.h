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
   Desc:   Constants for generating simple grasping posture
*/
#ifndef CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_GRASP_CONSTANTS_H
#define CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_GRASP_CONSTANTS_H

namespace cwru_davinci_grasp
{
  static const double NEEDLE_RADIUS = 0.012;  // needle radius is 12mm

  static const double ABOVE_DIST = 0.01;  // pre-grasping vertical distance btw gripper tip to needle grasping point

  static const double THETA_0 = 0.004;
  static const double THETA_1 = 0;
  static const double THETA_2 = 0;
  static const double THETA_3 = 1 / NEEDLE_RADIUS;

  static const double GRASPING_PARAMETERS[] = {THETA_0, THETA_1, THETA_2, THETA_3};
}  // namespace

#endif  // CWRU_DAVINCI_GRASP_DAVINCI_SIMPLE_GRASP_CONSTANTS_H
