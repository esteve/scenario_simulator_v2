// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GEOMETRY_MATH__TRANSFORM_HPP_
#define GEOMETRY_MATH__TRANSFORM_HPP_

#include <geometry_msgs/msg/pose.hpp>

namespace geometry_math
{
const geometry_msgs::msg::Pose getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to);
const geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & point);
std::vector<geometry_msgs::msg::Point> transformPoints(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & points);
}  // namespace geometry_math

#endif  // GEOMETRY_MATH__TRANSFORM_HPP_