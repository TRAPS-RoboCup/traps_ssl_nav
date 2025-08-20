// Copyright 2025 TRAPS
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

#ifndef TRAPS__SSL_NAV__ROS2__POSE_HPP_
#define TRAPS__SSL_NAV__ROS2__POSE_HPP_

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/type_adapter.hpp"

namespace traps::ssl_nav::ros2
{
struct Pose
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};
}  // namespace traps::ssl_nav::ros2

template<>
struct rclcpp::TypeAdapter<traps::ssl_nav::ros2::Pose, geometry_msgs::msg::Pose>
{
  using is_specialized = std::true_type;
  using custom_type = traps::ssl_nav::ros2::Pose;
  using ros_message_type = geometry_msgs::msg::Pose;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.position.x = source.position.x();
    destination.position.y = source.position.y();
    destination.position.z = source.position.z();

    destination.orientation.x = source.orientation.x();
    destination.orientation.y = source.orientation.y();
    destination.orientation.z = source.orientation.z();
    destination.orientation.w = source.orientation.w();
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.position.x() = source.position.x;
    destination.position.y() = source.position.y;
    destination.position.z() = source.position.z;

    destination.orientation.x() = source.orientation.x;
    destination.orientation.y() = source.orientation.y;
    destination.orientation.z() = source.orientation.z;
    destination.orientation.w() = source.orientation.w;
  }
};

#endif  // TRAPS__SSL_NAV__ROS2__POSE_HPP_
