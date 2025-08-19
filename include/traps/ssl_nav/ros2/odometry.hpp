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

#ifndef TRAPS__SSL_NAV__ROS2__ODOMETRY_HPP_
#define TRAPS__SSL_NAV__ROS2__ODOMETRY_HPP_

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/type_adapter.hpp"
#include "std_msgs/msg/header.hpp"
#include "traps/ssl_nav/ros2/pose_with_covariance.hpp"
#include "traps/ssl_nav/ros2/twist_with_covariance.hpp"

namespace traps::ssl_nav::ros2
{
struct Odometry
{
  std_msgs::msg::Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};
}  // namespace traps::ssl_nav::ros2

template<>
struct rclcpp::TypeAdapter<traps::ssl_nav::ros2::Odometry, nav_msgs::msg::Odometry>
{
  using is_specialized = std::true_type;
  using custom_type = traps::ssl_nav::ros2::Odometry;
  using ros_message_type = nav_msgs::msg::Odometry;
  using PoseWithCovarianceAdaptor = rclcpp::TypeAdapter<
    traps::ssl_nav::ros2::PoseWithCovariance, geometry_msgs::msg::PoseWithCovariance>;
  using TwistWithCovarianceAdaptor = rclcpp::TypeAdapter<
    traps::ssl_nav::ros2::TwistWithCovariance, geometry_msgs::msg::TwistWithCovariance>;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.header = source.header;
    destination.child_frame_id = source.child_frame_id;
    PoseWithCovarianceAdaptor::convert_to_ros_message(source.pose, destination.pose);
    TwistWithCovarianceAdaptor::convert_to_ros_message(source.twist, destination.twist);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.header = source.header;
    destination.child_frame_id = source.child_frame_id;
    PoseWithCovarianceAdaptor::convert_to_custom(source.pose, destination.pose);
    TwistWithCovarianceAdaptor::convert_to_custom(source.twist, destination.twist);
  }
};

#endif  // TRAPS__SSL_NAV__ROS2__ODOMETRY_HPP_
