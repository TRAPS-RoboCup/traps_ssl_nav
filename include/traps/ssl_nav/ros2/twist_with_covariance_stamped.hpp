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

#ifndef TRAPS__SSL_NAV__ROS2__TWIST_WITH_COVARIANCE_STAMPED_HPP_
#define TRAPS__SSL_NAV__ROS2__TWIST_WITH_COVARIANCE_STAMPED_HPP_

#include <utility>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/type_adapter.hpp"
#include "std_msgs/msg/header.hpp"
#include "traps/ssl_nav/ros2/twist_with_covariance.hpp"

namespace traps::ssl_nav::ros2
{
struct TwistWithCovarianceStamped
{
  std_msgs::msg::Header header;
  TwistWithCovariance twist;
};
}  // namespace traps::ssl_nav::ros2

template<>
struct rclcpp::TypeAdapter<
  traps::ssl_nav::ros2::TwistWithCovarianceStamped, geometry_msgs::msg::TwistWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = traps::ssl_nav::ros2::TwistWithCovarianceStamped;
  using ros_message_type = geometry_msgs::msg::TwistWithCovarianceStamped;
  using TwistWithCovarianceAdaptor = rclcpp::TypeAdapter<
    traps::ssl_nav::ros2::TwistWithCovariance, geometry_msgs::msg::TwistWithCovariance>;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.header = source.header;
    TwistWithCovarianceAdaptor::convert_to_ros_message(source.twist, destination.twist);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.header = source.header;
    TwistWithCovarianceAdaptor::convert_to_custom(source.twist, destination.twist);
  }
};

#endif  // TRAPS__SSL_NAV__ROS2__TWIST_WITH_COVARIANCE_STAMPED_HPP_
