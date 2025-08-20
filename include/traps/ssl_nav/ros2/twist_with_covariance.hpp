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

#ifndef TRAPS__SSL_NAV__ROS2__TWIST_WITH_COVARIANCE_HPP_
#define TRAPS__SSL_NAV__ROS2__TWIST_WITH_COVARIANCE_HPP_

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "rclcpp/type_adapter.hpp"
#include "traps/ssl_nav/ros2/twist.hpp"

namespace traps::ssl_nav::ros2
{
struct TwistWithCovariance
{
  Twist twist;
  Eigen::Matrix<double, 6, 6> covariance;
};
}  // namespace traps::ssl_nav::ros2

template<>
struct rclcpp::TypeAdapter<
  traps::ssl_nav::ros2::TwistWithCovariance, geometry_msgs::msg::TwistWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = traps::ssl_nav::ros2::TwistWithCovariance;
  using ros_message_type = geometry_msgs::msg::TwistWithCovariance;
  using TwistAdaptor = rclcpp::TypeAdapter<traps::ssl_nav::ros2::Twist, geometry_msgs::msg::Twist>;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    TwistAdaptor::convert_to_ros_message(source.twist, destination.twist);
    Eigen::Map<Eigen::Matrix<double, 6, 6>>(destination.covariance.data()) = source.covariance;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    TwistAdaptor::convert_to_custom(source.twist, destination.twist);
    destination.covariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6>>(source.covariance.data());
  }
};

#endif  // TRAPS__SSL_NAV__ROS2__TWIST_WITH_COVARIANCE_HPP_
