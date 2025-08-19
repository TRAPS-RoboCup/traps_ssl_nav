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

#ifndef TRAPS__SSL_NAV__ROS2__POSE_WITH_COVARIANCE_HPP_
#define TRAPS__SSL_NAV__ROS2__POSE_WITH_COVARIANCE_HPP_

#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "rclcpp/type_adapter.hpp"
#include "traps/ssl_nav/ros2/pose.hpp"

namespace traps::ssl_nav::ros2
{
struct PoseWithCovariance
{
  Pose pose;
  Eigen::Matrix<double, 6, 6> covariance;
};
}  // namespace traps::ssl_nav::ros2

template<>
struct rclcpp::TypeAdapter<
  traps::ssl_nav::ros2::PoseWithCovariance, geometry_msgs::msg::PoseWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = traps::ssl_nav::ros2::PoseWithCovariance;
  using ros_message_type = geometry_msgs::msg::PoseWithCovariance;
  using PoseAdaptor = rclcpp::TypeAdapter<traps::ssl_nav::ros2::Pose, geometry_msgs::msg::Pose>;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    PoseAdaptor::convert_to_ros_message(source.pose, destination.pose);
    Eigen::Map<Eigen::Matrix<double, 6, 6>>(destination.covariance.data()) = source.covariance;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    PoseAdaptor::convert_to_custom(source.pose, destination.pose);
    destination.covariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6>>(source.covariance.data());
  }
};

#endif  // TRAPS__SSL_NAV__ROS2__POSE_WITH_COVARIANCE_HPP_
