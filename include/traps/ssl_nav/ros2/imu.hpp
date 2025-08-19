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

#ifndef TRAPS__SSL_NAV__ROS2__IMU_HPP_
#define TRAPS__SSL_NAV__ROS2__IMU_HPP_

#include <string>

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

namespace traps::ssl_nav::ros2
{
struct Imu
{
  std_msgs::msg::Header header;
  Eigen::Quaterniond orientation;
  Eigen::Matrix3d orientation_covariance;
  Eigen::Vector3d angular_velocity;
  Eigen::Matrix3d angular_velocity_covariance;
  Eigen::Vector3d linear_acceleration;
  Eigen::Matrix3d linear_acceleration_covariance;
};
}  // namespace traps::ssl_nav::ros2

template<>
struct rclcpp::TypeAdapter<traps::ssl_nav::ros2::Imu, sensor_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = traps::ssl_nav::ros2::Imu;
  using ros_message_type = sensor_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.header = source.header;
    destination.orientation.x = source.orientation.x();
    destination.orientation.y = source.orientation.y();
    destination.orientation.z = source.orientation.z();
    destination.orientation.w = source.orientation.w();

    Eigen::Map<Eigen::Matrix<double, 3, 3>>(destination.orientation_covariance.data()) =
      source.orientation_covariance;

    destination.angular_velocity.x = source.angular_velocity.x();
    destination.angular_velocity.y = source.angular_velocity.y();
    destination.angular_velocity.z = source.angular_velocity.z();

    Eigen::Map<Eigen::Matrix<double, 3, 3>>(destination.angular_velocity_covariance.data()) =
      source.angular_velocity_covariance;

    destination.linear_acceleration.x = source.linear_acceleration.x();
    destination.linear_acceleration.y = source.linear_acceleration.y();
    destination.linear_acceleration.z = source.linear_acceleration.z();

    Eigen::Map<Eigen::Matrix<double, 3, 3>>(destination.linear_acceleration_covariance.data()) =
      source.linear_acceleration_covariance;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.header = source.header;
    destination.orientation.x() = source.orientation.x;
    destination.orientation.y() = source.orientation.y;
    destination.orientation.z() = source.orientation.z;
    destination.orientation.w() = source.orientation.w;

    destination.orientation_covariance =
      Eigen::Map<const Eigen::Matrix<double, 3, 3>>(source.orientation_covariance.data());

    destination.angular_velocity.x() = source.angular_velocity.x;
    destination.angular_velocity.y() = source.angular_velocity.y;
    destination.angular_velocity.z() = source.angular_velocity.z;

    destination.angular_velocity_covariance =
      Eigen::Map<const Eigen::Matrix<double, 3, 3>>(source.angular_velocity_covariance.data());

    destination.linear_acceleration.x() = source.linear_acceleration.x;
    destination.linear_acceleration.y() = source.linear_acceleration.y;
    destination.linear_acceleration.z() = source.linear_acceleration.z;

    destination.linear_acceleration_covariance =
      Eigen::Map<const Eigen::Matrix<double, 3, 3>>(source.linear_acceleration_covariance.data());
  }
};

#endif  // TRAPS__SSL_NAV__ROS2__IMU_HPP_
