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

#ifndef TRAPS__SSL_NAV__ROS2__PUBLISHER_OPTIONS_HPP_
#define TRAPS__SSL_NAV__ROS2__PUBLISHER_OPTIONS_HPP_

#include "rclcpp/publisher_options.hpp"

namespace traps::ssl_nav::ros2
{

static const auto kQoSOverwriteablePublisherOptions = [] {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return options;
  }();

static const auto kQoSOverwriteableStaticPublisherOptions = [] {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return options;
  }();
}  // namespace traps::ssl_nav::ros2

#endif  // TRAPS__SSL_NAV__ROS2__PUBLISHER_OPTIONS_HPP_
