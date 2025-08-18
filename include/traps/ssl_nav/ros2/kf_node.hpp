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

#ifndef TRAPS__SSL_NAV__ROS2__KF_NODE_HPP_
#define TRAPS__SSL_NAV__ROS2__KF_NODE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "visibility.hpp"

namespace traps::ssl_nav::ros2
{

class KfNode : public rclcpp::Node
{
public:
  static constexpr auto kDefaultNodeName = "kf_node";

  TRAPS__SSL_NAV__ROS2_PUBLIC
  inline KfNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options)
  {
  }

  TRAPS__SSL_NAV__ROS2_PUBLIC
  explicit inline KfNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : KfNode(node_name, "", options)
  {
  }

  TRAPS__SSL_NAV__ROS2_PUBLIC
  explicit inline KfNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : KfNode(kDefaultNodeName, "", options)
  {
  }

  ~KfNode() override {}

private:
};

}  // namespace traps::ssl_nav::ros2

#endif  // TRAPS__SSL_NAV__ROS2__KF_NODE_HPP_
