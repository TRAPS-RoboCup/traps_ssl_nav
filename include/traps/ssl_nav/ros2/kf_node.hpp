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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "traps/ssl_nav/kf.hpp"
#include "traps/ssl_nav/ros2/imu.hpp"
#include "traps/ssl_nav/ros2/odometry.hpp"
#include "traps/ssl_nav/ros2/pose_with_covariance_stamped.hpp"
#include "traps/ssl_nav/ros2/publisher_options.hpp"
#include "traps/ssl_nav/ros2/subscription_options.hpp"
#include "traps/ssl_nav/ros2/twist_with_covariance_stamped.hpp"
#include "traps/ssl_nav/ros2/visibility.hpp"

namespace traps::ssl_nav::ros2
{

class KfNode : public rclcpp::Node
{
  using Odom = traps::ssl_nav::ros2::Odometry;
  using OdomAdaptor = rclcpp::TypeAdapter<traps::ssl_nav::ros2::Odometry, nav_msgs::msg::Odometry>;
  using Pose = traps::ssl_nav::ros2::PoseWithCovarianceStamped;
  using PoseAdaptor = rclcpp::TypeAdapter<
    traps::ssl_nav::ros2::PoseWithCovarianceStamped, geometry_msgs::msg::PoseWithCovarianceStamped>;
  using Twist = traps::ssl_nav::ros2::TwistWithCovarianceStamped;
  using TwistAdaptor = rclcpp::TypeAdapter<
    traps::ssl_nav::ros2::TwistWithCovarianceStamped,
    geometry_msgs::msg::TwistWithCovarianceStamped>;
  using Imu = traps::ssl_nav::ros2::Imu;
  using ImuAdaptor = rclcpp::TypeAdapter<traps::ssl_nav::ros2::Imu, sensor_msgs::msg::Imu>;

public:
  static constexpr auto kDefaultNodeName = "kf_node";

  TRAPS__SSL_NAV__ROS2_PUBLIC
  inline KfNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", "odom")),
    child_frame_id_(this->declare_parameter("child_frame_id", "base_link")),
    default_pose_cov_(to_matrix(this->declare_parameter(
        "default_pose_cov",
        std::vector<double> {
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0,         //
      0.0, 10.0, 0.0, 0.0, 0.0, 0.0,         //
      0.0, 0.0, 10.0, 0.0, 0.0, 0.0,         //
      0.0, 0.0, 0.0, 10.0, 0.0, 0.0,         //
      0.0, 0.0, 0.0, 0.0, 10.0, 0.0,         //
      0.0, 0.0, 0.0, 0.0, 0.0, 10.0,         //
    }))),
    default_vel_cov_(to_matrix(this->declare_parameter(
        "default_vel_cov",
        std::vector<double> {
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0,        //
      0.0, 10.0, 0.0, 0.0, 0.0, 0.0,        //
      0.0, 0.0, 10.0, 0.0, 0.0, 0.0,        //
      0.0, 0.0, 0.0, 10.0, 0.0, 0.0,        //
      0.0, 0.0, 0.0, 0.0, 10.0, 0.0,        //
      0.0, 0.0, 0.0, 0.0, 0.0, 10.0         //
    }))),
    default_accel_cov_(to_matrix(this->declare_parameter(
        "default_accel_cov",
        std::vector<double> {
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0,         //
      0.0, 10.0, 0.0, 0.0, 0.0, 0.0,         //
      0.0, 0.0, 10.0, 0.0, 0.0, 0.0,         //
      0.0, 0.0, 0.0, 10.0, 0.0, 0.0,         //
      0.0, 0.0, 0.0, 0.0, 10.0, 0.0,         //
      0.0, 0.0, 0.0, 0.0, 0.0, 10.00         //
    }))),
    kf_(this->now_chrono()),
    odom_pub_(this->create_publisher<OdomAdaptor>(
        "odom", rclcpp::QoS(10), kQoSOverwriteablePublisherOptions)),
    pose_sub_count_(0),
    pose_sub_(this->create_subscription<PoseAdaptor>(
        "pose/with_cov", rclcpp::QoS(10),
        [this](std::shared_ptr<const Pose> pose) {this->update(std::move(pose));},
        kQoSOverwriteableSubscriptionOptions)),
    prediction_timer_(
      this->create_wall_timer(
        std::chrono::duration<double>(1.0 / this->declare_parameter("prediction_rate", 100.0)),
        [this]() {this->predict();}))
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

  ~KfNode() {}

private:
  static inline Eigen::Map<const Eigen::Matrix<double, 6, 6>> to_matrix(
    const std::vector<double> & vec)
  {
    if (vec.size() != 36) {
      throw std::runtime_error("Invalid size for matrix conversion, expected 36 elements.");
    }
    return Eigen::Map<const Eigen::Matrix<double, 6, 6>>(vec.data());
  }

  inline std::chrono::time_point<std::chrono::system_clock> now_chrono()
  {
    return std::chrono::time_point<std::chrono::system_clock>(
      std::chrono::nanoseconds(this->now().nanoseconds()));
  }

  void predict()
  {
    kf_.predict(std::chrono::system_clock::now());
    auto odom = std::make_unique<Odom>();
    odom->header.stamp = this->now();
    odom->header.frame_id = frame_id_;
    odom->child_frame_id = child_frame_id_;
    odom->pose.pose.position = kf_.position();
    odom->pose.pose.orientation = kf_.rotation();
    odom->pose.covariance = kf_.pose_covariance();
    odom->twist.twist.linear = kf_.velocity().segment<3>(0);
    odom->twist.twist.angular = kf_.velocity().segment<3>(3);
    odom->twist.covariance = kf_.velocity_covariance();
    odom_pub_->publish(std::move(odom));
  }

  void update(std::shared_ptr<const Pose> pose)
  {
    if (pose->header.frame_id != frame_id_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(), "Pose frame_id '%s' does not match node frame_id '%s'.",
        pose->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    kf_.update_by_pose(
      pose->pose.pose.position, pose->pose.pose.orientation, pose->pose.covariance);
  }

  void update(std::shared_ptr<const Twist> twist)
  {
    if (twist->header.frame_id != frame_id_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(), "Twist frame_id '%s' does not match node frame_id '%s'.",
        twist->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    Eigen::Vector<double, 6> velocity;
    velocity << twist->twist.twist.linear, twist->twist.twist.angular;
    kf_.update_by_velocity(velocity, twist->twist.covariance);
  }

  void update(std::shared_ptr<const Imu> imu)
  {
    if (imu->header.frame_id != frame_id_) {
      RCLCPP_WARN_ONCE(
        this->get_logger(), "IMU frame_id '%s' does not match node frame_id '%s'.",
        imu->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    kf_.update_by_imu(
      imu->linear_acceleration, imu->linear_acceleration_covariance, imu->angular_velocity,
      imu->angular_velocity_covariance);
  }

  // paraeter
  std::string frame_id_, child_frame_id_;
  Eigen::Matrix<double, 6, 6> default_pose_cov_, default_vel_cov_, default_accel_cov_;

  // kalman filter
  Kf kf_;

  // odom publisher
  rclcpp::Publisher<OdomAdaptor>::SharedPtr odom_pub_;

  // pose subscription
  std::size_t pose_sub_count_;
  rclcpp::Subscription<PoseAdaptor>::SharedPtr pose_sub_;

  // velocity subscription
  std::size_t vel_sub_count_;
  rclcpp::Subscription<TwistAdaptor>::SharedPtr vel_sub_;

  // imu subscription
  std::size_t imu_sub_count_;
  rclcpp::Subscription<ImuAdaptor>::SharedPtr imu_sub_;

  // prediction
  std::size_t prediction_count_;
  rclcpp::TimerBase::SharedPtr prediction_timer_;
};

}  // namespace traps::ssl_nav::ros2

#endif  // TRAPS__SSL_NAV__ROS2__KF_NODE_HPP_
