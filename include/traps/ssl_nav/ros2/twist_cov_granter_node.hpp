#ifndef TRAPS__SSL_NAV__ROS2__CONVERTER_NODE_HPP_
#define TRAPS__SSL_NAV__ROS2__CONVERTER_NODE_HPP_

#include <memory>
#include <string>

#include "eigen3/Eigen/Dense"
#include "rclcpp/node.hpp"
#include "traps/ssl_nav/ros2/publisher_options.hpp"
#include "traps/ssl_nav/ros2/subscription_options.hpp"
#include "traps/ssl_nav/ros2/twist_stamped.hpp"
#include "traps/ssl_nav/ros2/twist_with_covariance_stamped.hpp"
#include "traps/ssl_nav/ros2/visibility.hpp"

namespace traps::ssl_nav::ros2
{

class TwistCovGranterNode : public rclcpp::Node
{
  using TwistAdaptor = rclcpp::TypeAdapter<TwistStamped, geometry_msgs::msg::TwistStamped>;
  using TwistWithCovAdaptor =
    rclcpp::TypeAdapter<TwistWithCovarianceStamped, geometry_msgs::msg::TwistWithCovarianceStamped>;

public:
  static constexpr auto kDefaultNodeName = "twist_cov_granter";

  TRAPS__SSL_NAV__ROS2_PUBLIC
  inline TwistCovGranterNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    cov_coef_order0_(to_matrix(this->declare_parameter(
        "cov_coef.order0",
        std::vector<double> {
      0.01, 0.00, 0.00, 0.00, 0.00, 0.00,    //
      0.00, 0.01, 0.00, 0.00, 0.00, 0.00,    //
      0.00, 0.00, 0.01, 0.00, 0.00, 0.00,    //
      0.00, 0.00, 0.00, 0.01, 0.00, 0.00,    //
      0.00, 0.00, 0.00, 0.00, 0.01, 0.00,    //
      0.00, 0.00, 0.00, 0.00, 0.00, 0.01,    //
    }))),
    cov_coef_order2_(to_matrix(this->declare_parameter(
        "cov_coef.order2",
        std::vector<double> {
      0.25, 0.00, 0.00, 0.00, 0.00, 0.00,    //
      0.00, 0.25, 0.00, 0.00, 0.00, 0.00,    //
      0.00, 0.00, 0.25, 0.00, 0.00, 0.00,    //
      0.00, 0.00, 0.00, 0.25, 0.00, 0.00,    //
      0.00, 0.00, 0.00, 0.00, 0.25, 0.00,    //
      0.00, 0.00, 0.00, 0.00, 0.00, 0.25,    //
    }))),
    vel_with_cov_publisher_(this->create_publisher<TwistWithCovAdaptor>(
        "vel/with_cov", rclcpp::QoS(10), kQoSOverwriteablePublisherOptions)),
    vel_subscription_(this->create_subscription<TwistAdaptor>(
        "vel", rclcpp::QoS(10),
        [this](std::shared_ptr<const TwistStamped> vel) {this->grant(std::move(vel));},
        kQoSOverwriteableSubscriptionOptions))
  {
  }

  TRAPS__SSL_NAV__ROS2_PUBLIC
  explicit inline TwistCovGranterNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : TwistCovGranterNode(node_name, "", options)
  {
  }

  TRAPS__SSL_NAV__ROS2_PUBLIC
  explicit inline TwistCovGranterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : TwistCovGranterNode(kDefaultNodeName, "", options)
  {
  }

  ~TwistCovGranterNode() override {}

private:
  static Eigen::Matrix<double, 6, 6> to_matrix(const std::vector<double> & vec)
  {
    if (vec.size() != 36) {
      throw std::runtime_error("Invalid size for matrix conversion, expected 36 elements.");
    }
    return Eigen::Matrix<double, 6, 6>(vec.data());
  }

  void grant(std::shared_ptr<const TwistStamped> vel)
  {
    Eigen::Vector<double, 6> vel_twist;
    vel_twist << vel->twist.linear, vel->twist.angular;

    auto vel_with_cov = std::make_unique<TwistWithCovarianceStamped>();
    vel_with_cov->header = vel->header;
    vel_with_cov->twist.twist = vel->twist;
    vel_with_cov->twist.covariance =
      cov_coef_order0_ + cov_coef_order2_ * vel_twist * vel_twist.transpose();

    vel_with_cov_publisher_->publish(std::move(vel_with_cov));
  }

  // covariance coefficients
  Eigen::Matrix<double, 6, 6> cov_coef_order0_, cov_coef_order2_;

  // vel with cov publisher
  rclcpp::Publisher<TwistWithCovAdaptor>::SharedPtr vel_with_cov_publisher_;

  // vel subscription
  rclcpp::Subscription<TwistAdaptor>::SharedPtr vel_subscription_;
};

}  // namespace traps::ssl_nav::ros2

#endif  // TRAPS__SSL_NAV__ROS2__CONVERTER_NODE_HPP_
