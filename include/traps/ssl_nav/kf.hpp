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

#ifndef TRAPS__SSL_NAV__KF_HPP_
#define TRAPS__SSL_NAV__KF_HPP_

#include <chrono>
#include <limits>

#include "Eigen/Dense"

namespace traps::ssl_nav
{

class Kf
{
  template<std::size_t Rows, std::size_t Cols>
  using Matrix = Eigen::Matrix<double, Rows, Cols>;

  template<std::size_t Size>
  using Vector = Eigen::Vector<double, Size>;

  using Quaternion = Eigen::Quaternion<double>;

  using AngleAxis = Eigen::AngleAxis<double>;

  using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

public:
  inline Kf(
    const TimePoint & stamp, const Matrix<6, 6> & accel_process_cov = Matrix<6, 6>::Identity())
  : state_(Vector<18>::Zero()),
    covariance_(Matrix<18, 18>::Identity() * kCovMax),
    accel_process_cov_(accel_process_cov),
    stamp_(stamp)
  {
  }

  /// @brief pose
  inline Eigen::VectorBlock<Vector<18>, 6> pose() {return state_.segment<6>(0);}

  inline Eigen::VectorBlock<Vector<18>, 3> position() {return state_.segment<3>(0);}

  inline Quaternion rotation()
  {
    const auto state_rotation = this->pose().segment<3>(3);
    const auto state_rotation_angle = state_rotation.norm();
    if (state_rotation_angle) {
      return Quaternion(
        AngleAxis(state_rotation_angle, state_rotation * (1.0 / state_rotation_angle))
        .toRotationMatrix());
    } else {
      return Quaternion::Identity();
    }
  }

  /// @brief velocity
  inline Eigen::VectorBlock<Vector<18>, 6> velocity() {return state_.segment<6>(6);}

  /// @brief acceleration
  inline Eigen::VectorBlock<Vector<18>, 6> acceleration() {return state_.segment<6>(12);}

  /// @brief prediction
  void predict(const std::chrono::time_point<std::chrono::system_clock> & stamp)
  {
    const auto dt = std::chrono::duration<double>(stamp - stamp_).count() * 0.01;
    const auto dt2 = dt * dt;

    // predict state
    this->pose() += this->velocity() * dt;
    this->velocity() += this->acceleration() * dt;

    // predict covariance
    const Matrix<12, 18> cov_delta_row = covariance_.block<12, 18>(6, 0) * dt;
    const Matrix<18, 12> cov_delta_col = covariance_.block<18, 12>(0, 6) * dt;
    const Matrix<12, 12> cov_delta2 = covariance_.block<12, 12>(6, 6) * dt2;
    covariance_.block<12, 18>(0, 0) += cov_delta_row;
    covariance_.block<18, 12>(0, 0) += cov_delta_col;
    covariance_.block<12, 12>(0, 0) += cov_delta2;
    covariance_.block<6, 6>(0, 0) += accel_process_cov_ * dt;

    // limit covariance
    covariance_ = covariance_.array().min(kCovMax);
  }

  /// @brief update
  void update_by_pose(
    const Vector<3> & position, const Quaternion & rotation, const Matrix<6, 6> & covariance)
  {
    Matrix<6, 18> h;
    h << Matrix<6, 6>::Identity(), Matrix<6, 6>::Zero(), Matrix<6, 6>::Zero();

    const auto y_rotation = rotation * this->rotation().inverse();
    const auto y_rotation_angle_axis = AngleAxis(y_rotation);

    Vector<6> y;
    y << position - this->position(), y_rotation_angle_axis.axis() * y_rotation_angle_axis.angle();

    this->update(h, y, covariance);
  }

  /// @brief update by velocity
  void update_by_velocity(const Vector<6> & velocity, const Matrix<6, 6> & covariance)
  {
    Matrix<6, 18> h;
    h << Matrix<6, 6>::Zero(), Matrix<6, 6>::Identity(), Matrix<6, 6>::Zero();

    auto y = velocity - this->velocity();

    this->update(h, y, covariance);
  }

  /// @brief update by imu
  void update_by_imu(
    const Vector<3> & acceleration_linear, const Matrix<3, 3> & acceleration_linear_covariance,
    const Vector<3> & velocity_angular, const Matrix<3, 3> & velocity_angular_covariance)
  {
    Matrix<6, 18> h;
    h << Matrix<3, 3>::Zero(), Matrix<3, 3>::Zero(), Matrix<3, 3>::Zero(), Matrix<3, 3>::Zero(),
      Matrix<3, 3>::Zero(), Matrix<3, 3>::Identity(), Matrix<3, 3>::Zero(), Matrix<3, 3>::Zero(),
      Matrix<3, 3>::Zero(), Matrix<3, 3>::Identity(), Matrix<3, 3>::Zero(), Matrix<3, 3>::Zero();

    Vector<6> y;
    y << acceleration_linear - this->acceleration().segment<3>(0),
      velocity_angular - this->velocity().segment<3>(3);

    Matrix<6, 6> covariance;
    covariance << acceleration_linear_covariance, Matrix<3, 3>::Zero(), Matrix<3, 3>::Zero(),
      velocity_angular_covariance;

    this->update(h, y, covariance);
  }

  inline Matrix<18, 18> & covariance() {return covariance_;}

  inline Eigen::Block<Matrix<18, 18>, 6, 6> pose_covariance()
  {
    return covariance_.block<6, 6>(0, 0);
  }

  inline Eigen::Block<Matrix<18, 18>, 6, 6> velocity_covariance()
  {
    return covariance_.block<6, 6>(6, 6);
  }

  inline Eigen::Block<Matrix<18, 18>, 6, 6> acceleration_covariance()
  {
    return covariance_.block<6, 6>(12, 12);
  }

private:
  template<class HVec, class YVec, class RMat>
  void update(const HVec & h, const YVec & y, const RMat & r)
  {
    // calculate kalman gain
    const auto s = h * covariance_ * h.transpose() + r.array().min(kCovMax).matrix();
    const auto k = covariance_ * h.transpose() * s.inverse();

    // update
    state_ += k * y;
    covariance_ -= k * h * covariance_;

    // limit covariance
    covariance_ = covariance_.array().min(kCovMax);
  }

  /// @brief state size
  static constexpr auto kStateSize = 6 * 3;

  /// @brief cov max
  static constexpr auto kCovMax = std::numeric_limits<double>::max() * 0.5;

  /// @brief robot state(pose, velocity, acceleration)
  Vector<kStateSize> state_;

  /// @brief state covariance
  Eigen::Matrix<double, kStateSize, kStateSize> covariance_;
  Matrix<6, 6> accel_process_cov_;

  /// @brief time stamp of the last prediction
  TimePoint stamp_;
};

}  // namespace traps::ssl_nav

#endif  // TRAPS__SSL_NAV__KF_HPP_
