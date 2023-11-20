// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>
#include <Eigen/Dense>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

class CompliantJointVelocityController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void joint_velocity_callback(const std_msgs::Float64MultiArray& msg);
  void kp_callback(const std_msgs::Float64MultiArray& msg);
  void ki_callback(const std_msgs::Float64MultiArray& msg);
  void load_callback(const std_msgs::Float64& msg);

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};
  double vel_error_{0.0};
  double alpha_;

  std::vector<double> kp_;
  std::vector<double> ki_;
  std::vector<double> vel_error_cum_max_;
  std::vector<double> vel_error_cum_min_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  std::array<double, 7> vel_error_cum_;
  std::array<double, 7> dq_d_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  Eigen::Matrix<double, 7, 1> tau_load_;
  Eigen::Matrix<double, 6, 1> load_;
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber kp_subscriber_;
  ros::Subscriber ki_subscriber_;
  ros::Subscriber load_subscriber_;
};

}  // namespace franka_example_controllers
