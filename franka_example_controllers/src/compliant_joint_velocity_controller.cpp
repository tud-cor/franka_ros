// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/compliant_joint_velocity_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool CompliantJointVelocityController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CompliantJointVelocityController: Could not read parameter arm_id");
    return false;
  }
	
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CompliantJointVelocityController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
	
  if (!node_handle.getParam("alpha", alpha_)) {
    ROS_ERROR(
        "CompliantJointVelocityController:  Invalid or no alpha parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kp", kp_) || kp_.size() != 7) {
    ROS_ERROR(
        "CompliantJointVelocityController:  Invalid or no kp parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ki", ki_) || ki_.size() != 7) {
    ROS_ERROR(
        "CompliantJointVelocityController:  Invalid or no ki parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("vel_error_cum_max", vel_error_cum_max_) || vel_error_cum_max_.size() != 7) {
    ROS_ERROR(
        "CompliantJointVelocityController:  Invalid or no vel_error_cum_max provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("vel_error_cum_min", vel_error_cum_min_) || vel_error_cum_min_.size() != 7) {
    ROS_ERROR(
        "CompliantJointVelocityController:  Invalid or no vel_error_cum_min provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("CompliantJointVelocityController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("CompliantJointVelocityController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CompliantJointVelocityController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CompliantJointVelocityController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface== nullptr) {
    ROS_ERROR_STREAM(
        "CompliantJointVelocityController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CompliantJointVelocityController: Exception getting robot state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CompliantJointVelocityController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CompliantJointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);
	velocity_subscriber_ = node_handle.subscribe("/panda_joint_velocity_controller/command", 1, &CompliantJointVelocityController::joint_velocity_callback, this);
	kp_subscriber_ = node_handle.subscribe("controller_kp", 1, &CompliantJointVelocityController::kp_callback, this);
	ki_subscriber_ = node_handle.subscribe("controller_ki", 1, &CompliantJointVelocityController::ki_callback, this);


  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void CompliantJointVelocityController::joint_velocity_callback(const std_msgs::Float64MultiArray& msg) {
	for (size_t i = 0; i < 7; ++i) {
		dq_d_[i] = msg.data[i];
	}
}

void CompliantJointVelocityController::kp_callback(const std_msgs::Float64MultiArray& msg) {
	ROS_INFO_STREAM("Setting kp values");
	for (size_t i = 0; i < 7; ++i) {
		ROS_INFO_STREAM("Value " << i << " " << msg.data[i]);
		kp_[i] = msg.data[i];
	}
}
void CompliantJointVelocityController::ki_callback(const std_msgs::Float64MultiArray& msg) {
	ROS_INFO_STREAM("Setting ki values");
	for (size_t i = 0; i < 7; ++i) {
		ROS_INFO_STREAM("Value " << i << " " << msg.data[i]);
		ki_[i] = msg.data[i];
	}
}


void CompliantJointVelocityController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
	for (size_t i = 0; i < 7; ++i) {
		dq_d_[i] = 0.0;
	}
}

void CompliantJointVelocityController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha_) * dq_filtered_[i] + alpha_ * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
		vel_error_ = dq_d_[i] - dq_filtered_[i];
		vel_error_cum_[i] += vel_error_;
		vel_error_cum_[i] = std::max(vel_error_cum_min_[i], vel_error_cum_[i]);
		vel_error_cum_[i] = std::min(vel_error_cum_max_[i], vel_error_cum_[i]);
		tau_d_calculated[i] = kp_[i] * vel_error_ + ki_[i] * vel_error_cum_[i];
		/*
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
		*/
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> CompliantJointVelocityController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CompliantJointVelocityController,
                       controller_interface::ControllerBase)
