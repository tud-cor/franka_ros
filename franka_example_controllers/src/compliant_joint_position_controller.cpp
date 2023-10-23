// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/compliant_joint_position_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool CompliantJointPositionController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CompliantJointPositionController: Could not read parameter arm_id");
    return false;
  }
	
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CompliantJointPositionController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
	
  if (!node_handle.getParam("alpha", alpha_)) {
    ROS_ERROR(
        "CompliantJointPositionController:  Invalid or no alpha parameter provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kp", kp_) || kp_.size() != 7) {
    ROS_ERROR(
        "CompliantJointPositionController:  Invalid or no kp parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("ki", ki_) || ki_.size() != 7) {
    ROS_ERROR(
        "CompliantJointPositionController:  Invalid or no ki parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("kd", kd_) || kd_.size() != 7) {
    ROS_ERROR(
        "CompliantJointPositionController:  Invalid or no kd parameters provided, aborting "
        "controller init!");
    return false;
  }
  
  if (!node_handle.getParam("pos_error_cum_limit", pos_error_cum_limit_) || pos_error_cum_limit_.size() != 7) {
    ROS_ERROR(
        "CompliantJointPositionController:  Invalid or no pos_error_cum_limit provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("CompliantJointPositionController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("CompliantJointPositionController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CompliantJointPositionController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CompliantJointPositionController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface== nullptr) {
    ROS_ERROR_STREAM(
        "CompliantJointPositionController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CompliantJointPositionController: Exception getting robot state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CompliantJointPositionController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CompliantJointPositionController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  pos_error_publisher_.init(node_handle, "position_error", 1);
	position_subscriber_ = node_handle.subscribe("/compliant_joint_position_controller/command", 1, &CompliantJointPositionController::joint_position_callback, this);

  dynamic_reconfigure_pid_params_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_pid_params_node");

  dynamic_server_pid_params_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::pid_paramsConfig>>(

      dynamic_reconfigure_pid_params_node_);
  dynamic_server_pid_params_->setCallback(
      boost::bind(&CompliantJointPositionController::pidParamsCallback, this, _1, _2));

	// kp_subscriber_ = node_handle.subscribe("controller_kp", 1, &CompliantJointPositionController::kp_callback, this);
	// ki_subscriber_ = node_handle.subscribe("controller_ki", 1, &CompliantJointPositionController::ki_callback, this);
	// kd_subscriber_ = node_handle.subscribe("controller_kd", 1, &CompliantJointPositionController::kd_callback, this);


  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  std::fill(q_filtered_.begin(), q_filtered_.end(), 0);

  // set initial q_d_ (desired) to current state
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_d_ = robot_state.q;

  return true;
}

void CompliantJointPositionController::pidParamsCallback(
    franka_example_controllers::pid_paramsConfig& config,
    uint32_t /*level*/) {
  kp_[0] = config.kp1;
  kp_[1] = config.kp2;
  kp_[2] = config.kp3;
  kp_[3] = config.kp4;
  kp_[4] = config.kp5;
  kp_[5] = config.kp6;
  kp_[6] = config.kp7;

  ki_[0] = config.ki1;
  ki_[1] = config.ki2;
  ki_[2] = config.ki3;
  ki_[3] = config.ki4;
  ki_[4] = config.ki5;
  ki_[5] = config.ki6;
  ki_[6] = config.ki7;

  kd_[0] = config.kd1;
  kd_[1] = config.kd2;
  kd_[2] = config.kd3;
  kd_[3] = config.kd4;
  kd_[4] = config.kd5;
  kd_[5] = config.kd6;
  kd_[6] = config.kd7;

  pos_error_cum_limit_[0] = config.ki1_saturation;
  pos_error_cum_limit_[1] = config.ki2_saturation;
  pos_error_cum_limit_[2] = config.ki3_saturation;
  pos_error_cum_limit_[3] = config.ki4_saturation;
  pos_error_cum_limit_[4] = config.ki5_saturation;
  pos_error_cum_limit_[5] = config.ki6_saturation;
  pos_error_cum_limit_[6] = config.ki7_saturation;
}

void CompliantJointPositionController::joint_position_callback(const sensor_msgs::JointState& msg) {
	for (size_t i = 0; i < 7; ++i) {
		q_d_[i] = msg.position[i];
		dq_d_[i] = msg.velocity[i];
	}
}

void CompliantJointPositionController::kd_callback(const std_msgs::Float64MultiArray& msg) {
	ROS_INFO_STREAM("Setting kp values");
	for (size_t i = 0; i < 7; ++i) {
		ROS_INFO_STREAM("Value " << i << " " << msg.data[i]);
		kd_[i] = msg.data[i];
	}
}

void CompliantJointPositionController::kp_callback(const std_msgs::Float64MultiArray& msg) {
	ROS_INFO_STREAM("Setting kp values");
	for (size_t i = 0; i < 7; ++i) {
		ROS_INFO_STREAM("Value " << i << " " << msg.data[i]);
		kp_[i] = msg.data[i];
	}
}
void CompliantJointPositionController::ki_callback(const std_msgs::Float64MultiArray& msg) {
	ROS_INFO_STREAM("Setting ki values");
	for (size_t i = 0; i < 7; ++i) {
		ROS_INFO_STREAM("Value " << i << " " << msg.data[i]);
		ki_[i] = msg.data[i];
	}
}


void CompliantJointPositionController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
	for (size_t i = 0; i < 7; ++i) {
		dq_d_[i] = 0.0;
	}
}

void CompliantJointPositionController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha_) * dq_filtered_[i] + alpha_ * robot_state.dq[i];
    q_filtered_[i] = (1 - alpha_) * q_filtered_[i] + alpha_ * robot_state.q[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
		vel_error_ = dq_d_[i] - dq_filtered_[i];
		pos_error_[i] = q_d_[i] - q_filtered_[i];

		pos_error_cum_[i] += pos_error_[i];
		pos_error_cum_[i] = std::max(-pos_error_cum_limit_[i], pos_error_cum_[i]);
		pos_error_cum_[i] = std::min(pos_error_cum_limit_[i], pos_error_cum_[i]);
		tau_d_calculated[i] = kp_[i] * pos_error_[i] + kd_[i] * vel_error_ + ki_[i] * pos_error_cum_[i] * 0.001;
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

  if (rate_trigger_() && pos_error_publisher_.trylock()) {
    std::vector<double> pos_error_data(pos_error_.begin(), pos_error_.end());
    pos_error_publisher_.msg_.data = pos_error_data; 
    pos_error_publisher_.unlockAndPublish();
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

std::array<double, 7> CompliantJointPositionController::saturateTorqueRate(
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

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CompliantJointPositionController,
                       controller_interface::ControllerBase)
