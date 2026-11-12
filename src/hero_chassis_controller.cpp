#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool PidController::init(const ros::NodeHandle& parent_nh,
                         const std::string& name) {
  ros::NodeHandle pid_nh(parent_nh, name);
  return pid_.init(pid_nh);
}

void PidController::reset() {
  pid_.reset();
}

double PidController::update(double target_velocity, double current_velocity,
                             const ros::Duration& period) {
  if (period.toSec() <= 0.0) {
    return 0.0;
  }
  return pid_.computeCommand(target_velocity - current_velocity, period);
}

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

  controller_nh.param("max_effort", max_effort_, 5.0);
  if (max_effort_ <= 0.0) {
    ROS_ERROR_NAMED("hero_chassis_controller",
                    "max_effort must be greater than zero");
    return false;
  }

  if (!wheel_pids_[0].init(controller_nh, "pid/front_left") ||
      !wheel_pids_[1].init(controller_nh, "pid/front_right") ||
      !wheel_pids_[2].init(controller_nh, "pid/back_left") ||
      !wheel_pids_[3].init(controller_nh, "pid/back_right")) {
    ROS_ERROR_NAMED("hero_chassis_controller",
                    "Failed to load one or more wheel PID configurations");
    return false;
  }

  std_msgs::Float64MultiArray initial_targets;
  initial_targets.data.assign(4, 0.0);
  target_speed_buffer_.writeFromNonRT(initial_targets);

  wheel_speed_subscriber_ =
      root_nh.subscribe("/wheel_speed_command", 1,
                        &HeroChassisController::wheelSpeedCommandCallback,
                        this);

  return true;
}

void HeroChassisController::starting(const ros::Time& time) {
  static_cast<void>(time);
  for (PidController& pid : wheel_pids_) {
    pid.reset();
  }

  std_msgs::Float64MultiArray initial_targets;
  initial_targets.data.assign(4, 0.0);
  target_speed_buffer_.writeFromNonRT(initial_targets);
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  static_cast<void>(time);

  const auto* target_message = target_speed_buffer_.readFromRT();
  std::array<double, 4> target_velocities{};
  if (target_message != nullptr && target_message->data.size() == 4) {
    std::copy(target_message->data.begin(), target_message->data.end(),
              target_velocities.begin());
  }

  const std::array<double, 4> current_velocities = {
      front_left_joint_.getVelocity(),
      front_right_joint_.getVelocity(),
      back_left_joint_.getVelocity(),
      back_right_joint_.getVelocity(),
  };

  const double max_effort = max_effort_;
  for (std::size_t i = 0; i < wheel_pids_.size(); ++i) {
    const double command =
        wheel_pids_[i].update(target_velocities[i], current_velocities[i],
                              period);
    const double limited_command =
        std::max(-max_effort, std::min(max_effort, command));

    switch (i) {
      case 0:
        front_left_joint_.setCommand(limited_command);
        break;
      case 1:
        front_right_joint_.setCommand(limited_command);
        break;
      case 2:
        back_left_joint_.setCommand(limited_command);
        break;
      case 3:
        back_right_joint_.setCommand(limited_command);
        break;
    }
  }
}

void HeroChassisController::stopping(const ros::Time& time) {
  static_cast<void>(time);
  for (PidController& pid : wheel_pids_) {
    pid.reset();
  }

  front_left_joint_.setCommand(0.0);
  front_right_joint_.setCommand(0.0);
  back_left_joint_.setCommand(0.0);
  back_right_joint_.setCommand(0.0);
}

void HeroChassisController::wheelSpeedCommandCallback(
    const std_msgs::Float64MultiArray::ConstPtr& message) {
  if (message->data.size() != 4) {
    ROS_WARN_NAMED("hero_chassis_controller",
                   "Expected 4 wheel velocities, received %zu",
                   message->data.size());
    return;
  }

  for (const double velocity : message->data) {
    if (!std::isfinite(velocity)) {
      ROS_WARN_NAMED("hero_chassis_controller",
                     "Rejected non-finite wheel velocity command");
      return;
    }
  }

  target_speed_buffer_.writeFromNonRT(*message);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(
    hero_chassis_controller::HeroChassisController,
    controller_interface::ControllerBase)
