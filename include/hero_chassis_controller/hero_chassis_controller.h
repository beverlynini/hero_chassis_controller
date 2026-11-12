#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_

#include <array>
#include <string>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64MultiArray.h>

namespace hero_chassis_controller {

class PidController {
 public:
  PidController() = default;

  bool init(const ros::NodeHandle& parent_nh, const std::string& name);
  void reset();
  double update(double target_velocity, double current_velocity,
                const ros::Duration& period);

 private:
  control_toolbox::Pid pid_;
};

class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  void wheelSpeedCommandCallback(
      const std_msgs::Float64MultiArray::ConstPtr& message);

  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;

  std::array<PidController, 4> wheel_pids_;
  ros::Subscriber wheel_speed_subscriber_;
  realtime_tools::RealtimeBuffer<std_msgs::Float64MultiArray>
      target_speed_buffer_;
  double max_effort_{5.0};
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H_
