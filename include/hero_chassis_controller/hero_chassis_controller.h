#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace hero_chassis_controller {

class HeroChassisController : public 
			      controller_interface::Controller
			<hardware_interface::EffortJointInterface>
{
	public:
  	HeroChassisController() = default;
	~HeroChassisController(） override = default;

	bool init(hardware_interface::EffortJointInterface *effort_joint_interface,ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh) overridde;

	void update(const ros::Time &time, const ros::Duration &period)override;

	private:
	hardware_interface::JointHandle front_left_joint_,front_right_joint_,back_left_joint_,back_right_joint_;
	int state_{};
	ros::Time last_change_;
};
}

#endif
