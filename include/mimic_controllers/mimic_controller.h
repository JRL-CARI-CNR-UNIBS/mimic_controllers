#pragma once



# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <subscription_notifier/subscription_notifier.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>
# include <name_sorting/name_sorting.h>
# include <ros/callback_queue.h>

namespace robot_control
{

class MimicController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

protected:

  hardware_interface::EffortJointInterface* m_hw;


  void setTargetCallback(const sensor_msgs::JointStateConstPtr& msg);

  std::string m_tool_frame;
  ros::CallbackQueue m_queue;

  std::string m_leading_joint;
  std::vector<std::string> m_joint_names;
  std::vector<double> m_multiplier;
  std::vector<double> m_offset;
  std::vector<double> m_kp;
  std::vector<double> m_kv;
  std::vector<double> m_ki;
  std::vector<double> m_xi; // integral state
  std::vector<double> m_max_effort;

  std::vector<hardware_interface::JointHandle> m_joint_handles;
  hardware_interface::JointHandle m_leading_joint_handle;

  double m_target_position;
  double m_target_velocity;
  double m_target_effort;

  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_target_js_rec;

  ros::Subscriber  m_js_target_sub;
  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  std::size_t m_nax;
  bool m_configured;
};


}
