#include <mimic_controllers/two_fingers_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_control::TwoFingersController, controller_interface::ControllerBase);


namespace robot_control
{

bool TwoFingersController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

  m_hw=hw;
  m_controller_nh=controller_nh;
  m_root_nh=root_nh;

  m_nax=2;


  if (!m_controller_nh.getParam("leading_joint",m_leading_joint))
  {
    ROS_ERROR("%s/leading_joint not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("following_joint",m_following_joint))
  {
    ROS_ERROR("%s/following_joint not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  if (!m_controller_nh.getParam("spring",m_spring))
  {
    ROS_ERROR("%s/spring not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  else if (m_spring<0)
  {
    ROS_ERROR("%s/spring must be non negative",m_controller_nh.getNamespace().c_str());
    return false;
  }

  if (!m_controller_nh.getParam("damper",m_damper))
  {
    ROS_ERROR("%s/damper not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  else if (m_damper<0)
  {
    ROS_ERROR("%s/damper must be non negative",m_controller_nh.getNamespace().c_str());
    return false;
  }

  m_joint_names.push_back(m_leading_joint);
  m_joint_names.push_back(m_following_joint);

  // check joint names
  bool flag=false;
  for (std::string joint_name: m_joint_names)
  {
    for (unsigned idx=0;idx<m_hw->getNames().size();idx++)
    {
      if (!m_hw->getNames().at(idx).compare(joint_name))
      {
        m_joint_handles.push_back(m_hw->getHandle(joint_name));
        ROS_DEBUG("ADD %s handle",joint_name.c_str());
        flag=true;
        break;
      }
    }
    if (!flag)
    {
      ROS_FATAL("Joint %s is not managed",joint_name.c_str());
      return false;
    }
  }

  // set target topic name
  std::string setpoint_topic_name;
  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_js_rec.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh,setpoint_topic_name, 1,boost::bind(&TwoFingersController::setTargetCallback,this,_1)));

  ROS_DEBUG("Controller '%s' controls the following joint:",m_controller_nh.getNamespace().c_str());
  for (std::string& name: m_joint_names)
    ROS_DEBUG("- %s",name.c_str());

  m_target_effort = m_joint_handles.at(0).getEffort();

  return true;
}

void TwoFingersController::starting(const ros::Time& /*time*/)
{
  m_configured = false;
  m_target_effort = m_joint_handles.at(0).getEffort();
}

void TwoFingersController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller", m_controller_nh.getNamespace().c_str());
}

void TwoFingersController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{

  m_queue.callAvailable();

  double delta_pos=m_joint_handles.at(0).getPosition()-m_joint_handles.at(1).getPosition();
  double delta_vel=m_joint_handles.at(0).getVelocity()-m_joint_handles.at(1).getVelocity();

  double correction=delta_pos*m_spring+delta_vel*m_damper;

  m_joint_handles.at(0).setCommand(m_target_effort-correction);
  m_joint_handles.at(1).setCommand(m_target_effort+correction);
}


void TwoFingersController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    std::vector<std::string> jn;
    jn.push_back(m_leading_joint);
    if (!name_sorting::permutationName(jn,tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort))
    {
      ROS_ERROR_THROTTLE(1,"joints not found");
      m_configured=false;
      return;
    }
    if (!m_configured)
      ROS_INFO("First target message received");

    m_configured=true;
    m_target_effort = tmp_msg.effort.at(0);
  }
  catch(...)
  {
    ROS_ERROR("something wrong in target callback");
    m_configured=false;
  }
}
}  //  namespace robot_control
