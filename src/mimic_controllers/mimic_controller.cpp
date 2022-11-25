#include <mimic_controllers/mimic_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_control::MimicController, controller_interface::ControllerBase);


namespace robot_control
{

bool MimicController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{

  m_hw=hw;
  m_controller_nh=controller_nh;
  m_root_nh=root_nh;

  if (!m_controller_nh.getParam("joint_names",m_joint_names))
  {
    ROS_ERROR("%s/joint_names not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }

  m_nax=m_joint_names.size();


  if (!m_controller_nh.getParam("multiplier",m_multiplier))
  {
    ROS_INFO("%s/multiplier not defined, using 1.0",m_controller_nh.getNamespace().c_str());
    m_multiplier.resize(m_nax,1.0);
  }
  else
  {
    if (m_multiplier.size() != m_nax)
    {
      ROS_ERROR("%s/multiplier has wrong dimension (%zu instead of %zu)",m_controller_nh.getNamespace().c_str(),m_multiplier.size(),m_nax);
      return false;
    }
  }

  if (!m_controller_nh.getParam("offset",m_offset))
  {
    ROS_INFO("%s/offset not defined, using 0.0",m_controller_nh.getNamespace().c_str());
    m_offset.resize(m_nax,0.0);
  }
  else
  {
    if (m_offset.size() != m_nax)
    {
      ROS_ERROR("%s/offset has wrong dimension (%zu instead of %zu)",m_controller_nh.getNamespace().c_str(),m_offset.size(),m_nax);
      return false;
    }
  }

  if (!m_controller_nh.getParam("Kp",m_kp))
  {
    ROS_ERROR("%s/Kp not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  else
  {
    if (m_kp.size() != m_nax)
    {
      ROS_ERROR("%s/Kp has wrong dimension (%zu instead of %zu)",m_controller_nh.getNamespace().c_str(),m_kp.size(),m_nax);
      return false;
    }
  }

  if (!m_controller_nh.getParam("Kv",m_kv))
  {
    ROS_INFO("%s/Kv not defined, using 0.0",m_controller_nh.getNamespace().c_str());
    m_kv.resize(m_nax,0.0);
  }
  else
  {
    if (m_kv.size() != m_nax)
    {
      ROS_ERROR("%s/Kv has wrong dimension (%zu instead of %zu)",m_controller_nh.getNamespace().c_str(),m_kv.size(),m_nax);
      return false;
    }
  }

  if (!m_controller_nh.getParam("Ki",m_ki))
  {
    ROS_INFO("%s/Ki not defined, using 0.0",m_controller_nh.getNamespace().c_str());
    m_ki.resize(m_nax,0.0);
  }
  else
  {
    if (m_ki.size() != m_nax)
    {
      ROS_ERROR("%s/Ki has wrong dimension (%zu instead of %zu)",m_controller_nh.getNamespace().c_str(),m_ki.size(),m_nax);
      return false;
    }
  }

  if (!m_controller_nh.getParam("max_effort",m_max_effort))
  {
    ROS_ERROR("%s/max_effort not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }
  else
  {
    if (m_max_effort.size() != m_nax)
    {
      ROS_ERROR("%s/max_effort has wrong dimension (%zu instead of %zu)",m_controller_nh.getNamespace().c_str(),m_max_effort.size(),m_nax);
      return false;
    }
  }

  m_xi.resize(m_nax,0.0);

  if (!m_controller_nh.getParam("leading_joint",m_leading_joint))
  {
    ROS_ERROR("%s/leading_joint not defined",m_controller_nh.getNamespace().c_str());
    return false;
  }

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

  try {
    m_leading_joint_handle=m_hw->getHandle(m_leading_joint);

  } catch (std::logic_error& e) {
    ROS_FATAL("Leading joint %s is not managed by the hardware interface",m_leading_joint.c_str());
    ROS_FATAL("List of joints managed by '%s':",m_controller_nh.getNamespace().c_str());
    for (std::string& name: m_joint_names)
      ROS_FATAL("- %s",name.c_str());
    return false;
  }

  // set target topic name
  std::string setpoint_topic_name;
  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace()+"/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_js_rec.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh,setpoint_topic_name, 1,boost::bind(&MimicController::setTargetCallback,this,_1)));

  ROS_DEBUG("Controller '%s' controls the following joint:",m_controller_nh.getNamespace().c_str());
  for (std::string& name: m_joint_names)
    ROS_DEBUG("- %s",name.c_str());

  m_target_position = m_leading_joint_handle.getPosition();
  m_target_velocity = m_leading_joint_handle.getVelocity();

  return true;
}

void MimicController::starting(const ros::Time& /*time*/)
{
  m_configured = false;

  m_target_position = m_leading_joint_handle.getPosition();
  m_target_velocity = m_leading_joint_handle.getVelocity();
}

void MimicController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("[ %s ] Stopping controller", m_controller_nh.getNamespace().c_str());
  m_target_position = m_leading_joint_handle.getPosition();
  m_target_velocity = m_leading_joint_handle.getVelocity();
}

void MimicController::update(const ros::Time& /*time*/, const ros::Duration& period)
{

  m_queue.callAvailable();


  for (unsigned int iax=0;iax<m_nax;iax++)
  {
    double position = m_joint_handles.at(iax).getPosition();
    double velocity = m_joint_handles.at(iax).getVelocity();
    double target_pos=m_multiplier.at(iax)*m_target_position+m_offset.at(iax);
    double target_vel=m_multiplier.at(iax)*m_target_velocity;
    double target_eff=m_multiplier.at(iax)*m_target_effort;

    double pos_error=target_pos-position;
    double vel_error=target_vel-velocity;

    double xi=m_xi.at(iax)+m_ki.at(iax)*period.toSec()*pos_error;
    double effort=xi+m_kp.at(iax)*pos_error+m_kv.at(iax)*vel_error+target_eff;
    if (effort>m_max_effort.at(iax))
      effort=m_max_effort.at(iax);
    else if (effort<-m_max_effort.at(iax))
      effort=-m_max_effort.at(iax);
    else
      m_xi.at(iax)=xi;

    m_joint_handles.at(iax).setCommand(effort);
  }
}


void MimicController::setTargetCallback(const sensor_msgs::JointStateConstPtr& msg)
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
    m_target_position = tmp_msg.position.at(0);
    m_target_velocity = tmp_msg.velocity.at(0);
    m_target_effort   = tmp_msg.effort.at(0);

  }
  catch(...)
  {
    ROS_ERROR("something wrong in target callback");
    m_configured=false;
  }
}
}  //  namespace robot_control
