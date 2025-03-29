//
// Created by cch on 24-5-31.
//
#include "rm_manual/engineer2_manual.h"

namespace rm_manual
{
Engineer2Manual::Engineer2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalManual(nh, nh_referee)
  , operating_mode_(MANUAL)
  , action_client_("/engineer_middleware/move_steps", true)
{
  engineer_ui_pub_ = nh.advertise<rm_msgs::EngineerUi>("/engineer_ui", 10);
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  stone_num_sub_ = nh.subscribe<std_msgs::String>("/stone_num", 10, &Engineer2Manual::stoneNumCallback, this);
  gripper_state_sub_ = nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 10,
                                                       &Engineer2Manual::gpioStateCallback, this);
  gripper_ui_pub_ = nh.advertise<rm_msgs::VisualizeStateData>("/visualize_state", 10);
  // custom_controller_sub_ = nh.subscribe<std_msgs::Float64MultiArray>("/rm_referee/custom_controller_data", 10, &Engineer2Manual::customControllerCallback,this);
  custom_controller_sub_ = nh.subscribe<std_msgs::Float64MultiArray>("/rm_vt/custom_controller_data", 10, &Engineer2Manual::customControllerCallback,this);
  trajectory_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/controllers/arm_trajectory_controller/command", 10);
  // Servo
  ros::NodeHandle nh_servo(nh, "servo");
  servo_command_sender_ = new rm_common::Vel3DCommandSender(nh_servo);
  servo_reset_caller_ = new rm_common::ServiceCallerBase<std_srvs::Empty>(nh_servo, "/servo_server/reset_servo_status");
  // Custom Controller
  //  data 1-5 correspond to joint 2-6
  //  data 6 corresponds to joint 1
  ros::NodeHandle nh_custom_data_offset(nh, "custom_data_offset");
  nh_custom_data_offset.param("data1_offset", custom_data_offset_[0], 0.0);
  nh_custom_data_offset.param("data2_offset", custom_data_offset_[1], 0.0);
  nh_custom_data_offset.param("data3_offset", custom_data_offset_[2], 0.0);
  nh_custom_data_offset.param("data4_offset", custom_data_offset_[3], 0.0);
  nh_custom_data_offset.param("data5_offset", custom_data_offset_[4], 0.0);
  nh_custom_data_offset.param("data6_offset", custom_data_offset_[5], 0.0);
  ros::NodeHandle nh_custom_controller_offset(nh, "custom_controller_offset");
  nh_custom_controller_offset.param("joint2", custom_controller_offset_[0], 0.0);
  nh_custom_controller_offset.param("joint3", custom_controller_offset_[1], 0.0);
  nh_custom_controller_offset.param("joint4", custom_controller_offset_[2], 0.0);
  nh_custom_controller_offset.param("joint5", custom_controller_offset_[3], 0.0);
  nh_custom_controller_offset.param("joint6", custom_controller_offset_[4], 0.0);
  ros::NodeHandle nh_custom_controller_joint1(nh, "custom_controller_joint1");
  nh_custom_controller_joint1.param("custom_dead_zone", custom_dead_zone_, 0.08);
  nh_custom_controller_joint1.param("custom_speed_scale", custom_speed_scale_, 0.1);
  // Vel
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_nh.param("fast_speed_scale", fast_speed_scale_, 1.0);
  chassis_nh.param("normal_speed_scale", normal_speed_scale_, 0.5);
  chassis_nh.param("low_speed_scale", low_speed_scale_, 0.1);
  chassis_nh.param("exchange_speed_scale", exchange_speed_scale_, 0.2);
  chassis_nh.param("big_island_speed_scale", big_island_speed_scale_, 0.02);
  chassis_nh.param("fast_gyro_scale", fast_gyro_scale_, 0.5);
  chassis_nh.param("normal_gyro_scale", normal_gyro_scale_, 0.15);
  chassis_nh.param("low_gyro_scale", low_gyro_scale_, 0.05);
  chassis_nh.param("exchange_gyro_scale", exchange_gyro_scale_, 0.12);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&Engineer2Manual::leftSwitchUpFall, this));
  left_switch_up_event_.setRising(boost::bind(&Engineer2Manual::leftSwitchUpRise, this));
  left_switch_down_event_.setFalling(boost::bind(&Engineer2Manual::leftSwitchDownFall, this));
  left_switch_down_event_.setRising(boost::bind(&Engineer2Manual::leftSwitchDownRise, this));
  ctrl_a_event_.setRising(boost::bind(&Engineer2Manual::ctrlAPress, this));
  ctrl_b_event_.setRising(boost::bind(&Engineer2Manual::ctrlBPress, this));
  ctrl_c_event_.setRising(boost::bind(&Engineer2Manual::ctrlCPress, this));
  ctrl_b_event_.setActiveHigh(boost::bind(&Engineer2Manual::ctrlBPressing, this));
  ctrl_b_event_.setFalling(boost::bind(&Engineer2Manual::ctrlBRelease, this));
  ctrl_d_event_.setRising(boost::bind(&Engineer2Manual::ctrlDPress, this));
  ctrl_e_event_.setRising(boost::bind(&Engineer2Manual::ctrlEPress, this));
  ctrl_f_event_.setRising(boost::bind(&Engineer2Manual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&Engineer2Manual::ctrlGPress, this));
  ctrl_q_event_.setRising(boost::bind(&Engineer2Manual::ctrlQPress, this));
  ctrl_r_event_.setRising(boost::bind(&Engineer2Manual::ctrlRPress, this));
  ctrl_s_event_.setRising(boost::bind(&Engineer2Manual::ctrlSPress, this));
  ctrl_v_event_.setRising(boost::bind(&Engineer2Manual::ctrlVPress, this));
  ctrl_v_event_.setFalling(boost::bind(&Engineer2Manual::ctrlVRelease, this));
  ctrl_w_event_.setRising(boost::bind(&Engineer2Manual::ctrlWPress, this));
  ctrl_x_event_.setRising(boost::bind(&Engineer2Manual::ctrlXPress, this));
  ctrl_z_event_.setRising(boost::bind(&Engineer2Manual::ctrlZPress, this));
  b_event_.setActiveHigh(boost::bind(&Engineer2Manual::bPressing, this));
  b_event_.setFalling(boost::bind(&Engineer2Manual::bRelease, this));
  c_event_.setActiveHigh(boost::bind(&Engineer2Manual::cPressing, this));
  c_event_.setFalling(boost::bind(&Engineer2Manual::cRelease, this));
  e_event_.setActiveHigh(boost::bind(&Engineer2Manual::ePressing, this));
  e_event_.setFalling(boost::bind(&Engineer2Manual::eRelease, this));
  f_event_.setRising(boost::bind(&Engineer2Manual::fPress, this));
  f_event_.setFalling(boost::bind(&Engineer2Manual::fRelease, this));
  g_event_.setRising(boost::bind(&Engineer2Manual::gPress, this));
  g_event_.setFalling(boost::bind(&Engineer2Manual::gRelease, this));
  q_event_.setActiveHigh(boost::bind(&Engineer2Manual::qPressing, this));
  q_event_.setFalling(boost::bind(&Engineer2Manual::qRelease, this));
  r_event_.setRising(boost::bind(&Engineer2Manual::rPress, this));
  r_event_.setFalling(boost::bind(&Engineer2Manual::rRelease, this));
  v_event_.setActiveHigh(boost::bind(&Engineer2Manual::vPressing, this));
  v_event_.setFalling(boost::bind(&Engineer2Manual::vRelease, this));
  x_event_.setRising(boost::bind(&Engineer2Manual::xPress, this));
  z_event_.setActiveHigh(boost::bind(&Engineer2Manual::zPressing, this));
  z_event_.setFalling(boost::bind(&Engineer2Manual::zRelease, this));
  shift_event_.setActiveHigh(boost::bind(&Engineer2Manual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&Engineer2Manual::shiftRelease, this));
  shift_b_event_.setRising(boost::bind(&Engineer2Manual::shiftBPress, this));
  shift_b_event_.setFalling(boost::bind(&Engineer2Manual::shiftBRelease, this));
  shift_c_event_.setRising(boost::bind(&Engineer2Manual::shiftCPress, this));
  shift_e_event_.setRising(boost::bind(&Engineer2Manual::shiftEPress, this));
  shift_f_event_.setRising(boost::bind(&Engineer2Manual::shiftFPress, this));
  shift_g_event_.setRising(boost::bind(&Engineer2Manual::shiftGPress, this));
  shift_q_event_.setRising(boost::bind(&Engineer2Manual::shiftQPress, this));
  shift_r_event_.setRising(boost::bind(&Engineer2Manual::shiftRPress, this));
  shift_r_event_.setFalling(boost::bind(&Engineer2Manual::shiftRRelease, this));
  shift_v_event_.setRising(boost::bind(&Engineer2Manual::shiftVPress, this));
  shift_v_event_.setFalling(boost::bind(&Engineer2Manual::shiftVRelease, this));
  shift_x_event_.setRising(boost::bind(&Engineer2Manual::shiftXPress, this));
  shift_z_event_.setRising(boost::bind(&Engineer2Manual::shiftZPress, this));
  shift_z_event_.setFalling(boost::bind(&Engineer2Manual::shiftZRelease, this));

  mouse_left_event_.setFalling(boost::bind(&Engineer2Manual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&Engineer2Manual::mouseRightRelease, this));
}

void Engineer2Manual::run()
{
  ChassisGimbalManual::run();
  if( servo_mode_ == CUSTOM )
    updateCustomController();
  calibration_gather_->update(ros::Time::now());
  engineer_ui_pub_.publish(engineer_ui_);
  gripper_ui_pub_.publish(gripper_ui_);
}

void Engineer2Manual::changeSpeedMode(SpeedMode speed_mode)
{
  switch (speed_mode)
  {
    case LOW:
      speed_change_scale_ = low_speed_scale_;
      gyro_scale_ = low_gyro_scale_;
      break;
    case NORMAL:
      speed_change_scale_ = normal_speed_scale_;
      gyro_scale_ = normal_gyro_scale_;
      break;
    case FAST:
      speed_change_scale_ = fast_speed_scale_;
      gyro_scale_ = fast_gyro_scale_;
      break;
    case EXCHANGE:
      speed_change_scale_ = exchange_speed_scale_;
      gyro_scale_ = exchange_gyro_scale_;
      break;
    case BIG_ISLAND_SPEED:
      speed_change_scale_ = big_island_speed_scale_;
      gyro_scale_ = exchange_gyro_scale_;
    default:
      speed_change_scale_ = normal_speed_scale_;
      gyro_scale_ = normal_gyro_scale_;
      break;
  }
}

void Engineer2Manual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  ctrl_a_event_.update(dbus_data->key_ctrl & dbus_data->key_a);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_d_event_.update(dbus_data->key_ctrl & dbus_data->key_d);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);
  ctrl_f_event_.update(dbus_data->key_ctrl & dbus_data->key_f);
  ctrl_g_event_.update(dbus_data->key_ctrl & dbus_data->key_g);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_s_event_.update(dbus_data->key_ctrl & dbus_data->key_s);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_w_event_.update(dbus_data->key_ctrl & dbus_data->key_w);
  ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);

  b_event_.update(dbus_data->key_b & !dbus_data->key_ctrl & !dbus_data->key_shift);
  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
  e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl & !dbus_data->key_shift);
  f_event_.update(dbus_data->key_f & !dbus_data->key_ctrl & !dbus_data->key_shift);
  g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl & !dbus_data->key_shift);
  q_event_.update(dbus_data->key_q & !dbus_data->key_ctrl & !dbus_data->key_shift);
  r_event_.update(dbus_data->key_r & !dbus_data->key_ctrl & !dbus_data->key_shift);
  v_event_.update(dbus_data->key_v & !dbus_data->key_ctrl & !dbus_data->key_shift);
  x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl & !dbus_data->key_shift);
  z_event_.update(dbus_data->key_z & !dbus_data->key_ctrl & !dbus_data->key_shift);

  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
  shift_b_event_.update(dbus_data->key_shift & dbus_data->key_b);
  shift_c_event_.update(dbus_data->key_shift & dbus_data->key_c);
  shift_e_event_.update(dbus_data->key_shift & dbus_data->key_e);
  shift_f_event_.update(dbus_data->key_shift & dbus_data->key_f);
  shift_g_event_.update(dbus_data->key_shift & dbus_data->key_g);
  shift_q_event_.update(dbus_data->key_shift & dbus_data->key_q);
  shift_r_event_.update(dbus_data->key_shift & dbus_data->key_r);
  shift_v_event_.update(dbus_data->key_shift & dbus_data->key_v);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x);
  shift_z_event_.update(dbus_data->key_shift & dbus_data->key_z);

  mouse_left_event_.update(dbus_data->p_l);
  mouse_right_event_.update(dbus_data->p_r);

  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
}

void Engineer2Manual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  vel_cmd_sender_->setAngularZVel(dbus_data->wheel);
  vel_cmd_sender_->setLinearXVel(dbus_data->ch_r_y);
  vel_cmd_sender_->setLinearYVel(-dbus_data->ch_r_x);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(dbus_data->s_l == rm_msgs::DbusData::DOWN);
}

void Engineer2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  checkKeyboard(dbus_data);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  if (servo_mode_ == CUSTOM)
    chassis_cmd_sender_->getMsg()->command_source_frame = "exchange_orientation";
  else
    chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  if (servo_mode_ == JOINT)
    vel_cmd_sender_->setAngularZVel(-dbus_data->m_x * gimbal_scale_);
}

void Engineer2Manual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  servo_command_sender_->setLinearVel(dbus_data->wheel, dbus_data->ch_l_x, -dbus_data->ch_l_y);
  servo_command_sender_->setAngularVel(-angular_z_scale_, -dbus_data->ch_r_y, dbus_data->ch_r_x);
}

void Engineer2Manual::updateCustomController()
{
  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.header.seq = custom_seq_;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.joint_names = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
  joint_trajectory_point.positions.push_back( joint1_state_ ) ;
  joint_trajectory_point.time_from_start = ros::Duration(0.25);
  for( double i : custom_joint_state_ )
  {
    joint_trajectory_point.positions.push_back( i );
  }
  joint_trajectory.points.push_back( joint_trajectory_point );
  trajectory_cmd_pub_.publish( joint_trajectory );
  custom_seq_++;
  if( custom_seq_ > 100000 )
    custom_seq_ = 1;
}

void Engineer2Manual::updateJoint1(const rm_msgs::DbusData::ConstPtr &dubs_data)
{
  joint1_state_ = joint_state_.position[0];
  joint1_state_ += (dubs_data->ch_l_y) * rc_speed_scale_;
}

void Engineer2Manual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  if (servo_mode_ == SERVO)
    updateServo(data);
  if (servo_mode_ == CUSTOM)
    updateJoint1(data);
}

void Engineer2Manual::stoneNumCallback(const std_msgs::String::ConstPtr& data)
{
  auto it = stoneNumMap_.find(data->data);
  if (it != stoneNumMap_.end())
    stone_state_[it->second] = (data->data[0] == '+');
}

void Engineer2Manual::gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data)
{
  gripper_ui_.state = { data->gpio_state.begin(), data->gpio_state.end() - 2 };
  main_gripper_on_ = data->gpio_state[0];
}

void Engineer2Manual::customControllerCallback(const std_msgs::Float64MultiArray::ConstPtr& custom_data)
{
  double m_data = 0.0;
  for( int i = 0; i < 5; i++ )
  {
    m_data = custom_data->data[i] + custom_data_offset_[i];
    if( m_data >= 6.28 )
      m_data -= 6.28;
    if( m_data < 0)
      m_data += 6.28;
    m_data *= custom_joints_orientation_[i];
    m_data += custom_controller_offset_[i];
    custom_joint_state_[i] = m_data;
  }
  m_data = custom_data->data[5] + custom_data_offset_[5];
  if( m_data > custom_dead_zone_ && m_data < -custom_dead_zone_ ){
    joint1_state_ = joint_state_.position[0];
    joint1_state_ += m_data * custom_speed_scale_;
  }
}

void Engineer2Manual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendChassisCommand(time, false);
    vel_cmd_sender_->sendCommand(time);
  }
  if (servo_mode_ == SERVO)
  {
    changeSpeedMode(EXCHANGE);
    servo_command_sender_->sendCommand(time);
    if (gimbal_mode_ == RATE)
      gimbal_cmd_sender_->sendCommand(time);
  }
  if (servo_mode_ == CUSTOM)
  {
    changeSpeedMode(EXCHANGE);
    if (gimbal_mode_ == RATE)
      gimbal_cmd_sender_->sendCommand(time);
  }
}

void Engineer2Manual::runStepQueue(const std::string& step_queue_name)
{
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = step_queue_name;
  if (action_client_.isServerConnected())
  {
    if (operating_mode_ == MANUAL)
      action_client_.sendGoal(goal, boost::bind(&Engineer2Manual::actionDoneCallback, this, _1, _2),
                              boost::bind(&Engineer2Manual::actionActiveCallback, this),
                              boost::bind(&Engineer2Manual::actionFeedbackCallback, this, _1));
    operating_mode_ = MIDDLEWARE;
  }
  else
    ROS_ERROR("Can not connect to middleware");
}

void Engineer2Manual::actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback)
{
}

void Engineer2Manual::actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                         const rm_msgs::EngineerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  ROS_INFO("Done %s", (prefix_ + root_).c_str());
  mouse_left_pressed_ = true;
  mouse_right_pressed_ = true;
  ROS_INFO("%i", result->finish);
  operating_mode_ = MANUAL;
  if (root_ == "HOME")
  {
    initMode();
    changeSpeedMode(NORMAL);
  }
  // if (root_ == "BIG_ISLAND0" || root_ == "THREE_SILVER0")
  //   engineer_ui_.symbol = UiState::NONE;
  // if (prefix_ == "LV4_" || prefix_ == "LV5_L_" || prefix_ == "LV5_R_")
  // {
  //   had_ground_stone_ = false;
  //   had_side_gold_ = false;
  //   changeSpeedMode(EXCHANGE);
  //   enterServo();
  // }
  // if (root_ == "GROUND_STONE0")
  // {
  //   changeSpeedMode(NORMAL);
  // }
  // if (prefix_ + root_ == "SIDE_BIG_ISLAND1")
  // {
  //   enterServo();
  //   changeSpeedMode(BIG_ISLAND_SPEED);
  // }
  // if (prefix_ + root_ == "MID_BIG_ISLAND11" || prefix_ + root_ == "BOTH_BIG_ISLAND1")
  // {
  //   enterServo();
  //   changeSpeedMode(BIG_ISLAND_SPEED);
  // }
  // if (prefix_ + root_ == "MID_BIG_ISLAND111" || prefix_ + root_ == "SIDE_BIG_ISLAND111" ||
  //     prefix_ + root_ == "BOTH_BIG_ISLAND111")
  // {
  //   initMode();
  //   changeSpeedMode(NORMAL);
  // }
}

void Engineer2Manual::enterServo()
{
  servo_mode_ = SERVO;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  servo_reset_caller_->callService();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  engineer_ui_.control_mode = "SERVO";
}

void Engineer2Manual::enterCustom()
{
  servo_mode_ = CUSTOM;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "exchange_orientation";
//  engineer_ui_.control_mode = "CUSTOM";
}

void Engineer2Manual::initMode()
{
  servo_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  engineer_ui_.control_mode = "NORMAL";
}

void Engineer2Manual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void Engineer2Manual::gimbalOutputOn()
{
  ChassisGimbalManual::gimbalOutputOn();
}

void Engineer2Manual::chassisOutputOn()
{
  if (operating_mode_ == MIDDLEWARE)
    action_client_.cancelAllGoals();
}

//-------------------controller input-------------------
void Engineer2Manual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  servo_mode_ = JOINT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void Engineer2Manual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = RATE;
  gimbal_cmd_sender_->setZero();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void Engineer2Manual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
//  servo_mode_ = SERVO;
  servo_mode_ = CUSTOM;
  gimbal_mode_ = RATE;
//  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
//  ROS_INFO_STREAM("servo_mode");
  ROS_INFO_STREAM("custom_controller_mode");
}

void Engineer2Manual::leftSwitchUpRise()
{
  prefix_ = "";
  root_ = "CALIBRATION";
  runStepQueue("CALIBRATION");
  // had_side_gold_ = false;
  // had_ground_stone_ = false;
  calibration_gather_->reset();
  for (auto& stone : stone_state_)
  {
    stone = false;
  }
  engineer_ui_.control_mode = "NORMAL";
  ROS_INFO_STREAM("START CALIBRATE");
}
void Engineer2Manual::leftSwitchUpFall()
{
  runStepQueue("HOME_WITH_PITCH");
}

void Engineer2Manual::leftSwitchDownRise()
{
  if (!main_gripper_on_)
  {
    runStepQueue("OA");
  }
  else
  {
    runStepQueue("CA");
  }
}
void Engineer2Manual::leftSwitchDownFall()
{
  runStepQueue("MIDDLE_PITCH_UP");
}

//--------------------- keyboard input ------------------------
// mouse input
void Engineer2Manual::mouseLeftRelease()
{
  if (mouse_left_pressed_)
  {
    root_ += "0";
    mouse_left_pressed_ = false;
    runStepQueue(prefix_ + root_);
    ROS_INFO("Finished %s", (prefix_ + root_).c_str());
  }
}

void Engineer2Manual::mouseRightRelease()
{
  runStepQueue(prefix_ + root_);
  ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}

void Engineer2Manual::bPressing()
{
}

void Engineer2Manual::bRelease()
{
}
void Engineer2Manual::cPressing()
{
  // angular_z_scale_ = -0.8;
}
void Engineer2Manual::cRelease()
{
  // angular_z_scale_ = 0.;
}
void Engineer2Manual::ePressing()
{
  if (servo_mode_ == SERVO || servo_mode_ == CUSTOM)
    vel_cmd_sender_->setAngularZVel(-gyro_scale_);
}
void Engineer2Manual::eRelease()
{
  if (servo_mode_ == SERVO || servo_mode_ == CUSTOM)
    vel_cmd_sender_->setAngularZVel(0.);
}
void Engineer2Manual::fPress()
{
}
void Engineer2Manual::fRelease()
{
}
void Engineer2Manual::gPress()
{
}
void Engineer2Manual::gRelease()
{
}
void Engineer2Manual::qPressing()
{
  if (servo_mode_ == SERVO || servo_mode_ == CUSTOM)
    vel_cmd_sender_->setAngularZVel(gyro_scale_);
}
void Engineer2Manual::qRelease()
{
  if (servo_mode_ == SERVO || servo_mode_ == CUSTOM)
    vel_cmd_sender_->setAngularZVel(0.);
}
void Engineer2Manual::rPress()
{
  // if (had_side_gold_)
  //   had_side_gold_ = false;
  // if (had_ground_stone_)
  //   had_ground_stone_ = false;
  // else if (stone_state_[0])
  //   stone_state_[0] = false;
  // else if (stone_state_[3])
  //   stone_state_[3] = false;
  // else if (stone_state_[2])
  //   stone_state_[2] = false;
  // else if (stone_state_[1])
  //   stone_state_[1] = false;
}

void Engineer2Manual::rRelease()
{
}

void Engineer2Manual::vPressing()
{
}
void Engineer2Manual::vRelease()
{
}
void Engineer2Manual::xPress()
{
  if (servo_mode_ == SERVO || servo_mode_ == CUSTOM)
  {
    switch (gimbal_direction_)
    {
      case 0:
        runStepQueue("GIMBAL_EE");
        gimbal_direction_ = 1;
        break;
      case 1:
        runStepQueue("GIMBAL_R");
        gimbal_direction_ = 0;
        break;
      default:
        gimbal_direction_ = 0;
        break;
    }
  }
  else
  {
    switch (gimbal_direction_)
    {
      case 0:
        runStepQueue("GIMBAL_R");
        gimbal_direction_ = 1;
        break;
      case 1:
        runStepQueue("GIMBAL_B");
        gimbal_direction_ = 2;
        break;
      case 2:
        runStepQueue("GIMBAL_F");
        gimbal_direction_ = 0;
        break;
      default:
        gimbal_direction_ = 0;
        break;
    }
  }
}
void Engineer2Manual::zPressing()
{
  // angular_z_scale_ = 0.8;
}
void Engineer2Manual::zRelease()
{
  // angular_z_scale_ = 0.;
}

//---------------------  CTRL  ---------------------
void Engineer2Manual::ctrlAPress()
{
  prefix_ = "";
  root_ = "GET_SMALL_ISLAND";
  runStepQueue(prefix_ + root_);
  changeSpeedMode(LOW);
  ROS_INFO("%s", (prefix_ + root_).c_str());
  small_arm_stone_ = SILVER;
}
void Engineer2Manual::ctrlBPress()
{
  prefix_ = "";
  root_ = "HOME";
  gimbal_direction_ = 0;
  runStepQueue(prefix_ + root_);
  changeSpeedMode(NORMAL);
}
void Engineer2Manual::ctrlBPressing()
{
}
void Engineer2Manual::ctrlBRelease()
{
}
void Engineer2Manual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  changeSpeedMode(NORMAL);
  initMode();
  ROS_INFO("cancel all goal");
}
void Engineer2Manual::ctrlDPress()
{
  // engineer_ui_.symbol = UiState::SMALL_ISLAND;
  // prefix_ = "GRIPPER_";
  // root_ = "THREE_SILVER";
  // changeSpeedMode(EXCHANGE);
  // runStepQueue(prefix_ + root_);
  // engineer_ui_.symbol = UiState::BIG_ISLAND;
  prefix_ = "";
  root_ = "GET_BIG_ISLAND";
  // had_side_gold_ = true;
  // had_ground_stone_ = false;
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
  small_arm_stone_ = GOLD;
}
void Engineer2Manual::ctrlEPress()
{
  prefix_ = "";
  root_ = "GET_BIG_ISLAND_RIGHT";
  // had_side_gold_ = true;
  // had_ground_stone_ = false;
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlFPress()
{
  // if (exchange_direction_ == "left")
  //   prefix_ = "LV5_L_";
  // else
  //   prefix_ = "LV5_R_";
  // if (exchange_arm_position_ == "normal")
  //   root_ = "EXCHANGE";
  // else
  //   root_ = "DROP_GOLD_EXCHANGE";
  prefix_ = "READY_CUSTOM_";
  root_ = "EXCHANGE";
  runStepQueue(prefix_ + root_);
}
void Engineer2Manual::ctrlGPress()
{
  // engineer_ui_.symbol = UiState::BIG_ISLAND;
  // prefix_ = "MID_";
  // root_ = "BIG_ISLAND";
  // changeSpeedMode(LOW);
  // runStepQueue(prefix_ + root_);
  prefix_ = "";
  if (main_gripper_on_)
  {
    if (servo_mode_ == CUSTOM)
    {
      root_ = "CM_GIMBAL_R";
      gimbal_direction_ = 0;
    }
    else
      root_ = "CM";
  }
  else
  {
    root_ = "OM";
  }
  runStepQueue(root_);
}
void Engineer2Manual::ctrlQPress()
{
  prefix_ = "";
  root_ = "GET_BIG_ISLAND_LEFT";
  // had_side_gold_ = true;
  // had_ground_stone_ = false;
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlRPress()
{
  runStepQueue("CALIBRATION");
  prefix_ = "";
  root_ = "CALIBRATION";
  servo_mode_ = JOINT;
  // had_ground_stone_ = false;
  // had_side_gold_ = false;
  calibration_gather_->reset();
  ROS_INFO_STREAM("START CALIBRATE");
  changeSpeedMode(NORMAL);
  // for (auto& stone : stone_state_)
  //   stone = false;
  small_arm_stone_ = EMPTY;
  runStepQueue("CA");
}
void Engineer2Manual::ctrlSPress()
{
  // engineer_ui_.symbol = UiState::BIG_ISLAND;
  // prefix_ = "";
  // root_ = "GET_BIG_ISLAND";
  // // had_side_gold_ = true;
  // // had_ground_stone_ = false;
  // changeSpeedMode(LOW);
  // runStepQueue(prefix_ + root_);
  // ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlVPress()
{
  prefix_ = "GET_SMALL_ARM_STONE_FROM_";
  switch (small_arm_stone_)
  {
    case EMPTY:
      ROS_INFO_STREAM("SMALL_ARM_HAS_NO_STONE");
      prefix_ = "";
      root_ = "";
      break;
    case GOLD:
      root_ = "UP";
      break;
    case SILVER:
      root_ = "SIDE";
      break;
  }
  runStepQueue(prefix_ + root_);
  small_arm_stone_ = EMPTY;
}
void Engineer2Manual::ctrlVRelease()
{
}
void Engineer2Manual::ctrlWPress()
{
  // // engineer_ui_.symbol = UiState::BIG_ISLAND;
  // prefix_ = "";
  // root_ = "GET_BIG_ISLAND_RIGHT";
  // // had_side_gold_ = true;
  // // had_ground_stone_ = false;
  // changeSpeedMode(LOW);
  // runStepQueue(prefix_ + root_);
  // ROS_INFO("%s", (prefix_ + root_).c_str());
  // engineer_ui_.symbol = UiState::BIG_ISLAND;
  prefix_ = "";
  root_ = "GET_BIG_ISLAND_MID";
  // had_side_gold_ = true;
  // had_ground_stone_ = false;
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlXPress()
{
  // had_ground_stone_ = true;
  // prefix_ = "";
  // root_ = "GROUND_STONE";
  // changeSpeedMode(LOW);
  // ROS_INFO_STREAM(prefix_ + root_);
  // runStepQueue(prefix_ + root_);
}
void Engineer2Manual::ctrlZPress()
{
}

//---------------  SHIFT  --------------------------

void Engineer2Manual::shiftPressing()
{
  changeSpeedMode(FAST);
}
void Engineer2Manual::shiftRelease()
{
  changeSpeedMode(NORMAL);
}
void Engineer2Manual::shiftBPress()
{
}
void Engineer2Manual::shiftBRelease()
{
}
void Engineer2Manual::shiftCPress()
{
  action_client_.cancelAllGoals();
  // if (servo_mode_ == SERVO)
  // {
  //   initMode();
  //   ROS_INFO("EXIT SERVO");
  // }
  // else
  // {
  //   enterServo();
  //   ROS_INFO("ENTER SERVO");
  // }
  if (servo_mode_ == CUSTOM)
  {
    initMode();
    ROS_INFO("EXIT CUSTOM CONTROLLER");
  }
  else
  {
    enterCustom();
    ROS_INFO("ENTER CUSTOM CONTROLLER");
  }
  ROS_INFO("cancel all goal");
}
void Engineer2Manual::shiftEPress()
{
  // exchange_direction_ = "right";
  // prefix_ = "LV5_R_";
  // if (had_ground_stone_)
  // {
  //   exchange_arm_position_ = "normal";
  //   root_ = "EXCHANGE";
  // }
  // else if (had_side_gold_)
  // {
  //   exchange_arm_position_ = "front";
  //   root_ = "DROP_GOLD_EXCHANGE";
  // }
  // else
  // {
  //   exchange_arm_position_ = "normal";
  //   if (stone_state_[0])
  //     root_ = "G";
  //   else if (stone_state_[3])
  //     root_ = "S3";
  //   else if (stone_state_[2])
  //     root_ = "S2";
  //   else if (stone_state_[1])
  //     root_ = "S1";
  // }
  // runStepQueue(prefix_ + root_);
  // engineer_ui_.symbol = UiState::BIG_ISLAND;
}
void Engineer2Manual::shiftFPress()
{
  // if (exchange_direction_ == "left")
  // {
  //   prefix_ = "L2R_";
  //   exchange_direction_ = "right";
  // }
  // else if (exchange_direction_ == "right")
  // {
  //   prefix_ = "R2L_";
  //   exchange_direction_ = "left";
  // }
  // if (exchange_arm_position_ == "normal")
  //   root_ = "EXCHANGE";
  // else
  //   root_ = "EXCHANGE";
  // ROS_INFO_STREAM(prefix_ + root_);
  // runStepQueue(prefix_ + root_);
}
void Engineer2Manual::shiftGPress()
{
  // prefix_ = "LV4_";
  // if (had_ground_stone_)
  // {
  //   exchange_arm_position_ = "normal";
  //   root_ = "EXCHANGE";
  // }
  // else if (had_side_gold_)
  // {
  //   exchange_arm_position_ = "front";
  //   root_ = "EXCHANGE";
  // }
  // else
  // {
  //   exchange_arm_position_ = "normal";
  //   if (stone_state_[0])
  //     root_ = "G";
  //   else if (stone_state_[3])
  //     root_ = "S3";
  //   else if (stone_state_[2])
  //     root_ = "S2";
  //   else if (stone_state_[1])
  //     root_ = "S1";
  // }
  // runStepQueue(prefix_ + root_);
  // ROS_INFO_STREAM(prefix_ + root_);
}
void Engineer2Manual::shiftQPress()
{
  // exchange_direction_ = "left";
  // prefix_ = "LV5_L_";
  // if (had_ground_stone_)
  // {
  //   exchange_arm_position_ = "normal";
  //   root_ = "EXCHANGE";
  // }
  // else if (had_side_gold_)
  // {
  //   exchange_arm_position_ = "front";
  //   root_ = "DROP_GOLD_EXCHANGE";
  // }
  // else
  // {
  //   exchange_arm_position_ = "normal";
  //   if (stone_state_[0])
  //     root_ = "G";
  //   else if (stone_state_[3])
  //     root_ = "S3";
  //   else if (stone_state_[2])
  //     root_ = "S2";
  //   else if (stone_state_[1])
  //     root_ = "S1";
  // }
  // runStepQueue(prefix_ + root_);
  // ROS_INFO_STREAM(prefix_ + root_);
  // engineer_ui_.symbol = UiState::BIG_ISLAND;
}
void Engineer2Manual::shiftRPress()
{
}
void Engineer2Manual::shiftRRelease()
{
}
void Engineer2Manual::shiftVPress()
{
  // prefix_ = "";
  // if (main_gripper_on_)
  // {
  //   root_ = "CM";
  // }
  // else
  // {
  //   root_ = "OM";
  // }
  // runStepQueue(root_);
}
void Engineer2Manual::shiftVRelease()
{
}
void Engineer2Manual::shiftXPress()
{
}
void Engineer2Manual::shiftZPress()
{
  // prefix_ = "";
  // root_ = "CG";
  // runStepQueue(prefix_ + root_);
  // ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::shiftZRelease()
{
}

}  // namespace rm_manual
