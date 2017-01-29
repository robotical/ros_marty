/**
 * @file      marty_core.cpp
 * @brief     Marty Core header providing access to Marty methods
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ros_marty/marty_core.hpp>

MartyCore::MartyCore(ros::NodeHandle& nh) : nh_(nh) {
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_INFO("MartyCore Ready!");
}

MartyCore::~MartyCore() {
  ros::param::del("/marty");
}

void MartyCore::loadParams() {
  ros::param::param("/marty/calibrated", calibrated_, false);
  if (!calibrated_) {
    ERR("Marty has not been calibrated before!\n");
    WARN("Please use 'roslaunch ros_marty calibration.launch' to calibrate\n");
    ros::param::del("/marty");
    ros::shutdown();
    exit(0);
  }
  for (int id = 0; id < NUMJOINTS; ++id) {
    std::string zero_p = "/marty/" + NAMES[id] + "/zero";
    std::string max_p = "/marty/" + NAMES[id] + "/max";
    std::string min_p = "/marty/" + NAMES[id] + "/min";
    std::string dir_p = "/marty/" + NAMES[id] + "/dir";
    std::string mult_p = "/marty/" + NAMES[id] + "/mult";
    ros::param::get(zero_p, joint_[id].cmdZero);
    ros::param::get(max_p, joint_[id].cmdMax);
    ros::param::get(min_p, joint_[id].cmdMin);
    ros::param::get(dir_p, joint_[id].cmdDir);
    ros::param::get(mult_p, joint_[id].cmdMult);
  }

  ros::param::param("/marty/check_fall", fall_disable_, true);
  ros::param::param("/marty/fall_threshold", acc_thr_, 0.9);
  ros::param::param("/marty/camera_ori", camera_ori_, 15.0);

  marty_msgs::ServoMsg joint;
  for (int id = 0; id < NUMJOINTS; ++id) {
    joint.servo_id = id;
    joint.servo_cmd = joint_[id].cmdZero;
    // servo_msg_array_.servo_msg.push_back(joint);
  }
}

void MartyCore::init() {
  falling_.data = false;
  for (int ji = 0; ji < NUMJOINTS; ji++) { jangles_.push_back(0); }
  // TF
  cam_tf_.header.frame_id = "base_link";
  cam_tf_.child_frame_id = "camera";
  cam_tf_.transform.translation.x = 0.02;
  cam_tf_.transform.translation.y = 0.0;
  cam_tf_.transform.translation.z = 0.01;
  // tf2::Quaternion q(-0.54168, 0.54168, -0.45452, 0.45452);
  // tf2::Quaternion q(-0.596, 0.596, -0.380, 0.380); // 25 deg
  tf2::Quaternion q(-0.627, 0.627, -0.327, 0.327); // 35 deg

  double ori = ((90 - camera_ori_) / 180) * M_PI;
  // ROS_INFO_STREAM("CamOri: " << camera_ori_ << " ORI: " << ori << " PI: " <<
  //                 M_PI);
  // q.setRPY(-M_PI, ori, (M_PI / 2)); // TODO: This is borked
  // ROS_INFO_STREAM("X: " << q.x() << " Y: " << q.y() <<
  //                 " Z: " << q.z() << " W: " << q.w());
  // cam_tf_.transform.rotation.x = q.x();
  // cam_tf_.transform.rotation.y = q.y();
  // cam_tf_.transform.rotation.z = q.z();
  // cam_tf_.transform.rotation.w = q.w();
  q.normalize();
  cam_tf_.transform.rotation.x = q.x();
  cam_tf_.transform.rotation.y = q.y();
  cam_tf_.transform.rotation.z = q.z();
  cam_tf_.transform.rotation.w = q.w();
}

void MartyCore::rosSetup() {
  // PUBLISHERS
  enable_pub_ = nh_.advertise<std_msgs::Bool>("/enable_motors", 10);
  while (enable_pub_.getNumSubscribers() == 0) {
    ROS_INFO("Waiting for rosserial to start...\n");
    sleepms(500);
  }
  falling_pub_ = nh_.advertise<std_msgs::Bool>("/falling", 1, true);
  servo_pub_ = nh_.advertise<marty_msgs::ServoMsg>("/servo", 10);
  servo_array_pub_ = nh_.advertise<marty_msgs::ServoMsgArray>("/servo_array", 10);
  // SUBSCRIBERS
  accel_sub_ = nh_.subscribe("/accel", 1000, &MartyCore::accelCB, this);
  batt_sub_ = nh_.subscribe("/battery", 1000, &MartyCore::battCB, this);
  gpio_sub_ = nh_.subscribe("/gpios", 1000, &MartyCore::gpioCB, this);
  // SERVICES
  fall_dis_srv_ = nh_.advertiseService("/marty/fall_disable",
                                       &MartyCore::setFallDetector, this);
  play_sound_ = nh_.serviceClient<marty_msgs::Sound>("/marty/play_sound");
  // ros::service::waitForService("/marty/set_gpio_config");
  // set_gpio_config_ =
  //   nh_.serviceClient<marty_msgs::GPIOConfig>("/marty/set_gpio_config");
  // ros::service::waitForService("/marty/get_gpio_config");
  // get_gpio_config_ =
  //   nh_.serviceClient<marty_msgs::GPIOConfig>("/marty/get_gpio_config");
  // marty_msgs::GPIOConfig srv;
  // srv.request.config[0] = marty_msgs::GPIOConfig::Request::DIGITAL_IN;
  // if (set_gpio_config_.call(srv)) {
  //   if (srv.response.success) {ROS_INFO("SUCCESS!");} else {ROS_WARN("NAY!");}
  // } else { ROS_ERROR("Failed to call set gpio service!"); }
  tf_timer_ = nh_.createTimer(ros::Duration(0.1), &MartyCore::tfCB, this);
}

bool MartyCore::setFallDetector(std_srvs::SetBool::Request&  req,
                                std_srvs::SetBool::Response& res) {
  fall_disable_ = req.data; res.success = true; return true;
}

void MartyCore::accelCB(const marty_msgs::Accelerometer::ConstPtr& msg) {
  accel_ = *msg;
  if ((accel_.y < acc_thr_) && (falling_.data == false)) {
    ROS_WARN_STREAM("Robot Falling! " << accel_.y << std::endl);
    falling_.data = true;
    if (fall_disable_) {
      enable_robot_.data = false;
      enable_pub_.publish(enable_robot_);
    }
    falling_pub_.publish(falling_);
  }
  if ((accel_.y > acc_thr_) && (falling_.data == true)) {
    ROS_WARN_STREAM("Robot Stable! " << accel_.y << std::endl);
    falling_.data = false;
    if (fall_disable_) {
      enable_robot_.data = true;
      enable_pub_.publish(enable_robot_);
    }
    falling_pub_.publish(falling_);
  }
}

void MartyCore::battCB(const std_msgs::Float32::ConstPtr& msg) {
  battery_val_ = msg->data;
}

void MartyCore::gpioCB(const marty_msgs::GPIOs::ConstPtr& msg) {
  gpios_val_ = *msg;
}

void MartyCore::tfCB(const ros::TimerEvent& e) {
  cam_tf_.header.stamp = ros::Time::now();
  tf_br_.sendTransform(cam_tf_);
}

void MartyCore::readySound() {
  this->playSound(440.0, 0.25);
  this->playSound(880.0, 0.25);
}

void MartyCore::playSound(float frequency, float duration) {
  marty_msgs::Sound sound_srv;
  sound_srv.request.frequency = frequency;
  sound_srv.request.duration = duration;
  play_sound_.call(sound_srv);
}

int MartyCore::jointPosToServoCmd(int id, float pos) {
  float cmd = float(joint_[id].cmdZero) +
              (pos * joint_[id].cmdMult * float(joint_[id].cmdDir));
  cmd = fmin(cmd, joint_[id].cmdMax); cmd = fmax(cmd, joint_[id].cmdMin);
  return int(cmd);
}

void MartyCore::setServoJointPos(std::string name, int pos) {
  try {servo_msg_.servo_id = JOINT_NAMES.at(name);}
  catch (const std::exception& e) {std::cerr << e.what();};
  servo_msg_.servo_cmd = pos;
  servo_pub_.publish(servo_msg_);
}

void MartyCore::setServoPos(int channel, int pos) {
  servo_msg_.servo_id = channel;
  servo_msg_.servo_cmd = pos;
  servo_pub_.publish(servo_msg_);
}

void MartyCore::enableRobot() {
  enable_robot_.data = true;
  enable_pub_.publish(enable_robot_);
}

void MartyCore::stopRobot() {
  enable_robot_.data = false;
  enable_pub_.publish(enable_robot_);
}

bool MartyCore::setServo(int id, float angle) {
  if (id < 0 || id >= NUMJOINTS) { return false; }
  servo_msg_.servo_id = id;
  servo_msg_.servo_cmd = jointPosToServoCmd(id, angle);
  servo_pub_.publish(servo_msg_);
  jangles_[id] = angle;
  return true;
}

void MartyCore::setServos(std::map<int, float> angles) {
  marty_msgs::ServoMsgArray servo_msg_array;
  for (std::map<int, float>::iterator i = angles.begin(); i != angles.end();
       ++i) {
    marty_msgs::ServoMsg servo_msg;
    servo_msg.servo_id = i->first;
    servo_msg.servo_cmd = jointPosToServoCmd(i->first, i->second);
    servo_msg_array.servo_msg.push_back(servo_msg);
    jangles_[i->first] = angles.at(i->first);
  }
  servo_array_pub_.publish(servo_msg_array);
  ros::spinOnce();
}
