/**
 * @file      marty_core.cpp
 * @brief     Marty Core header providing access to Marty methods
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ros_marty/marty_core.hpp>

MartyCore::MartyCore(ros::NodeHandle& nh) : nh_(nh), tf_ls_(tf_buff_) {
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
  odom_setup_ = false;
  for (int ji = 0; ji < NUMJOINTS; ji++) { jangles_.push_back(0); }
  // TF
  cam_tf_.header.frame_id = "base_link";
  cam_tf_.child_frame_id = "camera";
  cam_tf_.transform.translation.x = 0.023;
  cam_tf_.transform.translation.y = 0.0;
  cam_tf_.transform.translation.z = 0.027;
  // tf2::Quaternion q(-0.54168, 0.54168, -0.45452, 0.45452);
  // tf2::Quaternion q(-0.596, 0.596, -0.380, 0.380); // 25 deg
  tf2::Quaternion q(-0.627, 0.627, -0.327, 0.327); // 35 deg

  // double ori = ((90 - camera_ori_) / 180) * M_PI;
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
  this->setupJointStates();
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
  joints_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  // SUBSCRIBERS
  joint_sub_ = nh_.subscribe("/servo", 1000, &MartyCore::jointCB, this);
  joints_sub_ = nh_.subscribe("/servo_array", 1000, &MartyCore::jointsCB, this);
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

void MartyCore::setupJointStates() {
  joints_.name.push_back("face_joint");
  joints_.name.push_back("left_arm_servo_gear_joint");
  joints_.name.push_back("left_arm_gear_joint");
  joints_.name.push_back("right_arm_servo_gear_joint");
  joints_.name.push_back("right_arm_gear_joint");
  joints_.name.push_back("right_eye_joint");
  joints_.name.push_back("left_eye_joint");
  joints_.name.push_back("left_hip_link1_joint");
  joints_.name.push_back("left_hip_link2_joint");
  joints_.name.push_back("left_hip_link3_joint");
  joints_.name.push_back("left_hip_servo_link_joint");
  joints_.name.push_back("left_twist_servo_holder1_joint");
  joints_.name.push_back("left_twist_shaft_joint");
  joints_.name.push_back("left_knee_link1_joint");
  joints_.name.push_back("left_knee_link2_joint");
  joints_.name.push_back("left_knee_link3_joint");
  joints_.name.push_back("left_knee_servo_link_joint");
  joints_.name.push_back("left_foot_joint");
  joints_.name.push_back("right_hip_link1_joint");
  joints_.name.push_back("right_hip_link2_joint");
  joints_.name.push_back("right_hip_link3_joint");
  joints_.name.push_back("right_hip_servo_link_joint");
  joints_.name.push_back("right_twist_servo_holder1_joint");
  joints_.name.push_back("right_twist_shaft_joint");
  joints_.name.push_back("right_knee_link1_joint");
  joints_.name.push_back("right_knee_link2_joint");
  joints_.name.push_back("right_knee_link3_joint");
  joints_.name.push_back("right_knee_servo_link_joint");
  joints_.name.push_back("right_foot_joint");
  for (int j = 0; j < joints_.name.size(); ++j) {
    joints_.position.push_back(0.0);
  }
}

void MartyCore::setupOdometry() {
  try {
    l_foot_tf_ = tf_buff_.lookupTransform("base_link", "left_foot",
                                          ros::Time(0));
    r_foot_tf_ = tf_buff_.lookupTransform("base_link", "right_foot",
                                          ros::Time(0));
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_link";
    odom_tf_.transform.translation.x = 0;
    odom_tf_.transform.translation.y = 0;
    odom_tf_.transform.translation.z = 0;
    odom_tf_.transform.rotation.x = 0;
    odom_tf_.transform.rotation.y = 0;
    odom_tf_.transform.rotation.z = 0;
    odom_tf_.transform.rotation.w = 1;
    odom_setup_ = true;
  } catch (tf2::TransformException& ex) { ROS_WARN("%s", ex.what()); }
}

void MartyCore::jointCB(const marty_msgs::ServoMsg::ConstPtr& msg) {
  this->updateJointState(*msg);
}

void MartyCore::jointsCB(const marty_msgs::ServoMsgArray::ConstPtr& msg) {
  for (int id = 0; id < msg->servo_msg.size(); ++id) {
    this->updateJointState(msg->servo_msg[id]);
  }
}

void MartyCore::updateJointState(marty_msgs::ServoMsg servo) {
  // ROS_INFO_STREAM("ID: " << (int)msg->servo_id << " CMD: " <<
  //                 (int)msg->servo_cmd);
  if (servo.servo_id == 0) {
    joints_.position[10] = ( (float)servo.servo_cmd
                             - joint_[0].cmdZero ) / 123.3;       // LHIP
    joints_.position[7] = joints_.position[10];                   // LHIP1
    joints_.position[8] = joints_.position[10];                   // LHIP2
    joints_.position[9] = joints_.position[10];                   // LHIP3
    joints_.position[11] = -joints_.position[10];                 // LKNEEBox
  } else if (servo.servo_id == 1) {
    joints_.position[12] = -( (float)servo.servo_cmd
                              - joint_[1].cmdZero ) / 123.3;       // LTWIST
  } else if (servo.servo_id == 2) {
    joints_.position[16] = -( (float)servo.servo_cmd
                              - joint_[2].cmdZero ) / 123.3;      // LKNEE
    joints_.position[13] = joints_.position[16];                  // LKNEE1
    joints_.position[14] = joints_.position[16];                  // LKNEE2
    joints_.position[15] = joints_.position[16];                  // LKNEE3
    joints_.position[17] = -joints_.position[16];                 // LFOOT
  } else if (servo.servo_id == 3) {
    joints_.position[21] = ( (float)servo.servo_cmd
                             - joint_[3].cmdZero ) / 123.3;       // RHIP
    joints_.position[18] = joints_.position[21];                  // RHIP1
    joints_.position[19] = joints_.position[21];                  // RHIP2
    joints_.position[20] = joints_.position[21];                  // RHIP3
    joints_.position[22] = -joints_.position[21];                 // RKNEEBox
  } else if (servo.servo_id == 4) {
    joints_.position[23] = -( (float)servo.servo_cmd
                              - joint_[4].cmdZero ) / 123.3;       // RTWIST
  } else if (servo.servo_id == 5) {
    joints_.position[27] = -( (float)servo.servo_cmd
                              - joint_[5].cmdZero ) / 123.3;      // RKNEE
    joints_.position[24] = joints_.position[27];                  // RKNEE1
    joints_.position[25] = joints_.position[27];                  // RKNEE2
    joints_.position[26] = joints_.position[27];                  // RKNEE3
    joints_.position[28] = -joints_.position[27];                 // RFOOT
  } else if (servo.servo_id == 6) {
    joints_.position[1] = ( (float)servo.servo_cmd
                            - joint_[6].cmdZero ) / 123.3;        // LARM-Gear
    joints_.position[2] = -joints_.position[1] * 1.1875;          // LARM
  } else if (servo.servo_id == 7) {
    joints_.position[3] = ( (float)servo.servo_cmd
                            - joint_[7].cmdZero ) / 123.3;       // RARM-Gear
    joints_.position[4] = -joints_.position[3] * 1.1875;          // RARM
  } else if (servo.servo_id == 8) {
    joints_.position[6] = ( (float)servo.servo_cmd
                            - joint_[8].cmdZero ) / 123.3;        // EYE-Left
    joints_.position[5] = -joints_.position[6];                   // EYE-Right
  }
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
  double roll = atan2(accel_.y, accel_.x) - 1.57;
  double pitch = atan2(accel_.y, accel_.z) - 1.57;
  double yaw = atan2(accel_.z, accel_.x);
  roll = roundf(100.0 * roll) / 100.0;
  pitch = roundf(100.0 * pitch) / 100.0;
  yaw = roundf(100.0 * yaw) / 100.0;
  roll_.push_front(roll); pitch_.push_front(pitch); yaw_.push_front(yaw);
  int s = roll_.size();
  for (int i = 1; i < s; ++i) {
    roll += roll_[i]; pitch += pitch_[i]; yaw += yaw_[i];
  }
  roll = roll / s; pitch = pitch / s; yaw = yaw / s;
  roll_.resize(3); pitch_.resize(3); yaw_.resize(3);
  ROS_INFO_STREAM("R: " << (roll * 180) / 3.1415 <<
                  " P: " << (pitch * 180) / 3.1415 <<
                  " Y: " << (yaw * 180) / 3.1415);
  quat_ori_.setRPY(roll, -pitch, 0); //  Yaw Gimbal Lock
  quat_ori_.normalize();
}

void MartyCore::battCB(const std_msgs::Float32::ConstPtr& msg) {
  battery_val_ = msg->data;
}

void MartyCore::gpioCB(const marty_msgs::GPIOs::ConstPtr& msg) {
  gpios_val_ = *msg;
}

void MartyCore::updateOdom() {
  if (!odom_setup_) {
    this->setupOdometry();
  } else {
    geometry_msgs::TransformStamped l_foot_tf;
    geometry_msgs::TransformStamped r_foot_tf;
    geometry_msgs::Transform tf_diff;

    try {
      l_foot_tf = tf_buff_.lookupTransform("base_link", "left_foot", ros::Time(0));
      r_foot_tf = tf_buff_.lookupTransform("base_link", "right_foot", ros::Time(0));
    } catch (tf2::TransformException& ex) { ROS_WARN("%s", ex.what()); }
    if (l_foot_tf.transform.translation.z < r_foot_tf.transform.translation.z) {
      // Left Foot Down
      tf_diff.translation.x = l_foot_tf_.transform.translation.x -
                              l_foot_tf.transform.translation.x;
      tf_diff.translation.y = l_foot_tf_.transform.translation.y -
                              l_foot_tf.transform.translation.y;
      tf_diff.translation.z = l_foot_tf_.transform.translation.z -
                              l_foot_tf.transform.translation.z;
      tf_diff.rotation.x = l_foot_tf_.transform.rotation.x -
                           l_foot_tf.transform.rotation.x;
      tf_diff.rotation.y = l_foot_tf_.transform.rotation.y -
                           l_foot_tf.transform.rotation.y;
      tf_diff.rotation.z = l_foot_tf_.transform.rotation.z -
                           l_foot_tf.transform.rotation.z;
      tf_diff.rotation.w = l_foot_tf_.transform.rotation.w -
                           l_foot_tf.transform.rotation.w;
    } else {
      // Right Foot Down
      tf_diff.translation.x = r_foot_tf_.transform.translation.x -
                              r_foot_tf.transform.translation.x;
      tf_diff.translation.y = r_foot_tf_.transform.translation.y -
                              r_foot_tf.transform.translation.y;
      tf_diff.translation.z = r_foot_tf_.transform.translation.z -
                              r_foot_tf.transform.translation.z;
      tf_diff.rotation.x = r_foot_tf_.transform.rotation.x -
                           r_foot_tf.transform.rotation.x;
      tf_diff.rotation.y = r_foot_tf_.transform.rotation.y -
                           r_foot_tf.transform.rotation.y;
      tf_diff.rotation.z = r_foot_tf_.transform.rotation.z -
                           r_foot_tf.transform.rotation.z;
      tf_diff.rotation.w = r_foot_tf_.transform.rotation.w -
                           r_foot_tf.transform.rotation.w;
    }
    l_foot_tf_ = l_foot_tf;
    r_foot_tf_ = r_foot_tf;
    odom_tf_.transform.translation.x += tf_diff.translation.x;
    odom_tf_.transform.translation.y += tf_diff.translation.y;
    odom_tf_.transform.translation.z += tf_diff.translation.z;
    tf2::Quaternion q1 (odom_tf_.transform.rotation.x + tf_diff.rotation.x,
                        odom_tf_.transform.rotation.y + tf_diff.rotation.y,
                        odom_tf_.transform.rotation.z + tf_diff.rotation.z,
                        odom_tf_.transform.rotation.w + tf_diff.rotation.w);
    q1.normalize();
    odom_tf_.transform.rotation.x = q1.x();
    odom_tf_.transform.rotation.y = q1.y();
    odom_tf_.transform.rotation.z = q1.z();
    odom_tf_.transform.rotation.w = q1.w();

    // odom_tf_.transform.rotation.x = quat_ori_.x();
    // odom_tf_.transform.rotation.y = quat_ori_.y();
    // odom_tf_.transform.rotation.z = quat_ori_.z();
    // odom_tf_.transform.rotation.w = quat_ori_.w();

    // ROS_INFO_STREAM("TFx: " << tf_diff.translation.x <<
    //                 " TFy: " << tf_diff.translation.y <<
    //                 " TFz: " << tf_diff.translation.z);
  }
}

void MartyCore::tfCB(const ros::TimerEvent& e) {
  joints_.header.stamp = ros::Time::now();
  joints_pub_.publish(joints_);           // Publish Joint States
  cam_tf_.header.stamp = ros::Time::now();
  tf_br_.sendTransform(cam_tf_);          // Camera transformation
  odom_tf_.header.stamp = ros::Time::now();
  this->updateOdom();
  odom_br_.sendTransform(odom_tf_);       // Odometry transformation
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
  ros::spinOnce();
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
  ros::spinOnce();
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
