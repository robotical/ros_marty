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
  ros::param::param("/marty/fall_threshold", fall_thr_, 0.9);
  ros::param::param("/marty/odom_accel", odom_accel_, true);
  ros::param::param("/marty/camera", camera_, false);
  if (camera_) {ros::param::param("/marty/camera_ori", camera_ori_, 15.0);}

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

  // Camera TF
  if (camera_) {
    cam_tf_.header.frame_id = "base_link";
    cam_tf_.child_frame_id = "camera";
    cam_tf_.transform.translation.x = 0.023;
    cam_tf_.transform.translation.y = 0.0;
    cam_tf_.transform.translation.z = 0.027;
    // tf2::Quaternion q(-0.54168, 0.54168, -0.45452, 0.45452);
    // tf2::Quaternion q(-0.596, 0.596, -0.380, 0.380); // 25 deg
    tf2::Quaternion q(-0.627, 0.627, -0.327, 0.327); // 35 deg
    q.normalize();
    cam_tf_.transform.rotation = tf2::toMsg(q);
    // double ori = ((90 - camera_ori_) / 180) * M_PI;
    // ROS_INFO_STREAM("CamOri: " << camera_ori_ << " ORI: " << ori <<
    // " PI: " << M_PI);
    // q.setRPY(-M_PI, ori, (M_PI / 2)); // TODO: This is borked
    // ROS_INFO_STREAM("X: " << q.x() << " Y: " << q.y() <<
    //                 " Z: " << q.z() << " W: " << q.w());
    // cam_tf_.transform.rotation.x = tf2::toMsg(q);
  }

  // Odom TF
  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_link";
  odom_tf_.transform.translation.x = 0;
  odom_tf_.transform.translation.y = 0;
  odom_tf_.transform.translation.z = 0;
  odom_tf_.transform.rotation.x = 0;
  odom_tf_.transform.rotation.y = 0;
  odom_tf_.transform.rotation.z = 0;
  odom_tf_.transform.rotation.w = 1;
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
  // TIMERS
  tf_timer_ = nh_.createTimer(ros::Duration(0.1), &MartyCore::tfCB, this);
}

bool MartyCore::setFallDetector(std_srvs::SetBool::Request& req,
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
  try {   // Wait until transformation data from feet is received
    l_f_tf_ = tf_buff_.lookupTransform("base_link", "left_foot", ros::Time(0));
    r_f_tf_ = tf_buff_.lookupTransform("base_link", "right_foot", ros::Time(0));
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

/**
 * @brief Updates the joint state positions of every joint given servo cmds
 * @details This function is called on every callback that updates a joint
 * position, and publishes the appropriate updates to the joint_states topic.
 *
 * @param servo The servo cmd message (Id and Position)
 */
void MartyCore::updateJointState(marty_msgs::ServoMsg servo) {
  if (servo.servo_id == 0) {
    joints_.position[10] =
      ( (float)servo.servo_cmd - joint_[0].cmdZero ) / 123.3;     // LHIP
    joints_.position[7] = joints_.position[10];                   // LHIP1
    joints_.position[8] = joints_.position[10];                   // LHIP2
    joints_.position[9] = joints_.position[10];                   // LHIP3
    joints_.position[11] = -joints_.position[10];                 // LKNEEBox
  } else if (servo.servo_id == 1) {
    joints_.position[12] =
      -( (float)servo.servo_cmd - joint_[1].cmdZero ) / 123.3;    // LTWIST
  } else if (servo.servo_id == 2) {
    joints_.position[16] =
      -( (float)servo.servo_cmd - joint_[2].cmdZero ) / 123.3;    // LKNEE
    joints_.position[13] = joints_.position[16];                  // LKNEE1
    joints_.position[14] = joints_.position[16];                  // LKNEE2
    joints_.position[15] = joints_.position[16];                  // LKNEE3
    joints_.position[17] = -joints_.position[16];                 // LFOOT
  } else if (servo.servo_id == 3) {
    joints_.position[21] =
      ( (float)servo.servo_cmd - joint_[3].cmdZero ) / 123.3;     // RHIP
    joints_.position[18] = joints_.position[21];                  // RHIP1
    joints_.position[19] = joints_.position[21];                  // RHIP2
    joints_.position[20] = joints_.position[21];                  // RHIP3
    joints_.position[22] = -joints_.position[21];                 // RKNEEBox
  } else if (servo.servo_id == 4) {
    joints_.position[23] =
      -( (float)servo.servo_cmd - joint_[4].cmdZero ) / 123.3;    // RTWIST
  } else if (servo.servo_id == 5) {
    joints_.position[27] =
      -( (float)servo.servo_cmd - joint_[5].cmdZero ) / 123.3;    // RKNEE
    joints_.position[24] = joints_.position[27];                  // RKNEE1
    joints_.position[25] = joints_.position[27];                  // RKNEE2
    joints_.position[26] = joints_.position[27];                  // RKNEE3
    joints_.position[28] = -joints_.position[27];                 // RFOOT
  } else if (servo.servo_id == 6) {
    joints_.position[1] =
      ( (float)servo.servo_cmd - joint_[6].cmdZero ) / 123.3;     // LARM-Gear
    joints_.position[2] = -joints_.position[1] * 1.1875;          // LARM
  } else if (servo.servo_id == 7) {
    joints_.position[3] =
      ( (float)servo.servo_cmd - joint_[7].cmdZero ) / 123.3;     // RARM-Gear
    joints_.position[4] = -joints_.position[3] * 1.1875;          // RARM
  } else if (servo.servo_id == 8) {
    joints_.position[6] =
      ( (float)servo.servo_cmd - joint_[8].cmdZero ) / 123.3;     // EYE-Left
    joints_.position[5] = -joints_.position[6];                   // EYE-Right
  }
}

void MartyCore::accelCB(const marty_msgs::Accelerometer::ConstPtr& msg) {
  accel_ = *msg;
  // Check if Robot is falling
  if ((accel_.y < fall_thr_) && (falling_.data == false)) {
    ROS_WARN_STREAM("Robot Falling! " << accel_.y << std::endl);
    falling_.data = true;
    if (fall_disable_) {
      enable_robot_.data = false; enable_pub_.publish(enable_robot_);
    }
    falling_pub_.publish(falling_);
  }
  if ((accel_.y > fall_thr_) && (falling_.data == true)) {
    ROS_WARN_STREAM("Robot Stable! " << accel_.y << std::endl);
    falling_.data = false;
    if (fall_disable_) {
      enable_robot_.data = true; enable_pub_.publish(enable_robot_);
    }
    falling_pub_.publish(falling_);
  }
  // Obtain Roll, Pitch and Yaw angles from accelerometer data
  double roll = atan2(accel_.y, accel_.x) - 1.57;
  double pitch = atan2(accel_.y, accel_.z) - 1.57;
  double yaw = atan2(accel_.z, accel_.x);
  // Perform moving window average to filter accelerator noise
  roll_.push_front(roll); pitch_.push_front(pitch); yaw_.push_front(yaw);
  int s = roll_.size();
  for (int i = 1; i < s; ++i) {
    roll += roll_[i]; pitch += pitch_[i]; yaw += yaw_[i];
  }
  roll = roll / s; pitch = pitch / s; yaw = yaw / s;
  roll_.resize(3); pitch_.resize(3); yaw_.resize(3);
  r_ = roll; p_ = -pitch;   // Yaw discarded due to gimbal lock
}

void MartyCore::battCB(const std_msgs::Float32::ConstPtr& msg) {
  battery_val_ = msg->data;
}

void MartyCore::gpioCB(const marty_msgs::GPIOs::ConstPtr& msg) {
  gpios_val_ = *msg;
}

/**
 * @brief This function updates the Odometry given Marty's motion and sensors
 * @details Marty uses joint state data to estimate its pose as it moves. The
 * change in position of the feet (Assuming the base_link of the robot moves
 * inversely to the foot on the ground)
 */
void MartyCore::updateOdom() {
  if (!odom_setup_) {
    this->setupOdometry();
  } else {
    geometry_msgs::TransformStamped l_f_tf, r_f_tf; // Left/Right Foot TFs
    geometry_msgs::Transform tf_diff;  // TF change between time frames
    double dy;  // Yaw change from previous timestep

    // Initialise transformations
    tf_diff.translation.x = tf_diff.translation.y = tf_diff.translation.z = 0;
    tf_diff.rotation.x = tf_diff.rotation.y = tf_diff.rotation.z = 0;
    tf_diff.rotation.w = 1;
    l_f_tf.transform = r_f_tf.transform = tf_diff;

    try {   // Acquire latest feet transformation data
      l_f_tf = tf_buff_.lookupTransform("base_link", "left_foot", ros::Time(0));
      r_f_tf = tf_buff_.lookupTransform("base_link", "right_foot", ros::Time(0));
    } catch (tf2::TransformException& ex) { ROS_WARN("%s", ex.what()); }

    // Calculate feet previous and current roll, pitch, yaw orientations
    double rql_, pql_, yql_, rql, pql, yql, rqr_, pqr_, yqr_, rqr, pqr, yqr;
    tf2::Quaternion ql_(l_f_tf_.transform.rotation.x, l_f_tf_.transform.rotation.y,
                        l_f_tf_.transform.rotation.z, l_f_tf_.transform.rotation.w);
    ql_.normalize(); tf2::Matrix3x3(ql_).getRPY(rql_, pql_, yql_);
    tf2::Quaternion ql(l_f_tf.transform.rotation.x, l_f_tf.transform.rotation.y,
                       l_f_tf.transform.rotation.z, l_f_tf.transform.rotation.w);
    ql.normalize(); tf2::Matrix3x3(ql).getRPY(rql, pql, yql);
    tf2::Quaternion qr_(r_f_tf_.transform.rotation.x, r_f_tf_.transform.rotation.y,
                        r_f_tf_.transform.rotation.z, r_f_tf_.transform.rotation.w);
    qr_.normalize(); tf2::Matrix3x3(qr_).getRPY(rqr_, pqr_, yqr_);
    tf2::Quaternion qr(r_f_tf.transform.rotation.x, r_f_tf.transform.rotation.y,
                       r_f_tf.transform.rotation.z, r_f_tf.transform.rotation.w);
    qr.normalize(); tf2::Matrix3x3(qr).getRPY(rqr, pqr, yqr);

    // Compute change in Translation and Yaw given foot on ground
    if (l_f_tf.transform.translation.z < r_f_tf.transform.translation.z) {
      dy = yql_ - yql; // Left Foot Down
      tf_diff.translation.x =
        l_f_tf_.transform.translation.x - l_f_tf.transform.translation.x;
      tf_diff.translation.y =
        l_f_tf_.transform.translation.y - l_f_tf.transform.translation.y;
      tf_diff.translation.z =
        l_f_tf_.transform.translation.z - l_f_tf.transform.translation.z;
    } else if (l_f_tf.transform.translation.z > r_f_tf.transform.translation.z) {
      dy = yqr_ - yqr; // Right Foot Down
      tf_diff.translation.x =
        r_f_tf_.transform.translation.x - r_f_tf.transform.translation.x;
      tf_diff.translation.y =
        r_f_tf_.transform.translation.y - r_f_tf.transform.translation.y;
      tf_diff.translation.z =
        r_f_tf_.transform.translation.z - r_f_tf.transform.translation.z;
    } else {
      dy = ((yql_ - yql) + (yqr_ - yqr)) / 2; // Both Feet Down, take average
      tf_diff.translation.x =
        ((l_f_tf_.transform.translation.x - l_f_tf.transform.translation.x) +
         (r_f_tf_.transform.translation.x - r_f_tf.transform.translation.x)) / 2;
      tf_diff.translation.y =
        ((l_f_tf_.transform.translation.y - l_f_tf.transform.translation.y) +
         (r_f_tf_.transform.translation.y - r_f_tf.transform.translation.y)) / 2;
      tf_diff.translation.z =
        ((l_f_tf_.transform.translation.z - l_f_tf.transform.translation.z) +
         (r_f_tf_.transform.translation.z - r_f_tf.transform.translation.z)) / 2;
    }
    l_f_tf_ = l_f_tf; r_f_tf_ = r_f_tf;   // Update feet tfs for next step
    y_ += dy;   // Update Yaw and Translation Odometry estimates
    odom_tf_.transform.translation.x +=
      (tf_diff.translation.x * cos(-y_)) + (tf_diff.translation.y * sin(-y_));
    odom_tf_.transform.translation.y +=
      (tf_diff.translation.y * cos(-y_)) + (tf_diff.translation.x * -sin(-y_));
    odom_tf_.transform.translation.z += tf_diff.translation.z;
    if (odom_accel_) {  // Combine Accelerometer data
      quat_ori_.setRPY(r_, p_, y_); quat_ori_.normalize();
    } else {  // Use only estimated Yaw
      quat_ori_.setRPY(0, 0, y_); quat_ori_.normalize();
    }
    odom_tf_.transform.rotation = tf2::toMsg(quat_ori_);
  }
}

void MartyCore::tfCB(const ros::TimerEvent& e) {
  // Publish Joint States
  joints_.header.stamp = ros::Time::now(); joints_pub_.publish(joints_);
  if (camera_) {  // Camera transformation
    cam_tf_.header.stamp = ros::Time::now(); tf_br_.sendTransform(cam_tf_);
  }
  // Odometry transformation
  this->updateOdom();
  odom_tf_.header.stamp = ros::Time::now(); odom_br_.sendTransform(odom_tf_);
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
