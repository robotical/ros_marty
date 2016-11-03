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
  nh_.deleteParam("marty_core");
}

void MartyCore::loadParams() {
  nh_.param("calibrated", calibrated_, false);
  if (!calibrated_) {
    ERR("Marty has not been calibrated before!\n");
    WARN("Please use 'roslaunch ros_marty calibration.launch' to calibrate\n");
    ros::shutdown();
    exit(0);
  }
  for (int id = 0; id < NUMJOINTS; ++id) {
    std::string zero_p = JOINT_NAMES[id] + "/zero";
    std::string max_p = JOINT_NAMES[id] + "/max";
    std::string min_p = JOINT_NAMES[id] + "/min";
    std::string dir_p = JOINT_NAMES[id] + "/dir";
    std::string mult_p = JOINT_NAMES[id] + "/mult";
    nh_.getParam(zero_p, joint_[id].cmdZero);
    nh_.getParam(max_p, joint_[id].cmdMax);
    nh_.getParam(min_p, joint_[id].cmdMin);
    nh_.getParam(dir_p, joint_[id].cmdDir);
    nh_.getParam(mult_p, joint_[id].cmdMult);
  }
  // ROS_WARN_STREAM("EYESWIDE: " << EYESWIDE << " EYESNORMAL: " << EYESNORMAL <<
  // std::endl);

  marty_msgs::ServoMsg joint;
  for (int id = 0; id < NUMJOINTS; ++id) {
    joint.servo_id = id;
    joint.servo_cmd = 0;
    // joint.servo_cmd = joint_[id].cmdZero;
    servo_msg_array_.servo_msg.push_back(joint);
  }
}

void MartyCore::init() {
  numjoints_ = NUMJOINTS;
  for (int ji = 0; ji < NUMJOINTS; ji++) jangles_.push_back(0);
}

void MartyCore::rosSetup() {
  enable_pub_ = nh_.advertise<std_msgs::Bool>("/enable_motors", 10);
  servo_pub_ = nh_.advertise<marty_msgs::ServoMsg>("/servo", 10);
  servo_array_pub_ = nh_.advertise<marty_msgs::ServoMsgArray>("/servo_array", 10);
}

int MartyCore::jointPosToServoCmd(float pos, int zero, float mult,
                                  int dir, int max, int min) {
  // if (DEBUG_MODE) {
  //   printf("posToCmd: pos: %.2f, zero: %d, mult: %.2f, dir: %d, max: %d, min: %d\n",
  //          pos, zero, mult, dir, max, min);
  // }
  float cmd = float(zero) + (pos * mult * float(dir));
  // ROS_INFO_STREAM("CMD:" << cmd << " ZERO: " << zero << " POS: " << pos <<
  //                 " MULT: " << mult << " DIR:" << dir << " MAX:" << max <<
  //                 " MIN:" << min << std::endl);
  if (cmd < min) { cmd = min; } else if (cmd > max) { cmd = max; }
  return int(cmd);
}

void MartyCore::setServoJointPos(std::string name, int pos) {
  // servo_msg_.servo_id = ;
  servo_msg_.servo_cmd = pos;
  servo_pub_.publish(servo_msg_);
}

void MartyCore::setServoPos(int channel, int pos) {
  servo_msg_.servo_id = channel;
  servo_msg_.servo_cmd = pos;
  servo_pub_.publish(servo_msg_);
}

// bool MartyCore::stopServo(uint8_t jointIndex) {
//   setPWM(this->i2cptr, this->deviceAddr, this->joint[jointIndex].servoChannel, 0);
//   return true;
// }

void MartyCore::enableRobot() {
  enable_robot_.data = true;
  enable_pub_.publish(enable_robot_);
}

void MartyCore::stopRobot() {
  enable_robot_.data = false;
  enable_pub_.publish(enable_robot_);
}

bool MartyCore::setServo(int ji, float angle) {
  // ROS_WARN("SETTING SERVO!");
  if (ji < 0 || ji >= NUMJOINTS)
    return false;
  int cmd = jointPosToServoCmd(angle, joint_[ji].cmdZero, joint_[ji].cmdMult,
                               joint_[ji].cmdDir, joint_[ji].cmdMax, joint_[ji].cmdMin);
  // if (DEBUG_MODE) {
  //   printf("setting servo on channel: %d to %d\n",
  //          joint_[ji].servoChannel, cmd);
  // }
  // setServoPos(ji, cmd);
  servo_msg_.servo_id = ji;
  servo_msg_.servo_cmd = cmd;
  servo_pub_.publish(servo_msg_);
  jangles_[ji] = angle;
  return true;
}

void MartyCore::setServos(std::deque <float> angles) {
  // ROS_WARN("SETTING SERVOS");
  int ji = 0;
  for (std::deque<float>::iterator ai = angles.begin(); ai != angles.end();
       ai++) {
    int cmd = jointPosToServoCmd(*ai, joint_[ji].cmdZero, joint_[ji].cmdMult,
                                 joint_[ji].cmdDir, joint_[ji].cmdMax,
                                 joint_[ji].cmdMin);
    // setServoPos(ji, cmd); // TODO: User servo_msg_array instead!
    // servo_msg_array_.servo_msg[ji].servo_id = ji;
    servo_msg_array_.servo_msg[ji].servo_cmd = cmd;
    jangles_[ji] = *ai;
    ji++;
  }
  servo_array_pub_.publish(servo_msg_array_);
  // ros::spinOnce();
}

// void MartyCore::printAngles() {
//   if (DEBUG_MODE) {
//     printf("Angles from inside class: \t");
//     for (std::deque<float>::iterator di = this->jangles.begin();
//          di != this->jangles.end(); di++) {
//       printf("%2.2f\t", *di);
//     }
//     printf("\n");
//   }
//   return;
// }
