/**
 * @file      marty_core.hpp
 * @brief     Marty Core header providing access to Marty methods
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#ifndef MARTY_CORE_HPP
#define MARTY_CORE_HPP

// System
#include <deque>

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <marty_msgs/ServoMsg.h>
#include <marty_msgs/ServoMsgArray.h>
#include <marty_msgs/Accelerometer.h>

// MARTY
#include <ros_marty/definitions.hpp>

struct robotJoint {
  int cmdZero;
  int cmdMin;
  int cmdMax;
  int cmdDir;
  float cmdMult;
  int servoChannel;
};

class MartyCore {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.

  void loadParams();
  void init();
  void rosSetup();

 public:
  MartyCore(ros::NodeHandle& nh);
  ~MartyCore();
  int jointPosToServoCmd(float pos, int zero, float mult,
                         int dir, int max, int min);
  void setServoJointPos(std::string name, int pos);
  void setServoPos(int channel, int pos);
  // bool stopServo(uint8_t jointIndex);
  void enableRobot();
  void stopRobot();
  bool setServo(int joint, float angle);
  void setServos(std::deque <float> angles);
  bool hasFallen() {return falling_;}

  // Public Variables
  robotJoint joint_[NUMJOINTS];
  int numjoints_;
  std::deque<float> jangles_;

 private:
  void accelCB(const marty_msgs::Accelerometer::ConstPtr& msg);

  bool setFallDetector(std_srvs::SetBool::Request&  req,
                       std_srvs::SetBool::Response& res);

  // Flags
  bool falling_;

  // Parameters
  bool calibrated_;
  bool check_fall_;
  double acc_thr_;

  // Variables
  marty_msgs::Accelerometer accel_msg_;

  // ROS
  std_msgs::Bool enable_robot_;
  marty_msgs::ServoMsg servo_msg_;
  marty_msgs::ServoMsgArray servo_msg_array_;

  ros::Publisher  enable_pub_;
  ros::Publisher  servo_pub_;
  ros::Publisher  servo_array_pub_;
  ros::Subscriber accel_sub_;
  ros::ServiceServer check_fall_srv_;

};

#endif  /* MARTY_CORE_HPP */
