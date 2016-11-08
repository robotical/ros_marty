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
#include <std_msgs/Float32.h>
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
  int jointPosToServoCmd(int id, float pos);
  void setServoJointPos(std::string name, int pos);
  void setServoPos(int channel, int pos);
  // bool stopServo(uint8_t jointIndex);
  void enableRobot();
  void stopRobot();
  bool setServo(int id, float angle);
  void setServos(std::deque <float> angles);

  // Getters/Setters
  bool hasFallen() {return falling_.data;}

  // Public Variables
  robotJoint joint_[NUMJOINTS];
  std::deque<float> jangles_;

 private:
  void accelCB(const marty_msgs::Accelerometer::ConstPtr& msg);
  void battCB(const std_msgs::Float32::ConstPtr& msg);

  bool setFallDetector(std_srvs::SetBool::Request&  req,
                       std_srvs::SetBool::Response& res);

  // Flags

  // Parameters
  bool calibrated_;
  bool fall_disable_;
  double acc_thr_;

  // Variables
  marty_msgs::Accelerometer accel_;
  std_msgs::Bool falling_;
  float battery_val_;

  // ROS
  std_msgs::Bool enable_robot_;
  marty_msgs::ServoMsg servo_msg_;
  marty_msgs::ServoMsgArray servo_msg_array_;

  ros::Publisher  enable_pub_;
  ros::Publisher  falling_pub_;
  ros::Publisher  servo_pub_;
  ros::Publisher  servo_array_pub_;
  ros::Subscriber accel_sub_;
  ros::Subscriber batt_sub_;
  ros::ServiceServer fall_dis_srv_;

};

#endif  /* MARTY_CORE_HPP */
