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
#include <fstream>
#include <sstream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <marty_msgs/Accelerometer.h>
#include <marty_msgs/GPIOs.h>
#include <marty_msgs/MotorCurrents.h>
#include <marty_msgs/Output.h>
#include <marty_msgs/ServoMsg.h>
#include <marty_msgs/ServoMsgArray.h>
#include <marty_msgs/GPIOConfig.h>
#include <marty_msgs/Sound.h>
#include <marty_msgs/SoundArray.h>
#include <marty_msgs/Keyframe.h>
#include <marty_msgs/KeyframeArray.h>
#include <geometry_msgs/TransformStamped.h>

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
  void loadSound(std::string sound_name);
  void loadKeyframes(std::string keyframes_name, int move_time = 0);
  void readySound();
  void celebSound(float sound_time);
  int jointPosToServoCmd(int id, float pos);
  void setServoJointPos(std::string name, int pos);
  void setServoPos(int channel, int pos);
  // bool stopServo(uint8_t jointIndex);
  void enableRobot();
  void stopRobot();
  bool setServo(int id, float angle);
  void setServos(std::map<int, float> angles);

  void playSound(float frequency, float duration, float freq2 = -1);
  void playSoundArray(marty_msgs::SoundArray sound_array);
  void stopSound();

  // Getters/Setters
  bool hasFallen() {return falling_.data;}
  bool fallDisabled() {return fall_disable_;}
  float getBattery();

  // Public Variables
  robotJoint joint_[NUMJOINTS]; //  Internal Joint Data
  std::deque<float> jangles_;

 private:
  void setupJointStates();
  void setupOdometry();
  void jointCB(const marty_msgs::ServoMsg::ConstPtr& msg);
  void jointsCB(const marty_msgs::ServoMsgArray::ConstPtr& msg);
  void accelCB(const marty_msgs::Accelerometer::ConstPtr& msg);
  void battCB(const std_msgs::Float32::ConstPtr& msg);
  void gpioCB(const marty_msgs::GPIOs::ConstPtr& msg);
  void soundCB(const std_msgs::String::ConstPtr& msg)
  {this->loadSound(msg->data);}
  void sendKeyframes(marty_msgs::KeyframeArray k_array);
  void kfCB(const std_msgs::String::ConstPtr& msg)
  {this->loadKeyframes(msg->data);}
  void tfCB(const ros::TimerEvent& e);

  void updateJointState(marty_msgs::ServoMsg servo);
  void updateOdom();
  bool setFallDetector(std_srvs::SetBool::Request&  req,
                       std_srvs::SetBool::Response& res);

  // Flags
  bool calibrated_;     // Check whether Marty has been calibrated yet
  bool odom_setup_;     // Check whether odometry tfs have been setup

  // Parameters
  bool fall_disable_;   // If true disables motor commands when fallen
  double fall_thr_;     // Threshold for determining if Marty is falling
  bool odom_accel_;     // Combine accelerometer data with odometry
  bool simulated_;      // Whether Marty is living in a dream world
  // double batt_thr_;
  bool camera_;         // If true camera tf and functions are enabled
  double camera_ori_;   // Camera orientation relative to base_link

  // Variables
  marty_msgs::Accelerometer accel_;
  std_msgs::Bool falling_;
  float battery_val_;
  float battery_max_;
  float battery_min_;
  marty_msgs::GPIOs gpios_val_;
  std::deque<double> roll_;
  std::deque<double> pitch_;
  std::deque<double> yaw_;

  // ROS
  std_msgs::Bool enable_robot_;
  marty_msgs::ServoMsg servo_msg_;
  sensor_msgs::JointState joints_;  // ROS Joint State msg
  // marty_msgs::ServoMsgArray servo_msg_array_;
  geometry_msgs::TransformStamped cam_tf_;
  geometry_msgs::TransformStamped odom_tf_;
  geometry_msgs::TransformStamped l_f_tf_;
  geometry_msgs::TransformStamped r_f_tf_;

  ros::Publisher  enable_pub_;
  ros::Publisher  falling_pub_;
  ros::Publisher  servo_pub_;
  ros::Publisher  servo_array_pub_;
  ros::Publisher  joints_pub_;
  ros::Publisher  sound_pub_;
  ros::Publisher  keyframe_pub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber joints_sub_;
  ros::Subscriber accel_sub_;
  ros::Subscriber batt_sub_;
  ros::Subscriber gpio_sub_;
  ros::Subscriber sound_sub_;
  ros::Subscriber keyframe_sub_;

  ros::ServiceServer fall_dis_srv_;
  ros::ServiceClient set_gpio_config_;
  ros::ServiceClient get_gpio_config_;

  // TF
  double r_, p_, y_;
  tf2::Quaternion quat_ori_;
  tf2_ros::TransformBroadcaster tf_br_;
  tf2_ros::TransformBroadcaster odom_br_;
  tf2_ros::Buffer tf_buff_;
  tf2_ros::TransformListener tf_ls_;
  ros::Timer tf_timer_;
};

#endif  /* MARTY_CORE_HPP */
