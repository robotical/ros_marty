/**
 * @file      calibration.hpp
 * @brief     Calibration class for calibrating Marty's servos
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */
#ifndef MARTY_CALIBRATION_HPP
#define MARTY_CALIBRATION_HPP

// System
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <termios.h>
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Messages
#include <std_msgs/Bool.h>
#include <marty_msgs/ServoMsg.h>
#include <marty_msgs/ServoMsgArray.h>

// MARTY
#include <ros_marty/definitions.hpp>

class Calibration {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.

  void loadParams();
  void init();
  void rosSetup();

 public:
  Calibration(ros::NodeHandle& nh);
  ~Calibration();
  void calibrate();

 private:
  void writeCalVals();
  char getch();

// Flags
  bool saving_;
  bool enabled_;
  bool calibrated_;
  char c_;

// Parameters

// Variables
  marty_msgs::ServoMsgArray joints_;

// ROS
  ros::Publisher joint_pub_;
  ros::Publisher enable_pub_;

};

#endif  /* MARTY_CALIBRATION_HPP */
