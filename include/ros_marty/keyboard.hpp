/**
 * @file      keyboard.hpp
 * @brief     Keyboard app class for controlling Marty with a keyboard
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */
#ifndef MARTY_KEYBOARD_HPP
#define MARTY_KEYBOARD_HPP

// System
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <termios.h>
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <keyboard/Key.h>

// MARTY
#include <marty_msgs/Command.h>

class Keyboard {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.

  void loadParams();
  void init();
  void rosSetup();

 public:
  Keyboard(ros::NodeHandle& nh);
  ~Keyboard();
  void run();

 private:
  void keyUpCB(const keyboard::Key::ConstPtr& msg);
  void keyDownCB(const keyboard::Key::ConstPtr& msg);
  // char getch();

// Flags
  bool forw_;
  bool back_;
  bool left_;
  bool right_;
  // char c_;

// Parameters
  bool calibrated_;

// Variables
  marty_msgs::Command srv;

// ROS
  ros::Subscriber key_up_sub_;
  ros::Subscriber key_down_sub_;
  ros::ServiceClient send_cmd_;

};

#endif  /* MARTY_KEYBOARD_HPP */
