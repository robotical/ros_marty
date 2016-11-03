/**
 * @file      ros_marty.cpp
 * @brief     Sets up the ROS Marty API
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ros/ros.h>

#include <ros_marty/marty_core.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "marty");
  ros::NodeHandle nh("~");

  // MartyCore marty(&nh);
  ros::Rate r(50);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
