/**
 * @file      calibration.cpp
 * @brief     Calibration interactive script of Marty Servo positions
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ros/ros.h>

#include <ros_marty/calibration.hpp>
// #include <ros_marty/marty_core.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nh("~");
  ros::Rate r(50);

  bool launched(false);
  nh.getParam("launched", launched);
  if (!launched) {
    ROS_ERROR("Please use 'roslaunch ros_marty calibration.launch' to calibrate\n");
  } else {
    Calibration calibration(nh);
    while (ros::ok()) {
      calibration.calibrate();
      ros::spinOnce();
      r.sleep();
    }
  }
  ros::shutdown();
  return 0;
}
