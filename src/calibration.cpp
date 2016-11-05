/**
 * @file      calibration.cpp
 * @brief     Calibration class for calibrating Marty's servos
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

// MARTY
#include <ros_marty/calibration.hpp>

Calibration::Calibration(ros::NodeHandle& nh) : nh_(nh) {
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_DEBUG("Marty Calibration Ready!");
}

Calibration::~Calibration() {
  ros::param::del("calibrate");
}

void Calibration::loadParams() {
  nh_.param("calibrated", calibrated_, false);
  if (!calibrated_) {
    WARN("Marty has not been calibrated before!" << std::endl);
  }
}

void Calibration::init() {
  enabled_ = true;
  saving_ = false;
  exiting_ = false;
  marty_msgs::ServoMsg joint;
  for (int id = 0; id < NUMJOINTS; ++id) {
    joint.servo_id = id;
    joints_.servo_msg.push_back(joint);
  }
  int v;
  for (int id = 0; id < NUMJOINTS; ++id) {
    nh_.param(NAMES[id] + "/zero", v, 0); joints_.servo_msg[id].servo_cmd = v;
  }
}

void Calibration::rosSetup() {
  joint_pub_ = nh_.advertise<marty_msgs::ServoMsgArray>("/servo_array", 10);
  enable_pub_ = nh_.advertise<std_msgs::Bool>("/enable_motors", 10);
}

void Calibration::calibrate() {
  CLEAR();
  INFO("q-a\tw-s\te-d\tr-f\tt-g\ty-h\tz-x\tc-v\tu-j\ti-k\to-l\tENTER to save\n");
  INFO("LHIP\tLTWIST\tLKNEE\tRHIP\tRTWIST\tRKNEE\tLARM\tRARM\tEYES\tAUX1\tAUX2\t");
  if (enabled_) { WARN("n - Enabled\n") } else { ERR("n - Disabled\n"); }
  INFO(int(joints_.servo_msg[LHIP].servo_cmd) << "\t" <<
       int(joints_.servo_msg[LTWIST].servo_cmd) << "\t" <<
       int(joints_.servo_msg[LKNEE].servo_cmd) << "\t" <<
       int(joints_.servo_msg[RHIP].servo_cmd) << "\t" <<
       int(joints_.servo_msg[RTWIST].servo_cmd) << "\t" <<
       int(joints_.servo_msg[RKNEE].servo_cmd) << "\t" <<
       int(joints_.servo_msg[LARM].servo_cmd) << "\t" <<
       int(joints_.servo_msg[RARM].servo_cmd) << "\t" <<
       int(joints_.servo_msg[EYES].servo_cmd) << "\t" <<
       int(joints_.servo_msg[AUX1].servo_cmd) << "\t" <<
       int(joints_.servo_msg[AUX2].servo_cmd) << "\t");
  WARN("m - Exit" << std::endl);

  // Publish if Robot is enabled
  std_msgs::Bool enabled;
  enabled.data = enabled_;
  enable_pub_.publish(enabled);
  // Publish Joint Commands
  if (enabled_) {
    joint_pub_.publish(joints_);
  }

  if (!exiting_) {
    if (!saving_) {
      if ((c_ = getch())) {
        ROS_INFO_STREAM("Char: " << c_);
        ROS_INFO_STREAM("SAVING: " << saving_ << " EXITING: " << exiting_);
        if (c_ == 'q') { joints_.servo_msg[LHIP].servo_cmd++; }
        if (c_ == 'a') { joints_.servo_msg[LHIP].servo_cmd--; }
        if (c_ == 'w') { joints_.servo_msg[LTWIST].servo_cmd++; }
        if (c_ == 's') { joints_.servo_msg[LTWIST].servo_cmd--; }
        if (c_ == 'e') { joints_.servo_msg[LKNEE].servo_cmd++; }
        if (c_ == 'd') { joints_.servo_msg[LKNEE].servo_cmd--; }
        if (c_ == 'r') { joints_.servo_msg[RHIP].servo_cmd++; }
        if (c_ == 'f') { joints_.servo_msg[RHIP].servo_cmd--; }
        if (c_ == 't') { joints_.servo_msg[RTWIST].servo_cmd++; }
        if (c_ == 'g') { joints_.servo_msg[RTWIST].servo_cmd--; }
        if (c_ == 'y') { joints_.servo_msg[RKNEE].servo_cmd++; }
        if (c_ == 'h') { joints_.servo_msg[RKNEE].servo_cmd--; }
        if (c_ == 'z') { joints_.servo_msg[LARM].servo_cmd++; }
        if (c_ == 'x') { joints_.servo_msg[LARM].servo_cmd--; }
        if (c_ == 'c') { joints_.servo_msg[RARM].servo_cmd++; }
        if (c_ == 'v') { joints_.servo_msg[RARM].servo_cmd--; }
        if (c_ == 'u') { joints_.servo_msg[EYES].servo_cmd++; }
        if (c_ == 'j') { joints_.servo_msg[EYES].servo_cmd--; }
        if (c_ == 'i') { joints_.servo_msg[AUX1].servo_cmd++; }
        if (c_ == 'k') { joints_.servo_msg[AUX1].servo_cmd--; }
        if (c_ == 'o') { joints_.servo_msg[AUX2].servo_cmd++; }
        if (c_ == 'l') { joints_.servo_msg[AUX2].servo_cmd--; }
        if (c_ == 'n') { enabled_ = !enabled_; }
        if (c_ == '\n') { saving_ = true; }
        if (c_ == 'm') { exiting_ = true; }
      }
      // Make sure the joint values do not go over the limit
      for (int id = 0; id < NUMJOINTS; ++id) {
        joints_.servo_msg[id].servo_cmd =
          std::min(int(joints_.servo_msg[id].servo_cmd), 126);
        joints_.servo_msg[id].servo_cmd =
          std::max(int(joints_.servo_msg[id].servo_cmd), -126);
      }
    } else {
      WARN("Are you sure you wish to save these values? (y/n) ");
      if ((c_ = getch())) {
        if (c_ == 'y') {
          this->writeCalVals();
          INFO("Saved!\n");
          ros::shutdown();
        } else { saving_ = false; }
      }
    }
  } else {
    WARN("Are you sure you want to exit? (y/n) ");
    if ((c_ = getch())) {
      if (c_ == 'y') {
        INFO("Exiting!\n");
        ros::shutdown();
      } else { exiting_ = false; }
    }
  }
}

void Calibration::writeCalVals() {
  std::string path = ros::package::getPath("ros_marty");
  std::ofstream of (path + "/cfg/joint_calib.cfg");
  of << "calibrated: true\n" << std::endl;
  for (int id = 0; id < NUMJOINTS; ++id) {
    std::string name = NAMES[id];
    int off(0);
    int val = int(joints_.servo_msg[id].servo_cmd);
    if ((name == "LHIP") or (name == "RHIP")) {off = HIPOFFSET;}
    else if ((name == "LTWIST") or (name == "RTWIST")) {off = TWISTOFFSET;}
    else if ((name == "LKNEE") or (name == "RKNEE")) {off = KNEEOFFSET;}
    else if ((name == "LARM") or (name == "RARM")) {off = ARMOFFSET;}
    else if (name == "EYES") {off = EYESOFFSET;}
    else if ((name == "AUX1") or (name == "AUX2")) {off = AUXOFFSET;}
    else {WARN("No offset found for joint: " << name << std::endl);}
    of << name << ": {min: " << std::max(val - off, -126) <<
       ", zero: " << val << ", max: " << std::min(val + off, 126) <<
       ", dir: " << JOINT_DIR[id] << ", mult: " << JOINT_MULT[id] <<  "}\n";
  }
  of.close();
}

char Calibration::getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
    perror ("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    perror ("tcsetattr ~ICANON");
  return (buf);
}
