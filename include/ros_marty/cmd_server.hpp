/**
 * @file      cmd_server.hpp
 * @brief     CMD Server for Marty, allowing execution of CMDs remotely
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#ifndef CMD_SERVER_HPP
#define CMD_SERVER_HPP

// System
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <signal.h>

// Marty
#include "marty_msgs/Command.h"
#include "ros_marty/marty_core.hpp"
// #include "ros_marty/data_t.hpp"
#include "ros_marty/trajectory.hpp"

#define PORT  1569

#define DEFAULT_STEPTIME  2.0
#define DEFAULT_MOVETIME  1.0

#define CMD_LEFT  0
#define CMD_RIGHT 1

#define CMD_FORWARD   0
#define CMD_BACKWARD  1

#define CMD_POSITIVE  0
#define CMD_NEGATIVE  1

#define w_before 100
#define w_after 0

/**
 * @brief      Add new commands here for the server to receive
 */
enum Commands {
  CMD_HELLO = 0,
  CMD_MOVEKNEE,       // TODO: Move individual motor is 1 command
  CMD_MOVEHIP,
  CMD_LEANSAGITTAL,   // TODO: Lean in 4 directions is 1 command
  CMD_LEANSIDEWAYS,
  CMD_STEP,           // TODO: NOT IMPLEMENTED?
  CMD_WALK,
  CMD_EYES,
  CMD_KICK,
  CMD_MOVEJOINTS,     // TODO: NOT IMPLEMENTED?
  CMD_LIFTLEG,
  CMD_LOWERLEG,
  CMD_CELEBRATE,
  CMD_HIPTOBESQUARE,
  CMD_ROLLERSKATE,
  CMD_ARMS,
  CMD_DEMO,
  CMD_STOP
};

class CmdServer {
 protected:
  ros::NodeHandle nh_;
  void loadParams();
  void init();
  void rosSetup();

 private:
  // Methods
  void runCommand(vector<int> data);
  void hello();
  void walk(vector<int> data);
  void kick(vector<int> data);
  void eyes(vector<int> data);
  void moveKnee(vector<int> data); // TODO: Make moveJoint
  void moveHip(vector<int> data); // TODO: Make moveJoint
  void leanSagittal(vector<int> data); // TODO: Make lean
  void leanSideways(vector<int> data); // TODO: Make lean
  void celebrate();
  void liftLeg(vector<int> data);
  void lowerLeg(vector<int> data);
  void dance(vector<int> data);
  void arms(vector<int> data);
  void demo();
  void stopRobot();

  bool cmd_service(marty_msgs::Command::Request&  req,
                   marty_msgs::Command::Response& res);

  // Flags
  bool busy_;
  bool ros_cmd_;

  // Variables
  MartyCore* robot_;
  int sock_;
  std::vector<int> cmd_data_;

  // ROS
  ros::ServiceServer cmd_srv_;

 public:
  CmdServer(ros::NodeHandle& nh);
  ~CmdServer();
  void robotReady();
  void setupServer();
  void stopServer();
  void waitForCmd();
};

#endif  /* CMD_SERVER_HPP */
