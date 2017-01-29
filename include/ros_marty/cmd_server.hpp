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

// ROS
#include "std_msgs/Float32.h"
#include "marty_msgs/GPIOs.h"
#include "marty_msgs/Accelerometer.h"
#include "marty_msgs/MotorCurrents.h"

#define PORT  1569

#define DEFAULT_STEPTIME  2.0
#define DEFAULT_MOVETIME  1.0

// #define CMD_LEFT  0
// #define CMD_RIGHT 1

// #define CMD_FORWARD   0
// #define CMD_BACKWARD  1

#define CMD_POSITIVE  0
#define CMD_NEGATIVE  1

#define w_before 150
#define w_after 150

/**
 * @brief      Add new commands here for the server to receive
 */
enum Direction {
  CMD_LEFT = 0,
  CMD_RIGHT,
  CMD_FORW,
  CMD_BACK
};

enum Axis {
  AXES_X = 0,
  AXES_Y,
  AXES_Z
};

enum Joint {
  J_HIP = 0,
  J_TWIST,
  J_KNEE,
  J_LEG,
  J_ARM,
  J_EYES
};

enum Commands {
  CMD_HELLO = 0,
  CMD_MOVEJOINT,
  CMD_LEAN,
  CMD_WALK,
  CMD_EYES,
  CMD_KICK,
  CMD_LIFTLEG,
  CMD_LOWERLEG,
  CMD_CELEBRATE,
  CMD_DANCE,
  CMD_ROLLERSKATE,
  CMD_ARMS,
  CMD_DEMO,
  CMD_GET,
  CMD_SIDESTEP,
  CMD_SOUND,
  CMD_STOP
};

enum Sensors {
  GET_GPIO = 0,
  GET_ACCEL,
  GET_BATT,
  GET_CURR
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
  void runSockCommand(vector<int8_t> data);

  void arms(int r_angle = 0, int l_angle = 0);
  void celebrate(int move_time = 4000);
  void dance(int robot_id);
  void demo();
  void eyes(int amount = 0, int amount2 = 0);
  void getData(int sensor, int id);
  void hello();
  void kick(int side = CMD_LEFT, int move_time = 3000);
  void lean(int dir, int amount = 100, int move_time = 2000);
  void liftLeg(int leg, int amount = 100, int move_time = 2000);
  void lowerLeg(int move_time = 1000);
  void moveJoint(int side, int joint, int amount, int movetime = 2000);
  void sideStep(int side, int num_steps = 1, int movetime = 1000,
                int step_length = 50);
  void playSound(int freq = 440, int duration = 500);
  void stopRobot();
  void walk(int num_steps = 2, int turn = 0,
            int move_time = 3000, int step_length = 50, int side = -1);

  void gpioCB(const marty_msgs::GPIOs& msg) {gpio_data_ = msg;}
  void accelCB(const marty_msgs::Accelerometer& msg) {accel_data_ = msg;}
  void battCB(const std_msgs::Float32& msg) {batt_data_ = msg;}
  void currCB(const marty_msgs::MotorCurrents& msg) {curr_data_ = msg;}
  bool cmd_service(marty_msgs::Command::Request&  req,
                   marty_msgs::Command::Response& res);

  // Flags
  bool busy_;
  bool ros_cmd_;
  bool resp_request_;

  // Params
  bool ready_move_;

  // Variables
  MartyCore* robot_;
  int sock_;
  std::vector<int> cmd_data_;
  std::vector<std::vector<int> > cmd_queue_;
  float val_request_;
  marty_msgs::GPIOs gpio_data_;
  marty_msgs::Accelerometer accel_data_;
  std_msgs::Float32 batt_data_;
  marty_msgs::MotorCurrents curr_data_;

  // ROS
  ros::Subscriber gpio_sub_;
  ros::Subscriber accel_sub_;
  ros::Subscriber batt_sub_;
  ros::Subscriber curr_sub_;
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
