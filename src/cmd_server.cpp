#include "ros_marty/cmd_server.hpp"

using namespace Trajectory;

/**
 * @brief      Robot motion executed when cmdServer is ready
 */
void CmdServer::robotReady() {
  robot_->enableRobot();
  data_t tSetpoints, tInterp;
  deque <float> tline(NUMJOINTS + 1, 0);
  tSetpoints.push_back(tline);
  setPointsLeanLeft(tSetpoints, 30, 0.5);
  setPointsLeanRight(tSetpoints, 30, 1.0);
  setPointsLegsZero(tSetpoints, 0.5);
  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);

  sleepms(w_before);
  robot_->setServoPos(EYES, EYES_ANGRY);
  sleepms(250);
  robot_->setServoPos(EYES, EYES_NORMAL);
  sleepms(w_after);
  robot_->martyReady();
  robot_->stopRobot();
}

/**
 * @brief Runs commands received by the server
 * @details Here is where you may add new functions for Marty to do.
 * Don't forget to create a CmdServer::function() as well as its
 * definition in the cmd_server.hpp
 *
 * @param data Received data array
 */
void CmdServer::runCommand(vector<int> data) {
  busy_ = true;
  // ROS_INFO_STREAM("CMD: " << data[0]);
  // for (int i = 0; i < data.size(); ++i) {
  //   ROS_INFO_STREAM("CMDData " << i << ": " << data[i]);
  // }
  int l = data.size();
  switch (data[0]) {
  case CMD_HELLO: hello(); break;
  case CMD_MOVEJOINT:
    if (l == 4) {moveJoint(data[1], data[2], data[3]);}
    if (l == 5) {moveJoint(data[1], data[2], data[3], data[4]);} break;
  case CMD_LEAN:
    if (l == 3) {lean(data[1], data[2]);}
    if (l == 4) {lean(data[1], data[2], data[3]);} break;
  case CMD_WALK:
    if (l == 2) {walk(data[1]);} if (l == 3) {walk(data[1], data[2]);}
    if (l == 4) {walk(data[1], data[2], data[3]);}
    if (l == 5) {walk(data[1], data[2], data[3], data[4]);} break;
  case CMD_KICK:
    if (l == 2) {kick(data[1]);} if (l == 3) {kick(data[1], data[2]);} break;
  case CMD_EYES:
    if (l == 1) {eyes();} if (l == 2) {eyes(data[1]);}
    if (l == 3) {eyes(data[1], data[2]);} break;
  case CMD_CELEBRATE:
    if (l == 2) {celebrate(data[1]);} break;
  case CMD_LIFTLEG:
    if (l == 2) {liftLeg(data[1]);} if (l == 3) {liftLeg(data[1], data[2]);}
    if (l == 4) {liftLeg(data[1], data[2], data[3]);} break;
  case CMD_LOWERLEG:
    if (l == 2) {lowerLeg(data[1]);} break;
  case CMD_DANCE:
    dance(data[1]); break;
  case CMD_ROLLERSKATE: rollerSkate(robot_); break;
  case CMD_ARMS:
    if (l == 2) {arms(data[1]);} if (l == 3) {arms(data[1], data[2]);} break;
  case CMD_DEMO: demo(); break;
  case CMD_GET: if (l == 3) {getData(data[1], data[2]);} break;
  case CMD_SOUND: if (l == 2) {playSound(data[1]);}
    if (l == 3) {playSound(data[1], data[2]);} break;
  case CMD_STOP: stopRobot(); break;
  default:
    ROS_ERROR_STREAM("CmdServer did not recognise command " << data[0]); break;
  }
  busy_ = false;
}

void CmdServer::runSockCommand(vector<int8_t> data) {
  if ((data.size() % 4) != 0) {ROS_ERROR("Data Package wrong size!");} else {
    vector<int> cmd_data;
    for (int i = 0; i < (data.size() / 4); ++i) {
      // ROS_INFO_STREAM("Data " << i << ": " << (int)data[i]);
      cmd_data.push_back(0);
      memcpy(&cmd_data[i], &data[(i * 4)], 4);
      // ROS_INFO_STREAM("NewData " << cmd_data[i]);
    }
    // if (busy_) {ROS_INFO("BUSY!");} else {ROS_INFO("NOT BUSY!");}
    if (!busy_) { runCommand(cmd_data); }
    else { cmd_queue_.push_back(cmd_data); }
  }
}

void CmdServer::hello() {
  data_t tInterp;
  // Return body to center
  tInterp = genReturnToZero(robot_, 1.5);
  runTrajectory(robot_, tInterp);
  // Move eyes
  sleepms(w_before);
  robot_->setServoPos(EYES, EYES_WIDE);
  sleepms(500);
  robot_->setServoPos(EYES, EYES_NORMAL);
  sleepms(w_after);
}

void CmdServer::walk(int num_steps, int turn, int move_time, int step_length) {
  data_t tInterp;
  float step_time = ((float) move_time) / 1000; // Float in seconds
  bool leftFoot = false;
  // Step with left foot if LHip is ahead
  if (robot_->jangles_[RHIP] < robot_->jangles_[LHIP]) { leftFoot = true; }
  // Invert if walking backwards
  if (step_length < 0) { leftFoot = !leftFoot; }
  // Start walking!
  for (int step_num = 0; step_num < num_steps; step_num++) {
    if (leftFoot) {
      tInterp = genStepLeft(robot_, step_length, turn, step_time, 0);
    } else {
      tInterp = genStepRight(robot_, step_length, turn, step_time, 0);
    }
    runTrajectory(robot_, tInterp);
    leftFoot = !leftFoot;
  }
}

void CmdServer::kick(int side, int move_time) {
  data_t tInterp;
  float kicktime = ((float)move_time) / 1000;
  if (side == CMD_LEFT) {
    tInterp = genKickLeft(robot_, kicktime);
  } else {
    tInterp = genKickRight(robot_, kicktime);
  }
  runTrajectory(robot_, tInterp);
}

void CmdServer::eyes(int amount, int amount2) {
  robot_->setServo(EYES, amount);
}

void CmdServer::getData(int sensor, int id) {
  resp_request_ = true;
  if (sensor == GET_GPIO) {
    if ((id >= 8) || (id < 0)) {resp_request_ = false;}
    else { val_request_ = gpio_data_.gpio[id]; }
  } else if (sensor == GET_ACCEL) {
    if (id == AXES_X) { val_request_ = accel_data_.x;}
    else if (id == AXES_Y) { val_request_ = accel_data_.y;}
    else if (id == AXES_Z) { val_request_ = accel_data_.z;}
    else {resp_request_ = false;}
  } else if (sensor == GET_BATT) {
    val_request_ = batt_data_.data;
  } else if (sensor == GET_CURR) {
    if ((id >= 8) || (id < 0)) {resp_request_ = false;}
    else { val_request_ = curr_data_.current[id]; }
  } else {
    resp_request_ = false;
  }
}

void CmdServer::moveJoint(int side, int joint, int amount, int move_time) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_); tline.push_front(0);
  // Generate a trajectory to move the joint
  // first line is the robot's present position
  tSetpoints.clear(); tSetpoints.push_back(tline);
  if (joint == J_HIP) {
    if (side == CMD_LEFT) {
      tline[1 + LHIP] = (float)amount;
    } else if (side == CMD_RIGHT) {
      tline[1 + RHIP] = (float)amount;
    }
  } else if (joint == J_TWIST) {
    if (side == CMD_LEFT) {
      tline[1 + LTWIST] = (float)amount;
    } else if (side == CMD_RIGHT) {
      tline[1 + RTWIST] = (float)amount;
    }
  } else if (joint == J_KNEE) {
    if (side == CMD_LEFT) {
      tline[1 + LKNEE] = (float)amount;
    } else if (side == CMD_RIGHT) {
      tline[1 + RKNEE] = (float)amount;
    }
  } else if (joint == J_LEG) {
    if (side == CMD_LEFT) {
      tline[1 + LHIP] = (float)amount;
    } else if (side == CMD_RIGHT) {
      tline[1 + RHIP] = (float)amount;
    }
  } else if (joint == J_ARM) {
    if (side == CMD_LEFT) {
      tline[1 + LARM] = (float)amount;
    } else if (side == CMD_RIGHT) {
      tline[1 + RARM] = (float)amount;
    }
  } else if (joint == J_EYES) {
    tline[1 + EYES] = (float)amount;
  }
  tline[0] = ((float)move_time) / 1000; tSetpoints.push_back(tline);
  interpTrajectory(tSetpoints, tInterp, 0.05); runTrajectory(robot_, tInterp);
}

void CmdServer::lean(int dir, int amount, int move_time) {
  data_t tSetpoints, tInterp; deque<float> tline(robot_->jangles_);
  tline.push_front(0); tSetpoints.push_back(tline);
  float lean_time = ((float)move_time) / 1000;
  if (dir == CMD_LEFT) {setPointsLeanLeft(tSetpoints, amount, lean_time);}
  if (dir == CMD_RIGHT) {setPointsLeanRight(tSetpoints, amount, lean_time);}
  if (dir == CMD_FORW) {setPointsLeanForward(tSetpoints, amount, lean_time);}
  if (dir == CMD_BACK) {setPointsLeanBackward(tSetpoints, amount, lean_time);}
  interpTrajectory(tSetpoints, tInterp, 0.05); runTrajectory(robot_, tInterp);
}

void CmdServer::celebrate(int move_time) {
  data_t tInterp;
  tInterp = genCelebration(robot_, ((float)move_time) / 1000);
  runTrajectory(robot_, tInterp);
}

// Generate a trajectory to move the knee
// First line is the robot's present position
// tline should already have one line,
// with 0.0 timecode and robot's present angles
void CmdServer::liftLeg(int leg, int amount, int move_time) {
  data_t tSetpoints, tInterp; deque<float> tline(robot_->jangles_);
  tline.push_front(0); tSetpoints.clear(); tSetpoints.push_back(tline);
  if (leg == CMD_LEFT) {
    tline[1 + LKNEE] = tline[1 + RKNEE] + (float)amount;
  } else if (leg == CMD_RIGHT) {
    tline[1 + RKNEE] = tline[1 + LKNEE] - (float)amount;
  }
  tline[0] = ((float)move_time) / 1000; tSetpoints.push_back(tline);
  interpTrajectory(tSetpoints, tInterp, 0.05); runTrajectory(robot_, tInterp);
}

// This will lower whichever leg is higher to the ground
// Initial implementation just thinks about the knees.
// TODO: should consider hip angle too,
// then adjust knee angle only to get foot to floor
// This function will never change the sign of the knee angle
void CmdServer::lowerLeg(int move_time) {
  data_t tSetpoints, tInterp; deque<float> tline(robot_->jangles_);
  tline.push_front(0); tSetpoints.push_back(tline);

  if (abs(robot_->jangles_[RKNEE]) > abs(robot_->jangles_[LKNEE])) {
    // Right knee is higher than left knee
    if (robot_->jangles_[RKNEE] < 0) {
      tline[1 + RKNEE] = 0 - abs(robot_->jangles_[LKNEE]);
    } else {
      tline[1 + RKNEE] = abs(robot_->jangles_[LKNEE]);
    }
  } else {  // Left knee is higher than right knee
    if (robot_->jangles_[LKNEE] < 0) {
      tline[1 + LKNEE] = 0 - abs(robot_->jangles_[RKNEE]);
    } else {
      tline[1 + LKNEE] = abs(robot_->jangles_[RKNEE]);
    }
  }
  tline[0] = ((float)move_time) / 1000; tSetpoints.push_back(tline);
  interpTrajectory(tSetpoints, tInterp, 0.05); runTrajectory(robot_, tInterp);
}

void CmdServer::dance(int robot_id) {
  if (robot_id < 0) { robot_id = 0; } if (robot_id > 2) { robot_id = 2; }
  hipToBeSquare(robot_, robot_id);
}

void CmdServer::arms(int r_angle, int l_angle) {
  std::map<int, float> angles = {{RARM, (float)r_angle}, {LARM, (float)l_angle}};
  robot_->setServos(angles);
}

void CmdServer::demo() {
  float alpha = 0;
  int delay = 20000;
  std::map<int, float> angles;
  for (int i = 0; i < NUMJOINTS; ++i) {
    angles[i] = robot_->jangles_[i];
  }
  // Default Arm positions
  robot_->setServo(EYES, EYES_NORMAL);
  sleep(1);
  robot_->setServo(EYES, EYES_ANGRY);
  sleep(1);
  robot_->setServo(EYES, EYES_EXCITED);
  sleep(1);
  robot_->setServo(EYES, EYES_WIDE);
  sleep(1);

  // Test eyes
  for (alpha = 0; alpha < 6.28; alpha += 0.1) {
    robot_->setServo(EYES, EYES_ANGRY * sin(alpha));;
    usleep(delay);
  }

  // Arms Swing Together
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    angles[RARM] = 100 * sin(alpha);
    angles[LARM] = 100 * sin(alpha);
    robot_->setServos(angles);
    usleep(delay);
  }

  // Arms Swing Opposed
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    angles[RARM] = 100 * sin(alpha);
    angles[LARM] = -100 * sin(alpha);
    robot_->setServos(angles);
    usleep(delay);
  }

  // Forward and Backward
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    angles[RHIP] = 45 * sin(alpha); angles[LHIP] = angles[RHIP];
    robot_->setServos(angles);
    usleep(delay);
  }

  // Side to Side
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    angles[RKNEE] = 45 * sin(alpha); angles[LKNEE] = angles[RKNEE];
    robot_->setServos(angles);
    usleep(delay);
  }

  // Feet Twist in same direction
  angles[RHIP] = 0; angles[RKNEE] = 0; angles[LHIP] = 0;
  angles[LKNEE] = 0;
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    angles[RTWIST] = 45 * sin(alpha); angles[LTWIST] = angles[RTWIST];
    robot_->setServos(angles);

    usleep(delay);
  }

  // Feet Twist in opposite direction
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    angles[RTWIST] = 20 * sin(alpha); angles[LTWIST] = 0 - angles[RTWIST];
    robot_->setServos(angles);
    usleep(delay);
  }

  // Test Left Leg
  data_t genTraj;
  genTraj = genRaisedFootTwistLeft(robot_, 4.0);
  runTrajectory(robot_, genTraj);
  genTraj.clear();

  // Test Right Leg
  genTraj = genRaisedFootTwistRight(robot_, 4.0);
  runTrajectory(robot_, genTraj);
  genTraj.clear();

  // Walk about
  for (int numstep = 0; numstep < 1; numstep++) {
    genTraj = genStepLeft(robot_, 0, 25, 1.25, 0);
    runTrajectory(robot_, genTraj);
    genTraj.clear();

    genTraj = genStepRight(robot_, 0, 25, 1.25, 0);
    runTrajectory(robot_, genTraj);
    genTraj.clear();
  }

  for (int numstep = 0; numstep < 2; numstep++) {
    genTraj = genStepLeft(robot_, 80, 0, 1.5, 0);
    runTrajectory(robot_, genTraj);
    genTraj.clear();

    genTraj = genStepRight(robot_, 80, 0, 1.5, 0);
    runTrajectory(robot_, genTraj);
    genTraj.clear();
  }

  // Test a Left Kick
  genTraj = genKickLeft(robot_, 2.0);
  runTrajectory(robot_, genTraj);
  genTraj.clear();

  // Celebrate!
  genTraj = genCelebration(robot_, 4.0);
  runTrajectory(robot_, genTraj);
  genTraj.clear();
}

void CmdServer::playSound(int frequency, int duration) {
  robot_->playSound(frequency, (float)duration / 1000);
}

void CmdServer::stopRobot() {
  sleepms(100);
  robot_->stopRobot();
}

// SERVER FUNCTIONS, DO NOT TOUCH!
volatile sig_atomic_t exiting = 0;
void exit_f(int sig) {
  ROS_WARN("EXITING!");
  ros::param::del("/marty");
  ros::shutdown();
  exiting = 1;
}

CmdServer::CmdServer(ros::NodeHandle& nh) : nh_(nh) {
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_INFO("CmdServer Ready!");
}

CmdServer::~CmdServer() {
  ros::param::del("/marty");
}

void CmdServer::loadParams() {
  bool launched;
  nh_.param("launched", launched, false);
  if (!launched) {
    ROS_ERROR("Please use 'roslaunch ros_marty marty.launch'\n");
    ros::shutdown();
  }
}

void CmdServer::init() {
  robot_ = new MartyCore(nh_);
  busy_ = false;
  ros_cmd_ = false;
  resp_request_ = false;
}

void CmdServer::rosSetup() {
  gpio_sub_ = nh_.subscribe("/gpios", 1000, &CmdServer::gpioCB, this);
  accel_sub_ = nh_.subscribe("/accel", 1000, &CmdServer::accelCB, this);
  batt_sub_ = nh_.subscribe("/battery", 1000, &CmdServer::battCB, this);
  curr_sub_ = nh_.subscribe("/motor_currents", 1000, &CmdServer::currCB, this);
  cmd_srv_ = nh_.advertiseService("/command", &CmdServer::cmd_service, this);
}

bool CmdServer::cmd_service(marty_msgs::Command::Request& req,
                            marty_msgs::Command::Response& res) {
  if (!busy_) { ros_cmd_ = true; cmd_data_ = req.data; res.success = true; }
  else { res.success = false; }
  return true;
}

void CmdServer::setupServer() {
  // Create socket, initialise and listen
  struct sockaddr_in servaddr;
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) { ROS_ERROR("cannot create socket"); ros::shutdown(); }
  int yes = 1;
  setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

  memset((char*)&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(PORT);
  if (bind(sock_, (struct sockaddr*) &servaddr, sizeof(servaddr)) < 0)
  { ROS_ERROR("ERROR on binding"); ros::shutdown(); }
  listen(sock_, 5);

  // set server socket to non-binding, so that accept will not hang
  fcntl(sock_, F_SETFL, O_NONBLOCK);
  printf("Socket initialised on port: %d. Waiting for incoming connection\n",
         PORT);
}

void CmdServer::stopServer() {
  close(sock_);
}

void CmdServer::waitForCmd() {
  struct sockaddr_in clientaddr;
  char buffer[256];
  char const* resp_msg = "";
  socklen_t clilen = sizeof(clientaddr);
  int clisock = accept(sock_, (struct sockaddr*) &clientaddr, &clilen);
  if (clisock >= 0) {
    robot_->enableRobot();
    ROS_DEBUG("Incoming connection accepted...\n");
    int nbytes = read(clisock, buffer, 256);
    if (nbytes > 0) {
      ROS_DEBUG("Data received: %d", buffer[0]);
      vector<int8_t> dbytes;
      dbytes.push_back(buffer[0]);
      for (int i = 1; i < nbytes; i++) {
        ROS_DEBUG("\t%d", buffer[i]);
        dbytes.push_back(buffer[i]);
      }
      ROS_DEBUG("Running Command\n");
      if (robot_->hasFallen()) {
        ROS_WARN("Marty has fallen over! Please pick him up and try again.");
        resp_msg = "Fallen";
      } else {
        runSockCommand(dbytes);
        resp_msg = "Success";
      }
      ROS_DEBUG("Done!\n");
    }
    if (resp_request_) {
      write(clisock, (char*)&val_request_, sizeof(val_request_));
      resp_request_ = false;
    } else {
      write(clisock, resp_msg, strlen(resp_msg));
    }
    close(clisock);
  } else if (ros_cmd_) {
    robot_->enableRobot();
    if (!busy_) { runCommand(cmd_data_); }
    else { cmd_queue_.push_back(cmd_data_); }
    ros_cmd_ = false;
  } else if ((!busy_) && (cmd_queue_.size() > 0)) {
    ROS_INFO_STREAM("RUNNING NEXT CMD: " << cmd_queue_[0][0]);
    runCommand(cmd_queue_[0]);
    cmd_queue_.erase(cmd_queue_.begin());
  }
}

int main(int argc, char** argv) {
  signal(SIGINT, exit_f);
  ros::init(argc, argv, "cmd_server");
  ros::NodeHandle nh("~");

  CmdServer cmd_server(nh);
  cmd_server.robotReady();
  cmd_server.setupServer();

  ros::Rate r(50);
  while (ros::ok() && !exiting) {
    cmd_server.waitForCmd();
    ros::spinOnce();
    r.sleep();
  }
  cmd_server.stopServer();
  ros::shutdown();
  return 0;
}
