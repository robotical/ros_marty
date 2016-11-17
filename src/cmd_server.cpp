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
  setPointsLeanLeft(tSetpoints, 30, 0, 1.0);
  setPointsLeanRight(tSetpoints, 30, 0, 2.0);
  setPointsLegsZero(tSetpoints, 1.0);
  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);

  sleepms(w_before);
  robot_->setServoPos(EYES, EYESANGRY);
  sleepms(250);
  robot_->setServoPos(EYES, EYESNORMAL);
  sleepms(w_after);
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
  switch (data[0]) {
  case CMD_HELLO: hello(); break;
  case CMD_MOVEKNEE: moveKnee(data); break;
  case CMD_MOVEHIP: moveHip(data); break;
  case CMD_LEANSAGITTAL: leanSagittal(data); break;
  case CMD_LEANSIDEWAYS: leanSideways(data); break;
  case CMD_WALK: walk(data); break;
  case CMD_KICK: kick(data); break;
  case CMD_EYES: eyes(data); break;
  case CMD_CELEBRATE: celebrate(); break;
  case CMD_LIFTLEG: liftLeg(data); break;
  case CMD_LOWERLEG: lowerLeg(data); break;
  case CMD_HIPTOBESQUARE: dance(data); break;
  case CMD_ROLLERSKATE: rollerSkate(robot_); break;
  case CMD_ARMS: arms(data); break;
  case CMD_DEMO: demo(); break;
  case CMD_STOP: stopRobot(); break;
  default:
    ROS_ERROR_STREAM("CmdServer did not recognise command " << data[0]); break;
  }
  busy_ = false;
}

void CmdServer::hello() {
  data_t tInterp;
  tInterp = genReturnToZero(robot_, 1.5);
  runTrajectory(robot_, tInterp);
  sleepms(w_before);
  robot_->setServoPos(EYES, EYESWIDE);
  sleepms(1000);
  robot_->setServoPos(EYES, EYESNORMAL);
  sleepms(w_after);
}

void CmdServer::walk(vector<int> data) {
  data_t tInterp;
  float steptime = DEFAULT_STEPTIME;
  // if (data.size() < 6) return -1;
  // check to see if steptime has been specified,
  // if so then change to seconds and set
  if (data.size() >= 7) { steptime = (float)data[6] / 10; }
  uint8_t numsteps = data[1];
  int walkDir = 1;
  if (data[2] == CMD_BACKWARD)
    walkDir = -1;
  uint8_t stepLength = data[3];
  int turnDir = 1;
  if (data[4] == CMD_RIGHT)
    turnDir = -1;
  uint8_t turn = data[5];
  uint8_t whichFoot = 0;
  if (robot_->jangles_[RHIP] < robot_->jangles_[LHIP])
    whichFoot = 1;
  if (walkDir < 0)
    whichFoot = (whichFoot + 1) % 2;
  for (int stepnum = 0; stepnum < numsteps; stepnum++) {
    if (whichFoot) {
      tInterp = genStepLeft(robot_, walkDir * stepLength,
                            turnDir * turn, steptime, 0);
    } else {
      tInterp = genStepRight(robot_, walkDir * stepLength,
                             turnDir * turn, steptime, 0);
    }
    runTrajectory(robot_, tInterp);
    whichFoot++;
    whichFoot %= 2;
  }
}

void CmdServer::kick(vector<int> data) {
  data_t tInterp;
  float kicktime = DEFAULT_STEPTIME;
  if (data.size() >= 3) { kicktime = (float)data[2] / 10; }
  if (data[1] == CMD_LEFT) {
    tInterp = genKickLeft(robot_, kicktime);
  } else {
    tInterp = genKickRight(robot_, kicktime);
  }
  runTrajectory(robot_, tInterp);
}

void CmdServer::eyes(vector<int> data) {
  // if (data.size() < 3) return -1;
  float angle = data[2];
  if (data[1] == CMD_NEGATIVE)
    angle *= -1;
  robot_->setServo(EYES, angle);
  if (data.size() >= 5) {
    angle = data[4];
    if (data[3] == CMD_NEGATIVE)
    {angle *= -1;}
  }
}

void CmdServer::moveKnee(vector<int> data) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_);
  tline.push_front(0);
  float movetime = DEFAULT_MOVETIME;
  if (data.size() >= 5) movetime = (float)data[4] / 10;

  float angle = (float)data[3];
  if (data[2] == CMD_NEGATIVE) angle *= -1;

  // Generate a trajectory to move the knee
  // first line is the robot's present position
  tSetpoints.clear();
  tSetpoints.push_back(tline);
  // tline should already have one line, with 0.0 timecode and robot's present angles
  if (data[1] == CMD_LEFT) {
    tline[1 + LKNEE] = angle;
  } else if (data[1] == CMD_RIGHT) {
    tline[1 + RKNEE] = angle;
  }
  tline[0] = movetime;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);
}

void CmdServer::moveHip(vector<int> data) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_);
  tline.push_front(0);
  float movetime = DEFAULT_MOVETIME;
  if (data.size() >= 5) movetime = (float)data[4] / 10;

  float angle = (float)data[3];
  if (data[2] == CMD_NEGATIVE) angle *= -1;

  tSetpoints.clear();
  tSetpoints.push_back(tline);
  if (data[1] == CMD_LEFT) {
    tline[1 + LHIP] = angle;
  } else if (data[1] == CMD_RIGHT) {
    tline[1 + RHIP] = angle;
  }
  tline[0] = movetime;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);
}

void CmdServer::leanSagittal(vector<int> data) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_);
  tline.push_front(0);
  float leanamount = data[2];
  float movetime = DEFAULT_MOVETIME;
  if (data.size() >= 4) { movetime = data[3] / 10; }

  tSetpoints.push_back(tline);
  if (data[1] == CMD_FORWARD) {
    setPointsLeanForward(tSetpoints, leanamount, movetime);
  } else if (data[1] == CMD_BACKWARD) {
    setPointsLeanBackward(tSetpoints, leanamount, movetime);
  }

  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);
}

void CmdServer::leanSideways(vector<int> data) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_);
  tline.push_front(0);
  float leanamount = data[2];
  float movetime = DEFAULT_MOVETIME;
  if (data.size() >= 4) { movetime = data[3] / 10; }

  tSetpoints.push_back(tline);
  if (data[1] == CMD_LEFT) {
    setPointsLeanLeft(tSetpoints, leanamount, 0, movetime);
  } else if (data[1] == CMD_RIGHT) {
    setPointsLeanRight(tSetpoints, leanamount, 0, movetime);
  }

  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);
}

void CmdServer::celebrate() {
  data_t tInterp;
  tInterp = genCelebration(robot_, 4.0);
  runTrajectory(robot_, tInterp);
}

void CmdServer::liftLeg(vector<int> data) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_);
  tline.push_front(0);
  float movetime = DEFAULT_MOVETIME;
  if (data.size() >= 4) { movetime = (float)data[3] / 10; }

  float angle = (float)data[2];

  // Generate a trajectory to move the knee
  // First line is the robot's present position
  tSetpoints.clear();
  tSetpoints.push_back(tline);
  // tline should already have one line, with 0.0 timecode and robot's present angles
  if (data[1] == CMD_LEFT) {
    tline[1 + LKNEE] = tline[1 + RKNEE] + angle;
  } else if (data[1] == CMD_RIGHT) {
    // note the negative for the right knee
    tline[1 + RKNEE] = tline[1 + LKNEE] - angle;
  }

  tline[0] = movetime;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);
}

// This will lower whichever leg is higher to the ground
// Initial implementation just thinks about the knees.
// TODO: should consider hip angle too,
// then adjust knee angle only to get foot to floor
// This function will never change the sign of the knee angle
void CmdServer::lowerLeg(vector<int> data) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot_->jangles_);
  tline.push_front(0);
  float movetime = DEFAULT_MOVETIME;
  if (data.size() >= 2) movetime = (float)data[1] / 10;
  // Save initial position to setpoints
  tSetpoints.push_back(tline);

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

  tline[0] = movetime;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot_, tInterp);
}

void CmdServer::dance(vector<int> data) {
  int robotID = (int)data[1];
  if (robotID < 0) robotID = 0; // TODO: Really...
  if (robotID > 2) robotID = 2;
  hipToBeSquare(robot_, robotID);
}

void CmdServer::arms(vector<int> data) {
  float angle1 = data[1];
  float angle2 = data[2];
  std::map<int, float> angles = {{RARM, angle1}, {LARM, angle2}};
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
  robot_->setServo(EYES, EYESNORMAL);
  sleep(1);
  robot_->setServo(EYES, EYESANGRY);
  sleep(1);
  robot_->setServo(EYES, EYESEXCITED);
  sleep(1);
  robot_->setServo(EYES, EYESWIDE);
  sleep(1);

  // Test eyes
  for (alpha = 0; alpha < 6.28; alpha += 0.1) {
    robot_->setServo(EYES, EYESANGRY * sin(alpha));;
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
}

void CmdServer::rosSetup() {
  cmd_srv_ = nh_.advertiseService("command", &CmdServer::cmd_service, this);
}

bool CmdServer::cmd_service(marty_msgs::Command::Request&  req,
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
  socklen_t clilen = sizeof(clientaddr);
  int clisock = accept(sock_, (struct sockaddr*) &clientaddr, &clilen);
  if (clisock >= 0) {
    robot_->enableRobot();
    ROS_DEBUG("Incoming connection accepted...\n");
    int nbytes = read(clisock, buffer, 256);
    if (nbytes > 0) {
      ROS_DEBUG("Data received: %d", buffer[0]);
      vector<int> dbytes;
      dbytes.push_back(buffer[0]);
      for (int i = 1; i < nbytes; i++) {
        ROS_DEBUG("\t%d", buffer[i]);
        dbytes.push_back(buffer[i]);
      }
      ROS_DEBUG("Running Command\n");
      if (robot_->hasFallen()) {
        ROS_WARN("Marty has fallen over! Please pick him up and try again.");
      } else {
        runCommand(dbytes);
      }
      ROS_DEBUG("Done!\n");
    }
    close(clisock);
  } else if (ros_cmd_) {
    robot_->enableRobot();
    runCommand(cmd_data_);
    ros_cmd_ = false;
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
