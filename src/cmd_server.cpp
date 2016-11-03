#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <deque>
#include <vector>
#include <time.h>
#include <cmath>

// #include "i2c.h"
#include "ros_marty/marty_core.hpp"
#include "ros_marty/data_t.hpp"
#include "ros_marty/trajectory.hpp"

#define PORT  1569

#define DEFAULT_STEPTIME  2.0
#define DEFAULT_MOVETIME  1.0

// TODO: Move to cross-language configuration file
enum Commands {
  CMD_HELLO = 0,
  CMD_MOVEKNEE,
  CMD_MOVEHIP,
  CMD_LEANSAGITTAL,
  CMD_LEANSIDEWAYS,
  CMD_STEP,
  CMD_WALK,
  CMD_EYES,
  CMD_KICK,
  CMD_MOVEJOINTS,
  CMD_LIFTLEG,
  CMD_LOWERLEG,
  CMD_CELEBRATE,
  CMD_HIPTOBESQUARE,
  CMD_ROLLERSKATE,
  CMD_ARMS
};

#define CMD_LEFT  0
#define CMD_RIGHT 1

#define CMD_FORWARD   0
#define CMD_BACKWARD  1

#define CMD_POSITIVE  0
#define CMD_NEGATIVE  1

using namespace std;

int runCommand(MartyCore& robot, vector<uint8_t> data) {
  ROS_WARN("RUNNING COMMAND");
  int nbytes = data.size();
  printf("packet size: %d\n", nbytes);
  uint8_t cmd = data[0];

  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles_);
  tline.push_front(0);
  switch (cmd) {
  case CMD_HELLO:
    // This function doesn't take any arguments, it just
    // makes the robot move a little and then ensures it's at the zero pos
    // tInterp = genReturnToZero(robot, 1.5);
    // runTrajectory(robot, tInterp);
    robot.setServo(EYES, EYESWIDE);
    sleep(1.0);
    robot.setServo(EYES, EYESNORMAL);
    break;

  case CMD_MOVEKNEE: {
    if (data.size() < 4) return -1;
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
    runTrajectory(robot, tInterp);
    break;
  }

  case CMD_MOVEHIP: {
    if (data.size() < 4) return -1;
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
    runTrajectory(robot, tInterp);
    break;
  }

  case CMD_LEANSAGITTAL: {
    if (data.size() < 3) return -1;
    float leanamount = data[2];
    float movetime = DEFAULT_MOVETIME;
    if (data.size() >= 4) movetime = data[3] / 10;


    tSetpoints.push_back(tline);
    if (data[1] == CMD_FORWARD) {
      setPointsLeanForward(tSetpoints, leanamount, movetime);
    } else if (data[1] == CMD_BACKWARD) {
      setPointsLeanBackward(tSetpoints, leanamount, movetime);
    }

    interpTrajectory(tSetpoints, tInterp, 0.05);
    runTrajectory(robot, tInterp);

    break;
  }

  case CMD_LEANSIDEWAYS: {
    if (data.size() < 3) return -1;
    float leanamount = data[2];
    float movetime = DEFAULT_MOVETIME;
    if (data.size() >= 4) movetime = data[3] / 10;

    tSetpoints.push_back(tline);
    if (data[1] == CMD_LEFT) {
      setPointsLeanLeft(tSetpoints, leanamount, 0, movetime);
    } else if (data[1] == CMD_RIGHT) {
      setPointsLeanRight(tSetpoints, leanamount, 0, movetime);
    }

    interpTrajectory(tSetpoints, tInterp, 0.05);
    runTrajectory(robot, tInterp);

    break;
  }


  case CMD_WALK: {
    ROS_WARN("CMD_WALK!");
    float steptime = DEFAULT_STEPTIME;
    if (data.size() < 6) return -1;
    // check to see if steptime has been specified, if so then change to seconds and set
    if (data.size() >= 7) steptime = (float)data[6] / 10;
    uint8_t numsteps = data[1];
    int walkDir = 1;
    if (data[2] == CMD_BACKWARD)
      walkDir = -1;
    uint8_t stepLength = data[3];
    int turnDir = 1;
    if (data[4] == CMD_LEFT)
      turnDir = -1;
    uint8_t turn = data[5];
    uint8_t whichFoot = 0;
    if (robot.jangles_[RHIP] < robot.jangles_[LHIP])
      whichFoot = 1;
    if (walkDir < 0)
      whichFoot = (whichFoot + 1) % 2;
    for (int stepnum = 0; stepnum < numsteps; stepnum++) {
      if (whichFoot) {
        tInterp = genStepLeft(robot, walkDir * stepLength, turnDir * turn, steptime, 0);
      } else {
        tInterp = genStepRight(robot, walkDir * stepLength, turnDir * turn, steptime,
                               0);
      }
      runTrajectory(robot, tInterp);
      whichFoot++;
      whichFoot %= 2;
    }
    break;
  }

  case CMD_KICK: {
    if (data.size() < 2) return -1;
    float kicktime = DEFAULT_STEPTIME;
    if (data.size() >= 3) kicktime = (float)data[2] / 10;
    if (data[1] == CMD_LEFT) {
      tInterp = genKickLeft(robot, kicktime);
    } else {
      tInterp = genKickRight(robot, kicktime);
    }
    runTrajectory(robot, tInterp);
    break;
  }

  case CMD_EYES: {
    if (data.size() < 3) return -1;
    float angle = data[2];
    if (data[1] == CMD_NEGATIVE)
      angle *= -1;
    robot.setServo(EYES, angle);
    if (data.size() >= 5) {
      angle = data[4];
      if (data[3] == CMD_NEGATIVE)
        angle *= -1;
    }
    break;
  }

  case CMD_CELEBRATE: {
    tInterp = genCelebration(robot, 4.0);
    runTrajectory(robot, tInterp);
    break;
  }

  case CMD_LIFTLEG: {
    if (data.size() < 3) return -1;
    float movetime = DEFAULT_MOVETIME;
    if (data.size() >= 4) movetime = (float)data[3] / 10;

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
    runTrajectory(robot, tInterp);
    break;
  }

  case CMD_LOWERLEG: {
    // This will lower whichever leg is higher to the ground
    // Initial implementation just thinks about the knees.
    // TODO: should consider hip angle too,
    // then adjust knee angle only to get foot to floor
    // This function will never change the sign of the knee angle

    printf("lower leg command\n");

    float movetime = DEFAULT_MOVETIME;
    if (data.size() >= 2) movetime = (float)data[1] / 10;

    // Save initial position to setpoints
    tSetpoints.push_back(tline);

    printf("angles: rknee: %.2f\tlknee: %.2f\n", robot.jangles_[RKNEE],
           robot.jangles_[LKNEE]);
    if (abs(robot.jangles_[RKNEE]) > abs(robot.jangles_[LKNEE])) {
      // Right knee is higher than left knee
      if (robot.jangles_[RKNEE] < 0) {
        tline[1 + RKNEE] = 0 - abs(robot.jangles_[LKNEE]);
      } else {
        tline[1 + RKNEE] = abs(robot.jangles_[LKNEE]);
      }
    } else {
      // Left knee is higher than right knee
      if (robot.jangles_[LKNEE] < 0) {
        tline[1 + LKNEE] = 0 - abs(robot.jangles_[RKNEE]);
      } else {
        tline[1 + LKNEE] = abs(robot.jangles_[RKNEE]);
      }
    }

    tline[0] = movetime;
    tSetpoints.push_back(tline);

    interpTrajectory(tSetpoints, tInterp, 0.05);
    runTrajectory(robot, tInterp);
    break;
  }
  case CMD_HIPTOBESQUARE: {
    if (data.size() < 2) return -1;
    int robotID = (int)data[1];
    if (robotID < 0) robotID = 0;
    if (robotID > 2) robotID = 2;
    hipToBeSquare(robot, robotID);
    break;
  }
  case CMD_ROLLERSKATE: {
    rollerSkate(robot);
  }
  case CMD_ARMS: {
    float angle1 = data[1];
    float angle2 = data[2];
    robot.setServo(RARM, angle1);
    robot.setServo(LARM, angle2);
  }
  }
  return 1;
}


int main(int argc, char** argv) {
  // Start robot-y stuff
  ros::init(argc, argv, "cmd_server");
  ros::NodeHandle nh("~");

  MartyCore robot(nh);
  ros::Publisher enable_pub_ = nh.advertise<std_msgs::Bool>("/enable_motors", 10);
  std_msgs::Bool enabled;
  enabled.data = true;
  enable_pub_.publish(enabled);
  ros::spinOnce();
  ROS_INFO("HELLO");
  // Little "Hello" from the robot
  data_t tSetpoints, tInterp;
  deque <float> tline(NUMJOINTS + 1, 0);
  tSetpoints.push_back(tline);
  setPointsLeanLeft(tSetpoints, 30, 0, 1.0);
  setPointsLeanRight(tSetpoints, 30, 0, 2.0);
  setPointsLegsZero(tSetpoints, 1.0);
  interpTrajectory(tSetpoints, tInterp, 0.05);
  runTrajectory(robot, tInterp);

  robot.setServo(EYES, EYESWIDE);
  sleep(1.0);
  robot.setServo(EYES, EYESNORMAL);

  // Networking setup
  struct sockaddr_in servaddr, clientaddr;
  socklen_t clilen;
  int port = PORT, sock, clisock;
  int nbytes;
  char buffer[256];

  // Create socket, initialise and listen
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("cannot create socket");
    return 0;
  }
  int yes = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

  memset((char*)&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port);
  if (bind(sock, (struct sockaddr*) &servaddr, sizeof(servaddr)) < 0)
    printf("ERROR on binding");
  listen(sock, 5);

  ros::Rate r(1);
  while (ros::ok()) {

    printf("Socket initialised on port: %d. Waiting for incoming connection\n",
           port);
    enabled.data = false;
    enable_pub_.publish(enabled);
    // Accept incoming connection
    clilen = sizeof(clientaddr);
    ros::spinOnce();
    clisock = accept(sock, (struct sockaddr*) &clientaddr, &clilen);
    if (clisock < 0)
      printf("ERROR on accept");
    // Set the client port to nonblocking, to allow the main loop
    // to run without pausing for incoming data
    // We assume that there is only one server running,
    // and just one client connected at any given time
    // fcntl(clisock, F_SETFL, O_NONBLOCK);

    printf("Incoming connection accepted...\n");
    // int error = 0;
    // socklen_t len = sizeof (error);
    // int retval;
    // while ((retval = getsockopt (clisock, SOL_SOCKET, SO_ERROR, &error, &len ))==0){
    nbytes = read(clisock, buffer, 256);

    if (nbytes > 0) {
      printf("Data received: %d", buffer[0]);
      vector<uint8_t> dbytes;
      dbytes.push_back(buffer[0]);
      for (int i = 1; i < nbytes; i++) {
        printf("\t%d", buffer[i]);
        dbytes.push_back(buffer[i]);
      }
      printf("\n");
      enabled.data = true;
      enable_pub_.publish(enabled);
      runCommand(robot, dbytes);
      // runCommand(robot, (uint8_t) buffer[0]);

    }
    //}
    printf("Closing connection\n");
    close(clisock);
    ros::spinOnce();
    r.sleep();
  }

  ros::param::del("cmd_server");
  ros::shutdown();
  return 0;
}
