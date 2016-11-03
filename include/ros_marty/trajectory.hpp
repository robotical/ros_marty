#ifndef MARTY_TRAJECTORY_HPP
#define MARTY_TRAJECTORY_HPP

#include <deque>
#include <vector>

#include "ros_marty/definitions.hpp"
#include "ros_marty/data_t.hpp"
#include "ros_marty/marty_core.hpp"

#define TRAJ_FROMZERO   0x01
#define TRAJ_TOZERO   0x02

float gettime();

bool interpTrajectory(data_t tIn, data_t& tOut, float dt);
void printTrajectory(data_t& traj);
bool runTrajectory(MartyCore& robot, data_t& traj);
data_t genStepLeft(MartyCore& robot, int stepLength, int turn, float period,
                   char flags);
data_t genStepRight(MartyCore& robot, int stepLength, int turn, float period,
                    char flags);
data_t genKickLeft(MartyCore& robot, float period);
data_t genKickRight(MartyCore& robot, float period);
data_t genGetUp(MartyCore& robot);
data_t genRaisedFootTwistLeft(MartyCore& robot, float period);
data_t genRaisedFootTwistRight(MartyCore& robot, float period);
data_t genCelebration(MartyCore& robot, float period);
data_t genReturnToZero(MartyCore& robot, float period);
bool setPointsLeanLeft(data_t& tSetpoints, int leanAmount, int legLift,
                       float period);
bool setPointsLeanRight(data_t& tSetpoints, int leanAmount, int legLift,
                        float period);
bool setPointsLeanForward(data_t& tSetpoints, int leanAmount, float period);
bool setPointsLeanBackward(data_t& tSetpoints, int leanAmount, float period);
bool setPointsLegsZero(data_t& tSetpoints, float period);
bool setPointsCrossLeftLeg(data_t& tSetpoints, float period);
bool setPointsCrossRightLeg(data_t& tSetpoints, float period);
bool setPointsLegsApart(data_t& tSetpoints, float period);
bool setPointsKickOutLeft(data_t& tSetpoints, float period);
bool setPointsKickOutRight(data_t& tSetpoints, float period);
bool setPointsFlickRight(data_t& tSetpoints, float period);
bool setPointsCircleACW(data_t& tSetpoints, float period);
bool setPointsCircleCW(data_t& tSetpoints, float period);
bool setPointsTapFR(data_t& tSetpoints, float period);
bool setPointsTapMR(data_t& tSetpoints, float period);
bool setPointsTapBR(data_t& tSetpoints, float period);
bool setPointsTapFL(data_t& tSetpoints, float period);
bool setPointsTapML(data_t& tSetpoints, float period);
bool setPointsTapBL(data_t& tSetpoints, float period);

bool setPointsArmsUp(data_t& tSetpoints, float amountRight, float amountLeft,
                     float period);
bool setPointsLeftArmUp(data_t& tSetpoints, float amount, float period);
bool setPointsRightArmUp(data_t& tSetpoints, float amount, float period);

bool setPointsEyes(data_t& tSetpoints, float targetPos, float period);

data_t combineTrajectories(data_t& t1, data_t& t2, vector<bool> ti);
data_t combineLegsArmsEyes(data_t& legs, data_t& arms, data_t& eyes);

int hipToBeSquare(MartyCore& robot, int robotID);
bool setPointsSkateLeft(data_t& tSetpoints, float amount, float period);
int rollerSkate(MartyCore& robot);

#endif  /* MARTY_TRAJECTORY_HPP */
