/**
 * @file      calibration.hpp
 * @brief     Calibration class for calibrating Marty's servos
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */
// ****************************************************************************
// **WARNING: DO NOT CHANGE ANYTHING BELOW UNLESS YOU KNOW WHAT YOU'RE DOING!**
// ****************************************************************************

#ifndef MARTY_DEFINITIONS_HPP
#define MARTY_DEFINITIONS_HPP

#include <unistd.h>
#include <map>

#define DEBUG_MODE false

// Console printing MACROS
#define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";    // RED
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m";   // YELLOW
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";   // WHITE
#define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";  // BLUE
#define CLEAR() std::cerr << "\x1B[2J\x1B[H"; // Clear Console

inline void sleepms(int ms) {usleep(ms * 1000);}

enum JOINTS {
  LHIP = 0,
  LTWIST,
  LKNEE,
  RHIP,
  RTWIST,
  RKNEE,
  LARM,
  RARM,
  EYES,
  AUX1,
  AUX2,
  NUMJOINTS // Number of Joints, Leave at the end!
};

const std::vector <std::string> NAMES = {
  "LHIP",
  "LTWIST",
  "LKNEE",
  "RHIP",
  "RTWIST",
  "RKNEE",
  "LARM",
  "RARM",
  "EYES",
  "AUX1",
  "AUX2"
};

const std::vector <int> JOINT_DIR = {
  1,  //   "LHIP",
  1,  //   "LTWIST",
  -1,  //   "LKNEE",
  1,  //   "RHIP",
  1,  //   "RTWIST",
  -1,  //   "RKNEE",
  1,  //   "LARM",
  -1,  //   "RARM",
  1,  //   "EYES",
  1,  //   "AUX1",
  1   //   "AUX2"
};

const std::vector <int> JOINT_MULT = {
  1,  //   "LHIP",
  1,  //   "LTWIST",
  1,  //   "LKNEE",
  1,  //   "RHIP",
  1,  //   "RTWIST",
  1,  //   "RKNEE",
  2,  //   "LARM",
  2,  //   "RARM",
  1,  //   "EYES",
  1,  //   "AUX1",
  1   //   "AUX2"
};

const std::map<std::string, int> JOINT_NAMES = {
  {"LHIP", LHIP},
  {"LTWIST", LTWIST},
  {"LKNEE", LKNEE},
  {"RHIP", RHIP},
  {"RTWIST", RTWIST},
  {"RKNEE", RKNEE},
  {"LARM", LARM},
  {"RARM", RARM},
  {"EYES", EYES},
  {"AUX1", AUX1},
  {"AUX2", AUX2}
};

# define HIP_OFFSET 100
# define TWIST_OFFSET 110
# define KNEE_OFFSET 110
# define ARM_OFFSET 300
# define EYES_OFFSET 300
# define AUX_OFFSET 110

// TODO: These should not be definitions, but in joint calibration as rosparams!
#define EYES_ANGRY   30
#define EYES_NORMAL  0
#define EYES_EXCITED -40
#define EYES_WIDE  -120

#endif  /* MARTY_DEFINITIONS_HPP */
