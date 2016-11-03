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

#include <map>

#define DEBUG_MODE false

// Console printing MACROS
#define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";    // RED
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m";   // YELLOW
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";   // WHITE
#define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";  // BLUE
#define CLEAR() std::cerr << "\x1B[2J\x1B[H"; // Clear Console

enum JOINTS {
  RHIP = 0,
  RTWIST,
  RKNEE,
  LHIP,
  LTWIST,
  LKNEE,
  EYES,
  RARM,
  LARM,
  AUX1,
  AUX2,
  NUMJOINTS // Number of Joints, Leave at the end!
};

const std::vector <std::string> JOINT_NAMES = {
  "RHIP",
  "RTWIST",
  "RKNEE",
  "LHIP",
  "LTWIST",
  "LKNEE",
  "EYES",
  "RARM",
  "LARM",
  "AUX1",
  "AUX2"
};

const std::map<std::string, int> NAMES = {
  {"RHIP", RHIP},
  {"RTWIST", RTWIST},
  {"RKNEE", RKNEE},
  {"LHIP", LHIP},
  {"LTWIST", LTWIST},
  {"LKNEE", LKNEE},
  {"EYES", EYES},
  {"RARM", RARM},
  {"LARM", LARM},
  {"AUX1", AUX1},
  {"AUX2", AUX2}
};

# define HIPOFFSET 100
# define TWISTOFFSET 110
# define KNEEOFFSET 110
# define ARMOFFSET 110
# define EYESOFFSET 100
# define AUXOFFSET 110

// Servo polarity and multiplier
// #define RHIPDIR   1
// #define RHIPMULT  0.5

// #define RTWISTDIR   1
// #define RTWISTMULT  0.5

// #define RKNEEDIR  -1
// #define RKNEEMULT 1

// #define LHIPDIR   1
// #define LHIPMULT  0.5

// #define LTWISTDIR   1
// #define LTWISTMULT  0.5

// #define LKNEEDIR  -1
// #define LKNEEMULT 0.5

// #define LARMDIR   1
// #define LARMMULT    0.5

// #define RARMDIR   -1
// #define RARMMULT    0.5

// #define AUX1DIR   1
// #define AUX1MULT  0.5

// #define EYESDIR   1
// #define EYESMULT  0.5

// #define AUX2DIR   1
// #define AUX2MULT  0.5

// These are "angles" rather than servo commands
#define EYESNORMAL  0
#define EYESANGRY   22
#define EYESEXCITED -20
#define EYESWIDE  -50

#endif  /* MARTY_DEFINITIONS_HPP */
