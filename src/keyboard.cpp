/**
 * @file      keyboard.cpp
 * @brief     Marty App for controlling Marty using a keyboard
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2016-02-06
 * @copyright (Apache) 2016 Robotical Ltd.
 */

// MARTY
#include <ros_marty/keyboard.hpp>

Keyboard::Keyboard(ros::NodeHandle& nh) : nh_(nh) {
  // this->loadParams();
  this->init();
  // this->rosSetup();
  ROS_DEBUG("Marty Keyboard App Ready!");
}

Keyboard::~Keyboard() {
}


void Keyboard::init() {
  srv.request.data.push_back(marty_msgs::Command::Request::CMD_WALK);
  srv.request.data.push_back(1);  // Num Steps
  srv.request.data.push_back(0); // Step Length
  srv.request.data.push_back(0); // Turn 0 = Straight, >0 = Right, <0 = Left
  srv.request.data.push_back(25); // Move Time
}

void Keyboard::rosSetup() {
  key_up_sub_ = nh_.subscribe("/keyboard/keyup", 1000, &Keyboard::keyUpCB, this);
  key_down_sub_ = nh_.subscribe("/keyboard/keydown", 1000, &Keyboard::keyDownCB,
                                this);
  ros::service::waitForService("/marty/cmd_server/command");
  send_cmd_ = nh_.serviceClient<marty_msgs::Command>("/marty/cmd_server/command");
}

void Keyboard::keyUpCB(const keyboard::Key::ConstPtr& msg) {
  uint16_t key = msg->code;
  if (key == keyboard::Key::KEY_UP) { forw_ = false; }
  else if (key == keyboard::Key::KEY_DOWN) { back_ = false; }
  else if (key == keyboard::Key::KEY_LEFT) { left_ = false; }
  else if (key == keyboard::Key::KEY_RIGHT) { right_ = false; }
}

void Keyboard::keyDownCB(const keyboard::Key::ConstPtr& msg) {
  uint16_t key = msg->code;
  if (key == keyboard::Key::KEY_UP) { forw_ = true; }
  else if (key == keyboard::Key::KEY_DOWN) { back_ = true; }
  else if (key == keyboard::Key::KEY_LEFT) { left_ = true; }
  else if (key == keyboard::Key::KEY_RIGHT) { right_ = true; }
}

void Keyboard::run() {
  // srv.request.data[0] = marty_msgs::Command::Request::CMD_WALK;
  // srv.request.data[1] = 1;  // Num Steps
  // srv.request.data[2] = 50; // Step Length
  // srv.request.data[4] = 25; // Move Time
  // if (forw_) {;}
  srv.request.data[3] = 0;
  if (forw_) { srv.request.data[2] += 50; }
  if (back_) { srv.request.data[2] += -50; }
  if (left_) { srv.request.data[3] += -5; }
  if (right_) { srv.request.data[3] += 5; }
  if (forw_ || back_ || left_ || right_) {
    if (send_cmd_.call(srv)) {
      if (srv.response.success) {ROS_INFO("SUCCESS!");} else {ROS_WARN("NAY!");}
    } else { ROS_ERROR("Failed to call send cmd service!"); }
  }
}

// void Keyboard::run() {
// if ((c_ = getch())) {
//   ROS_INFO_STREAM("Char: " << c_);
//   // if (c_ == 'q') {  }
// }
// new_event = false;

// SDL_Event event;
// if (SDL_PollEvent(&event)) {
//   switch (event.type) {
//   case SDL_KEYUP:
//     pressed = false;
//     code = event.key.keysym.sym;
//     modifiers = event.key.keysym.mod;
//     new_event = true;
//     break;
//   case SDL_KEYDOWN:
//     pressed = true;
//     code = event.key.keysym.sym;
//     modifiers = event.key.keysym.mod;
//     new_event = true;
//     break;
//   case SDL_QUIT:
//     return false;
//     break;
//   }
// }
// }

// char Keyboard::getch() {
//   char buf = 0;
//   struct termios old = {0};
//   if (tcgetattr(0, &old) < 0)
//     perror("tcsetattr()");
//   old.c_lflag &= ~ICANON;
//   old.c_lflag &= ~ECHO;
//   old.c_cc[VMIN] = 1;
//   old.c_cc[VTIME] = 0;
//   if (tcsetattr(0, TCSANOW, &old) < 0)
//     perror("tcsetattr ICANON");
//   if (read(0, &buf, 1) < 0)
//     perror ("read()");
//   old.c_lflag |= ICANON;
//   old.c_lflag |= ECHO;
//   if (tcsetattr(0, TCSADRAIN, &old) < 0)
//     perror ("tcsetattr ~ICANON");
//   return (buf);
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle nh("~");

  Keyboard keyboard(nh);
  ros::Rate r(1);

  while (ros::ok()) {
    keyboard.run();
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();
  return 0;
}
