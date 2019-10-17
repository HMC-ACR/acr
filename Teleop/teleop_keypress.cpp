#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopTurtle {
public:
  TeleopTurtle();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double angular_des_, angular_current_;
  double linear_des_, linear_current_;
  double l_scale_, a_scale_;
  geometry_msgs::Twist twist_;
  ros::Publisher twist_pub_;
};

TeleopTurtle::TeleopTurtle()
    : linear_des_(0), linear_current_(0), angular_des_(0), angular_current_(0),
      l_scale_(120), a_scale_(120) {
          // We scale and publish in velocity / angle values in the range [-120,120]. 
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear_up", l_scale_, l_scale_);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT, quit);

  teleop_turtle.keyLoop();

  return (0);
}

void TeleopTurtle::keyLoop() {
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");

  for (;;) {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    switch (c) {
    case KEYCODE_L:
      ROS_DEBUG("LEFT");
      angular_des_ += 0.25;
      dirty = true;
      break;
    case KEYCODE_R:
      ROS_DEBUG("RIGHT");
      angular_des_ += -0.25;
      dirty = true;
      break;
    case KEYCODE_U:
      ROS_DEBUG("UP");
      linear_des_ = 1.0;
      break;
    case KEYCODE_D:
      ROS_DEBUG("DOWN");
      linear_des_ = -1.0;
      break;
    }

    angular_current_ = angular_des_;
    // Ensuring the angular velocity we're publishing is within bounds.
    angular_current_ = min(1.0, angular_current_);
    angular_current_ = max(-1.0, angular_current_);

    if (linear_des_ > linear_current_) {
      dirty = true;
      // Increasing by 0.10 so it takes a full second to get to full speed.
      linear_current_ = linear_current_ + 0.10; 
    } else if (linear_des_ < linear_current_) {
      dirty = true;
      // Decreasing by 0.25 incerments means it takes 400ms to stop.
      linear_current_ = linear_current_ - 0.25;
    }
    // Ensuring the linear velocity we're publishing is within bounds.
    linear_current_ = min(1.0, linear_current_);
    linear_current_ = max(-1.0, linear_current_);

    if (dirty == true) {
      geometry_msgs::Twist twist;
      // Update linear and angular twist values.
      twist.angular.z = angular_current_ * a_scale_;
      twist.linear.x = linear_current_ * l_scale_;
      // Publish updated twist.
      twist_pub_.publish(twist);
      dirty = false;
    }
  }

  return;
}
