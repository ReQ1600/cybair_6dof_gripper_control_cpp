#include <memory>
#include <chrono>
#include <signal.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#define LEFT 0x44
#define RIGHT 0x43

#define GRIPPER_TOPIC "grip_val"
#define INCEREMENT_SPEED 0.005

using namespace std::chrono_literals;

struct termios new_terminal, old_terminal;

class TeleopGripperKey : public rclcpp::Node
{
public:
  TeleopGripperKey() : rclcpp::Node("teleop_gripper_key")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(GRIPPER_TOPIC, 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&TeleopGripperKey::Loop, this));
  }

  void Loop();

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double angle_ = 0;

};

void TeleopGripperKey::Loop()
{
  bool publish = false;

  char c;
  if(::read(STDIN_FILENO, &c, 1) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "read() error");
    exit(-1);
  }

  switch (c)
  {
  case LEFT:
    angle_ += INCEREMENT_SPEED;
    publish = true;
    break;
  case RIGHT:
    angle_ -= INCEREMENT_SPEED;
    publish = true;
    break;
  }
  if (publish)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = angle_;
    publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "message published");
    publish = false;

    //clears input buffer
    //for some reason i couldnt force it to accept only 1 character at the time and not buffer it so thats how i got around it (probably a skill issue on my part)
    tcflush(STDIN_FILENO, TCIFLUSH); 
  }
}

void OnExit(int sig)
{
  (void) sig;
  tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  puts("Teleoperation with keyboard");
  puts("Use left and right arrows to increase or decrease servo angle");
  puts("=============================================");
  //setting up terminal to nonblocking mode
  tcgetattr(STDIN_FILENO, &old_terminal);
  memcpy(&new_terminal, &old_terminal, sizeof(struct termios));

  new_terminal.c_lflag &= ~(ICANON | ECHO);
  new_terminal.c_cc[VEOL] = 1;
  new_terminal.c_cc[VEOF] = 2;

  tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);

  //returning terminal to normal on interrupt
  signal(SIGINT, OnExit);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopGripperKey>());
  rclcpp::shutdown();
  
  return 0;
}