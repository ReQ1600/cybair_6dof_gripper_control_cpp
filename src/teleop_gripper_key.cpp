//cpp headers
#include <memory>
#include <chrono>
#include <signal.h>
#include <termios.h>

//ros headers
#include <moveit/move_group_interface/move_group_interface.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#define LEFT 0x44
#define RIGHT 0x43

constexpr uint GRIPPER_JOINT_INDEX = 4;

using namespace std::chrono_literals;

struct termios new_terminal, old_terminal;

class TeleopGripperKey : public rclcpp::Node
{
public:
  TeleopGripperKey() : rclcpp::Node("teleop_gripper_key",   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("grip_angle", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&TeleopGripperKey::loop, this));
  }

  void SetupNode();

private:
  void loop();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  double angle_;

};

//the main loop of the program executed by a timer
void TeleopGripperKey::loop()
{
  bool publish = false;

  char c;
  if(::read(STDIN_FILENO, &c, 1) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "read() error");
    exit(-1);
  }

  RCLCPP_ERROR(this->get_logger(), "IM ALIVE");

  switch (c)
  {
  case LEFT:
    angle_ += 0.1;
    publish = true;
    break;
  case RIGHT:
    angle_ -= 0.1;
    publish = true;
    break;
  }
  if (publish)
  {
    // auto msg = std_msgs::msg::Float64();
    // msg.data = angle_;
    // publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "message published");

    std::vector<double> joint_vals = move_group_interface_->getCurrentJointValues();
    
    joint_vals[GRIPPER_JOINT_INDEX] = angle_;

    move_group_interface_->setJointValueTarget(joint_vals);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "moving joint %d [gripper] to %f position", GRIPPER_JOINT_INDEX, angle_);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "moving joint %d [gripper] to %f position plan FAILED", GRIPPER_JOINT_INDEX, angle_);
    }

    publish = false;
    //clears input buffer
    //for some reason i couldnt force it to accept only 1 character at the time and not buffer it so thats how i got around it (probably a skill issue on my part)
    
    tcflush(STDIN_FILENO, TCIFLUSH); 
  }
}

//sets up things that need the node pointer
void TeleopGripperKey::SetupNode()
{
  move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");//panda arm for tests only
  angle_ = move_group_interface_->getCurrentJointValues()[GRIPPER_JOINT_INDEX];

  this->get_logger() = this->get_logger();
}

void OnExit(int sig)
{
  (void) sig;
  tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
  puts("DYING");
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

  auto node = std::make_shared<TeleopGripperKey>();
  node->SetupNode();
  RCLCPP_INFO(node->get_logger(), "IM ALIVE");

  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
