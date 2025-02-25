#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <memory>
#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>


constexpr char SUBSCRIPTION_TOPIC[] = "grip_val";
constexpr char MOVE_GROUP[] = "panda_arm"; //will be changed to parameter later
constexpr uint GRIPPER_JOINT_INDEX = 4; //will be changed to parameter later

const auto LOGGER = rclcpp::get_logger("gripper_publisher");

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node
{    
public:
    PositionPublisher() : Node("teleop_gripper_publisher", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(SUBSCRIPTION_TOPIC, 10, std::bind(&PositionPublisher::parseAndSendToMoveit, this, std::placeholders::_1));
    }

    void MoveItSetup();
private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    const moveit::core::JointModelGroup* joint_model_group_;
    
    std::vector<double> joint_positions_;

    void parseAndSendToMoveit(const std_msgs::msg::Float64& position);
};

//sets up MoveIt objects inside the class that require "this" pointer
void PositionPublisher::MoveItSetup()
{
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), MOVE_GROUP);
    joint_model_group_ = move_group_interface_->getCurrentState()->getJointModelGroup(MOVE_GROUP);
}

//parses message recieved on grip_val topic
void PositionPublisher::parseAndSendToMoveit(const std_msgs::msg::Float64& position)
{
    auto& gripper_val = position.data;
    RCLCPP_INFO(LOGGER, "recieved: %f", gripper_val);

    //copying curent joint positions(values)
    moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group_, joint_positions_);

    //setting gripper joint value
    joint_positions_[GRIPPER_JOINT_INDEX] = gripper_val;
    
    //checking if position is within bounds
    if(!move_group_interface_->setJointValueTarget(joint_positions_)) RCLCPP_WARN(LOGGER, "Target gripper joint position: %f, out of bounds", gripper_val);
    else
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        //executing plan if planning went well
        if(move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) move_group_interface_->execute(plan);
        else RCLCPP_WARN(LOGGER, "planning move to gripper joint position: %f FAILED", gripper_val);
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionPublisher>();
    node->MoveItSetup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
