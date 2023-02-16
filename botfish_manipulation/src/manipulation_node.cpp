#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "manipulation/manipulation.hpp"

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options) {
    _cell_offset = this->declare_parameter("cell_offset", 5.0);
    _end_effector_link = this->declare_parameter("end_effector", "right_arm_6_link");
    _reference_link = this->declare_parameter("reference_link", "right_arm_podent_link");
    _moveit_group = this->declare_parameter("move_group", "right_arm");
    _grab_height = this->declare_parameter("grab_height", 0.352);
    _goal_tolerance = this->declare_parameter("goal_tolerance", 0.0125);

    _engine_move_sub = this->create_subscription<std_msgs::msg::String>(
            "/engine_move", 10, std::bind(&Manipulation::move_cb, this, std::placeholders::_1));

    (*move_group_interface) = moveit::planning_interface::MoveGroupInterface(static_cast<const SharedPtr>(this),
                                                                             _moveit_group);

    move_group_interface->setPoseReferenceFrame(_reference_link);
    move_group_interface->setEndEffector(_end_effector_link);
    move_group_interface->setGoalTolerance(_goal_tolerance);
}

void manip::Manipulation::move_cb(std_msgs::msg::String::SharedPtr msg) {
    std::vector<manip::cell_location> parsed_moves = parse_move(msg->data);
    for (auto i: parsed_moves) {
        actuate(i);
        grab(true);
    }
}

std::vector<manip::cell_location> manip::Manipulation::parse_move(std::string move) {
    std::vector<manip::cell_location> parsed_moves;


    return parsed_moves;
}

void manip::Manipulation::actuate(manip::cell_location location) {
    moveit::planning_interface::MoveGroupInterface::Plan msg;

    //Calculate cell location to move to
    _target_pose.position.x = this->_starting_position.position.x + location.x_dist;
    _target_pose.position.y = this->_starting_position.position.y + location.y_dist;
    _target_pose.position.z = this->_starting_position.position.z;
    _target_pose.orientation = this->_hand_orientation;
    move_group_interface->setPoseTarget(_target_pose);

    //Attempt to plan to that position
    RCLCPP_INFO(this->get_logger(), "Planning movement...");
    auto const plan_ok = static_cast<bool>(move_group_interface->plan(msg));
    if (!plan_ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan!!!");
    } else {
        //Execute our calculated plan
        RCLCPP_INFO(this->get_logger(), "Executing movement...");
        auto const exec_ok = static_cast<bool>(move_group_interface->execute(msg));
        if (!exec_ok) {
            RCLCPP_ERROR(this->get_logger(), "Execute failed!!!");
        }
    }
}

void manip::Manipulation::grab(bool position) {

}
