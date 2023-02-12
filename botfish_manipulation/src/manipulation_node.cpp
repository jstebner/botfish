#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "manipulation/manipulation.hpp"

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options){
    _cell_offset = this->declare_parameter("cell_offset", 5.0);
    _end_effector_link = this->declare_parameter("end_effector", "right_arm_6_link");
    _reference_link = this->declare_parameter("reference_link", "right_arm_podent_link");
    _moveit_group = this->declare_parameter("move_group", "right_arm");

    _engine_move_sub = this->create_subscription<std_msgs::msg::String>(
        "/engine_move", 10, std::bind(&Manipulation::calculate_trajectory, this, std::placeholders::_1));

    auto const logger = rclcpp::get_logger("manipulation");

    auto manip_node = std::make_shared<manip::Manipulation>(options);

    //moveit::planning_interface::MoveGroupInterface move_group(manip_node, _moveit_group);
    (*move_group_interface) = moveit::planning_interface::MoveGroupInterface(manip_node, _moveit_group);


}

void manip::Manipulation::calculate_trajectory(std_msgs::msg::String::SharedPtr msg){


}

std::tuple<manip::cell_location, manip::cell_location> manip::Manipulation::parse_move(std::string move){

}