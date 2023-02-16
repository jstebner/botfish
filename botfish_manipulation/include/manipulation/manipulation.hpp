#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace manip {

    struct cell_location {
        double x_dist;
        double y_dist;
    };

    class Manipulation : public rclcpp::Node {
    public:
        explicit Manipulation(rclcpp::NodeOptions options);

    private:
        std::optional<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> _engine_move_sub =
                std::nullopt;

        double _cell_offset{};
        double _grab_height{};
        double _goal_tolerance{};
        std::string _end_effector_link{};
        std::string _reference_link{};
        std::string _moveit_group{};
        geometry_msgs::msg::Quaternion _hand_orientation{};
        geometry_msgs::msg::Pose _starting_position{};
        geometry_msgs::msg::Pose _target_pose{};

        moveit::planning_interface::MoveGroupInterface *move_group_interface = nullptr;

        /// @brief Calculates the xyz/quaternion that the end effector of the robot needs to go to
        /// @param msg string msg containing the move outputted by the engine, format will be startingLocationEndingLocation ex: e2e4
        void calculate_trajectory(std_msgs::msg::String::SharedPtr &msg);

        /// @brief Converts string moves to xy locations
        /// @param move string move recieved in the calculate_trajectory callback
        /// @return tuple of two cell_location structs each containing two doubles for both x and y location
        std::tuple<cell_location, cell_location> parse_move(std::string move);
    };
}